#!/usr/bin/python3

import math
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_z(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def rotate_y(x, y, z, pitch):
    c = math.cos(pitch)
    s = math.sin(pitch)
    return c * x + s * z, y, -s * x + c * z


def height_color(height, min_height, max_height):
    span = max(max_height - min_height, 1e-3)
    t = max(0.0, min(1.0, (height - min_height) / span))
    if t < 0.5:
        local = t * 2.0
        return ColorRGBA(0.1 + 0.6 * local, 0.85, 0.15, 0.85)
    local = (t - 0.5) * 2.0
    return ColorRGBA(0.75 + 0.25 * local, 0.85 * (1.0 - local), 0.08, 0.9)


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round(max(0.0, min(1.0, fraction)) * (len(ordered) - 1)))
    return ordered[index]


def variance(values, mean):
    if len(values) < 2:
        return 0.0
    return sum((value - mean) * (value - mean) for value in values) / len(values)


class LocalElevationMapper:
    def __init__(self):
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/livox/lidar")
        self.odom_topic = rospy.get_param("~odom_topic", "/ground_truth/state")
        self.map_frame = rospy.get_param("~map_frame", "odom")

        self.lidar_x = rospy.get_param("~lidar_x", 0.14)
        self.lidar_y = rospy.get_param("~lidar_y", 0.0)
        self.lidar_z = rospy.get_param("~lidar_z", 0.03)
        self.lidar_pitch = rospy.get_param("~lidar_pitch", math.radians(30.0))
        self.lidar_yaw = rospy.get_param("~lidar_yaw", math.pi)

        self.map_length = rospy.get_param("~map_length", 6.0)
        self.resolution = rospy.get_param("~resolution", 0.10)
        self.min_range = rospy.get_param("~min_range", 0.20)
        self.max_range = rospy.get_param("~max_range", self.map_length)
        self.min_height = rospy.get_param("~min_height", -0.30)
        self.max_height = rospy.get_param("~max_height", 2.0)
        self.ground_quantile = rospy.get_param("~ground_quantile", 0.20)
        self.obstacle_height_threshold = rospy.get_param("~obstacle_height_threshold", 0.18)
        self.publish_rate = rospy.get_param("~publish_rate", 8.0)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.last_cloud = None
        self.lock = threading.Lock()

        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.cloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.cloud_callback, queue_size=1, buff_size=8 * 1024 * 1024)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self.publish)
        rospy.loginfo("Local elevation mapper listening to %s and %s", self.pointcloud_topic, self.odom_topic)

    def odom_callback(self, msg):
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_z = msg.pose.pose.position.z
            self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def cloud_callback(self, msg):
        with self.lock:
            self.last_cloud = msg

    def publish(self, _event):
        with self.lock:
            cloud = self.last_cloud
            robot_x = self.robot_x
            robot_y = self.robot_y
            robot_z = self.robot_z
            robot_yaw = self.robot_yaw

        if cloud is None:
            return

        half = self.map_length * 0.5
        cells = {}

        for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            lx, ly, lz = point[:3]
            if cloud.header.frame_id == self.map_frame:
                wx = lx
                wy = ly
                wz = lz
                dx = wx - robot_x
                dy = wy - robot_y
                bx, by = rotate_z(dx, dy, -robot_yaw)
                bz = wz - robot_z
                lidar_range = math.sqrt(dx * dx + dy * dy + bz * bz)
            else:
                lidar_range = math.sqrt(lx * lx + ly * ly + lz * lz)
                px, py, pz = rotate_y(lx, ly, lz, self.lidar_pitch)
                bx, by = rotate_z(px, py, self.lidar_yaw)
                bx += self.lidar_x
                by += self.lidar_y
                bz = pz + self.lidar_z
                gx, gy = rotate_z(bx, by, robot_yaw)
                wx = robot_x + gx
                wy = robot_y + gy
                wz = robot_z + bz

            if lidar_range < self.min_range or lidar_range > self.max_range:
                continue

            if bx < -half or bx > half or by < -half or by > half:
                continue
            if bz < self.min_height or bz > self.max_height:
                continue

            ix = int(math.floor((bx + half) / self.resolution))
            iy = int(math.floor((by + half) / self.resolution))
            key = (ix, iy)
            if key not in cells:
                cells[key] = []
            cells[key].append((wx, wy, wz))

        elevation_cells, obstacle_cells = self.estimate_cells(cells)

        markers = MarkerArray()
        now = rospy.Time.now()
        markers.markers.append(self.make_clear_marker())
        markers.markers.append(self.make_elevation_marker(elevation_cells, now))
        markers.markers.append(self.make_obstacle_marker(obstacle_cells, now))
        markers.markers.append(self.make_robot_marker(robot_x, robot_y, robot_z, robot_yaw, now))
        markers.markers.append(self.make_boundary_marker(robot_x, robot_y, robot_z, robot_yaw, half, now))
        self.marker_pub.publish(markers)

    def estimate_cells(self, cells):
        elevation_cells = []
        obstacle_cells = []
        for samples in cells.values():
            if not samples:
                continue
            xs = [sample[0] for sample in samples]
            ys = [sample[1] for sample in samples]
            zs = [sample[2] for sample in samples]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            ground_z = percentile(zs, self.ground_quantile)
            max_z = max(zs)
            mean_z = sum(zs) / len(zs)
            var_z = variance(zs, mean_z)
            elevation_cells.append((cx, cy, ground_z, var_z, len(zs)))
            if max_z - ground_z > self.obstacle_height_threshold:
                obstacle_cells.append((cx, cy, max_z, max_z - ground_z))
        return elevation_cells, obstacle_cells

    def make_clear_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def make_elevation_marker(self, cells, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "elevation_cells"
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution * 0.92
        marker.scale.y = self.resolution * 0.92
        marker.scale.z = 0.035
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.12)

        for wx, wy, wz, var_z, _count in cells:
            marker.points.append(Point(wx, wy, wz))
            if var_z > 0.015:
                marker.colors.append(ColorRGBA(0.95, 0.75, 0.1, 0.88))
            else:
                marker.colors.append(height_color(wz, self.min_height, self.max_height))
        return marker

    def make_obstacle_marker(self, cells, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "obstacle_cells"
        marker.id = 4
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.resolution * 0.65
        marker.scale.y = self.resolution * 0.65
        marker.scale.z = 0.08
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.12)

        for wx, wy, wz, height_above_ground in cells:
            marker.points.append(Point(wx, wy, wz))
            alpha = max(0.45, min(1.0, height_above_ground))
            marker.colors.append(ColorRGBA(1.0, 0.12, 0.05, alpha))
        return marker

    def make_robot_marker(self, robot_x, robot_y, robot_z, robot_yaw, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "robot_pose"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.45
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color = ColorRGBA(0.1, 0.35, 1.0, 1.0)
        marker.pose.position.x = robot_x
        marker.pose.position.y = robot_y
        marker.pose.position.z = robot_z + 0.25
        marker.pose.orientation.z = math.sin(robot_yaw * 0.5)
        marker.pose.orientation.w = math.cos(robot_yaw * 0.5)
        marker.lifetime = rospy.Duration(0.12)
        return marker

    def make_boundary_marker(self, robot_x, robot_y, robot_z, robot_yaw, half, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = "local_map_boundary"
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color = ColorRGBA(0.2, 0.7, 1.0, 0.9)
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.12)

        corners = [(-half, -half), (half, -half), (half, half), (-half, half), (-half, -half)]
        for cx, cy in corners:
            rx, ry = rotate_z(cx, cy, robot_yaw)
            marker.points.append(Point(robot_x + rx, robot_y + ry, robot_z + 0.04))
        return marker


if __name__ == "__main__":
    rospy.init_node("livox_local_elevation_mapper")
    LocalElevationMapper()
    rospy.spin()
