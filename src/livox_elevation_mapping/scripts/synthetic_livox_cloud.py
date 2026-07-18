#!/usr/bin/python3

import math
import random

import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField


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


class SyntheticLivoxCloud:
    def __init__(self):
        self.ground_truth_topic = rospy.get_param("~ground_truth_topic", rospy.get_param("~odom_topic", "/ground_truth/state"))
        self.target_odom_topic = rospy.get_param("~target_odom_topic", "/odom")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/livox/lidar")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.max_range = rospy.get_param("~max_range", 6.0)
        self.lidar_x = rospy.get_param("~lidar_x", 0.14)
        self.lidar_y = rospy.get_param("~lidar_y", 0.0)
        self.lidar_z = rospy.get_param("~lidar_z", 0.03)
        self.lidar_pitch = rospy.get_param("~lidar_pitch", math.radians(30.0))
        self.lidar_yaw = rospy.get_param("~lidar_yaw", math.pi)

        self.world_robot_x = 0.0
        self.world_robot_y = 0.0
        self.world_robot_z = 0.5
        self.world_robot_yaw = 0.0
        self.odom_robot_x = 0.0
        self.odom_robot_y = 0.0
        self.odom_robot_z = 0.5
        self.odom_robot_yaw = 0.0
        self.have_ground_truth = False
        self.have_target_odom = False

        self.obstacle_points = self.build_world_points()
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        self.fields = fields

        self.pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        self.ground_truth_sub = rospy.Subscriber(self.ground_truth_topic, Odometry, self.ground_truth_callback, queue_size=1)
        self.target_odom_sub = rospy.Subscriber(self.target_odom_topic, Odometry, self.target_odom_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self.publish)
        rospy.loginfo(
            "Synthetic Livox cloud publishing %s from Gazebo geometry, converting %s into %s",
            self.cloud_topic,
            self.ground_truth_topic,
            self.target_odom_topic,
        )

    def ground_truth_callback(self, msg):
        self.world_robot_x = msg.pose.pose.position.x
        self.world_robot_y = msg.pose.pose.position.y
        self.world_robot_z = msg.pose.pose.position.z
        self.world_robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_ground_truth = True

        if not self.have_target_odom:
            self.odom_robot_x = self.world_robot_x
            self.odom_robot_y = self.world_robot_y
            self.odom_robot_z = self.world_robot_z
            self.odom_robot_yaw = self.world_robot_yaw

    def target_odom_callback(self, msg):
        self.odom_robot_x = msg.pose.pose.position.x
        self.odom_robot_y = msg.pose.pose.position.y
        self.odom_robot_z = msg.pose.pose.position.z
        self.odom_robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_target_odom = True

    def build_world_points(self):
        points = []
        points.extend(self.box_points(2.6, 0.0, 0.125, 0.45, 0.35, 0.25, 0.035))
        points.extend(self.cylinder_points(3.4, 0.65, 0.45, 0.08, 0.9, 48, 18))
        points.extend(self.cylinder_points(3.4, -0.65, 0.45, 0.08, 0.9, 48, 18))
        points.extend(self.box_points(2.6, -1.45, 0.80, 2.2, 0.06, 0.08, 0.05))
        points.extend(self.box_points(2.6, -1.45, 0.40, 2.2, 0.05, 0.06, 0.05))
        for px in (1.65, 2.6, 3.55):
            points.extend(self.box_points(px, -1.45, 0.35, 0.07, 0.07, 0.9, 0.04))
        points.extend(self.ground_points())
        return points

    def ground_points(self):
        points = []
        step = 0.18
        x = -1.0
        while x <= 5.0:
            y = -2.5
            while y <= 2.5:
                points.append((x, y, 0.0))
                y += step
            x += step
        return points

    def box_points(self, cx, cy, cz, sx, sy, sz, step):
        points = []
        nx = max(2, int(sx / step))
        ny = max(2, int(sy / step))
        nz = max(2, int(sz / step))
        for ix in range(nx + 1):
            x = cx - sx * 0.5 + sx * ix / nx
            for iy in range(ny + 1):
                y = cy - sy * 0.5 + sy * iy / ny
                points.append((x, y, cz + sz * 0.5))
                points.append((x, y, cz - sz * 0.5))
        for ix in range(nx + 1):
            x = cx - sx * 0.5 + sx * ix / nx
            for iz in range(nz + 1):
                z = cz - sz * 0.5 + sz * iz / nz
                points.append((x, cy + sy * 0.5, z))
                points.append((x, cy - sy * 0.5, z))
        for iy in range(ny + 1):
            y = cy - sy * 0.5 + sy * iy / ny
            for iz in range(nz + 1):
                z = cz - sz * 0.5 + sz * iz / nz
                points.append((cx + sx * 0.5, y, z))
                points.append((cx - sx * 0.5, y, z))
        return points

    def cylinder_points(self, cx, cy, cz, radius, height, angle_steps, height_steps):
        points = []
        for ia in range(angle_steps):
            angle = 2.0 * math.pi * ia / angle_steps
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            for iz in range(height_steps + 1):
                z = cz - height * 0.5 + height * iz / height_steps
                points.append((x, y, z))
        return points

    def publish(self, _event):
        if not self.have_ground_truth:
            return

        cloud_points = []
        for wx, wy, wz in self.obstacle_points:
            rel_world_x = wx - self.world_robot_x
            rel_world_y = wy - self.world_robot_y
            rel_world_z = wz - self.world_robot_z

            body_x, body_y = rotate_z(rel_world_x, rel_world_y, -self.world_robot_yaw)
            body_z = rel_world_z

            lidar_body_x = body_x - self.lidar_x
            lidar_body_y = body_y - self.lidar_y
            lidar_body_z = body_z - self.lidar_z

            yx, yy = rotate_z(lidar_body_x, lidar_body_y, -self.lidar_yaw)
            lx, ly, lz = rotate_y(yx, yy, lidar_body_z, -self.lidar_pitch)
            distance = math.sqrt(lx * lx + ly * ly + lz * lz)
            if distance > self.max_range or distance < 0.15:
                continue

            odom_rel_x, odom_rel_y = rotate_z(body_x, body_y, self.odom_robot_yaw)
            odom_x = self.odom_robot_x + odom_rel_x
            odom_y = self.odom_robot_y + odom_rel_y
            odom_z = self.odom_robot_z + body_z

            noise = 0.004
            if self.frame_id == "odom":
                cloud_points.append((
                    odom_x + random.uniform(-noise, noise),
                    odom_y + random.uniform(-noise, noise),
                    odom_z + random.uniform(-noise, noise),
                    max(20.0, 255.0 - distance * 25.0),
                ))
            else:
                cloud_points.append((
                    lx + random.uniform(-noise, noise),
                    ly + random.uniform(-noise, noise),
                    lz + random.uniform(-noise, noise),
                    max(20.0, 255.0 - distance * 25.0),
                ))

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        self.pub.publish(pc2.create_cloud(header, self.fields, cloud_points))


if __name__ == "__main__":
    rospy.init_node("synthetic_livox_cloud")
    SyntheticLivoxCloud()
    rospy.spin()
