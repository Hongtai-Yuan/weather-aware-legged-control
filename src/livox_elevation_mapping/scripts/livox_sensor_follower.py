#!/usr/bin/python3

import math

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def rotate_z(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class LivoxSensorFollower:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "livox_mid360_sensor")
        self.odom_topic = rospy.get_param("~odom_topic", "/ground_truth/state")
        self.parent_frame = rospy.get_param("~parent_frame", "world")
        self.tf_parent_frame = rospy.get_param("~tf_parent_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "livox_mid360")
        self.lidar_x = rospy.get_param("~lidar_x", 0.10)
        self.lidar_y = rospy.get_param("~lidar_y", 0.0)
        self.lidar_z = rospy.get_param("~lidar_z", 0.03)
        self.lidar_yaw = rospy.get_param("~lidar_yaw", math.pi)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.last_update = rospy.Time(0)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.loginfo("Livox sensor follower keeping %s aligned with robot pose", self.model_name)

    def odom_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_update).to_sec() < 0.02:
            return
        self.last_update = now

        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        robot_z = msg.pose.pose.position.z
        robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        dx, dy = rotate_z(self.lidar_x, self.lidar_y, robot_yaw)
        lidar_x = robot_x + dx
        lidar_y = robot_y + dy
        lidar_z = robot_z + self.lidar_z
        lidar_yaw = robot_yaw + self.lidar_yaw
        qx, qy, qz, qw = quaternion_from_yaw(lidar_yaw)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = self.parent_frame
        state.pose.position.x = lidar_x
        state.pose.position.y = lidar_y
        state.pose.position.z = lidar_z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        try:
            self.set_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to move Livox sensor model: %s", exc)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.tf_parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = lidar_x
        tf_msg.transform.translation.y = lidar_y
        tf_msg.transform.translation.z = lidar_z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)


if __name__ == "__main__":
    rospy.init_node("livox_sensor_follower")
    LivoxSensorFollower()
    rospy.spin()
