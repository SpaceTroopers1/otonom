import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rover_msgs.msg import EncoderMsg
import tf_transformations
import tf2_ros
import math
import numpy as np
import struct

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.subscription = self.create_subscription(
            EncoderMsg,
            '/encoder',
            self.update_odometry,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Robot parameters
        self.WHEEL_BASE = 0.94  # Distance between left and right wheels
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def update_odometry(self, msg: EncoderMsg):
        # Simulated wheel speeds
        V_left = (msg.m1 + msg.m3) /2 # Left wheels (m/s)
        V_right = (msg.m2 + msg.m4) /2# Right wheels (m/s)

        V = (V_left + V_right) / 2
        omega = (V_right - V_left) / (self.WHEEL_BASE *2)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert ns to seconds
        self.last_time = current_time

        # Update pose using odometry equations
        self.theta += omega * dt
        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt


        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quaternion = Quaternion()
        quaternion.x, quaternion.y, quaternion.z, quaternion.w = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = quaternion

        # Publish TF: base_link -> odom
        t_base = TransformStamped()
        t_base.header.stamp = self.get_clock().now().to_msg()
        t_base.header.frame_id = "odom"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = self.x
        t_base.transform.translation.y = self.y
        t_base.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t_base)

        # Publish TF: sensor_link -> base_link (offset in front of the robot)
        t_sensor = TransformStamped()
        t_sensor.header.stamp = self.get_clock().now().to_msg()
        t_sensor.header.frame_id = "base_link"
        t_sensor.child_frame_id = "camera_depth_optical_frame"
        t_sensor.transform.translation.x = 0.2  # 20cm in front of base_link
        t_sensor.transform.translation.y = 0.0
        t_sensor.transform.translation.z = 0.1  # Slightly above ground
        q = tf_transformations.quaternion_from_euler(-1.57, 0, -1.57)  # Rotate -90° (or -π/2) around Y-axis
        t_sensor.transform.rotation.x = q[0]
        t_sensor.transform.rotation.y = q[1]
        t_sensor.transform.rotation.z = q[2]
        t_sensor.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t_sensor)

        self.odom_pub.publish(odom_msg)

def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

