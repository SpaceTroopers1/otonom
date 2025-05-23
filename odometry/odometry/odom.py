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
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import struct
from message_filters import Subscriber, ApproximateTimeSynchronizer


odom_msg = Odometry()

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publisher for odometry data
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.encoder_left_sub = Subscriber(self, EncoderMsg, '/encoder/left')  # Left wheels
        self.encoder_right_sub = Subscriber(self, EncoderMsg, '/encoder/right')

        # Parameters
        self.declare_parameter('update_rate', 10)  # Hz
        self.declare_parameter('noise_level', 0.00)  # Maximum noise in meters/radians
        
        self.ts = ApproximateTimeSynchronizer(
            [self.encoder_left_sub, self.encoder_right_sub], queue_size=10, slop=0.05,
            allow_headerless = True
        )
        
        self.ts.registerCallback(self.publish_odometry_and_transforms)
        
        self.update_rate = self.get_parameter('update_rate').value
        self.noise_level = self.get_parameter('noise_level').value

        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.WHEEL_BASE = 0.94  # Distance between left and right wheels
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(1/10 , self.timer_callback)
        # Publish static transforms
        self.publish_static_transforms()
        
        
    def timer_callback(self):
        self.odom_publisher.publish(odom_msg)
        
        
    def publish_static_transforms(self):
        # Static transform: base_footprint -> base_link
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.0
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0

        # Static transform: base_link -> laser_frame
        base_link_to_laser_frame = TransformStamped()
        base_link_to_laser_frame.header.stamp = self.get_clock().now().to_msg()
        base_link_to_laser_frame.header.frame_id = 'base_link'
        base_link_to_laser_frame.child_frame_id = 'laser_frame'
        base_link_to_laser_frame.transform.translation.x = 0.0
        base_link_to_laser_frame.transform.translation.y = 0.0
        base_link_to_laser_frame.transform.translation.z = 0.2  # Adjust as needed
        base_link_to_laser_frame.transform.rotation.x = 0.0
        base_link_to_laser_frame.transform.rotation.y = 0.0
        base_link_to_laser_frame.transform.rotation.z = 0.0
        base_link_to_laser_frame.transform.rotation.w = 1.0

        # Broadcast static transforms
        self.static_broadcaster.sendTransform([base_footprint_to_base_link, base_link_to_laser_frame])

    def publish_odometry_and_transforms(self, encoder_left_msg: EncoderMsg, encoder_right_msg: EncoderMsg):
        current_time = self.get_clock().now()

        V_left = (encoder_left_msg.l_front + encoder_left_msg.l_back) / 2
        V_right = (encoder_right_msg.r_front + encoder_right_msg.r_back) / 2

        V = (V_left + V_right) / 2
        omega = (V_right - V_left) / (self.WHEEL_BASE * 2)

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.x += V * math.cos(self.theta) * dt
        self.y += V * math.sin(self.theta) * dt
        self.theta += omega * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

    # Make odometry less confident than IMU
    # X, Y, and yaw covariances relaxed to reduce confidence
        odom_msg.pose.covariance = [
        0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e-5, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e12, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e12, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1  # Yaw
    ]

        odom_msg.twist.twist.linear.x = V * math.cos(self.theta)
        odom_msg.twist.twist.linear.y = V * math.sin(self.theta)
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        odom_msg.twist.covariance = [
        0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e-5, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e12, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e12, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1  # Yaw
    ]

        self.odom_publisher.publish(odom_msg)

        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = current_time.to_msg()
        odom_to_base_footprint.header.frame_id = 'odom'
        odom_to_base_footprint.child_frame_id = 'base_footprint'
        odom_to_base_footprint.transform.translation.x = self.x
        odom_to_base_footprint.transform.translation.y = self.y
        odom_to_base_footprint.transform.translation.z = 0.0
        odom_to_base_footprint.transform.rotation.x = 0.0
        odom_to_base_footprint.transform.rotation.y = 0.0
        odom_to_base_footprint.transform.rotation.z = math.sin(self.theta / 2.0)
        odom_to_base_footprint.transform.rotation.w = math.cos(self.theta / 2.0)


def main():
    rclpy.init()
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

