#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import cv2
import cv2.aruco as aruco
import numpy as np
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import tf_transformations as tf



class CamNode(Node):
    def __init__(self):
        super().__init__("cam_node")

        # ROS2 setup
        self.bridge = CvBridge()
        #self.cam_sub = self.create_subscription(Image, 'image', self.cam_callback, 10)
        self.cam_pose_pub = self.create_publisher(PoseStamped, 'marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ArUco setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.aruco_parameters = aruco.DetectorParameters_create()
        self.marker_size = 0.15  # meters

        # Initialize RealSense
        self.init_realsense()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def init_realsense(self):
        # Configure the RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()

        self.camera_matrix = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array(intr.coeffs[:5])  # [k1, k2, p1, p2, k3]

        self.get_logger().info("RealSense calibration loaded.")


    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            self.get_logger().warn('No color frame received.')
            return

        # Convert to numpy array
        self.image = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        corners, ids, _=aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                    # Draw rectangle around detected marker
                    aruco.drawDetectedMarkers(self.image, corners, ids)

                    # Get the bottom-left corner of the marker for text placement
                    corner = corners[i][0]
                    text_pos = (int(corner[0][0]), int(corner[0][1]) - 10)

                    # Write the marker ID below the marker
                    cv2.putText(self.image, f"ID: {marker_id[0]}", text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            for i, corner in enumerate(corners):
                 rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, self.marker_size, self.camera_matrix, self.dist_coeffs)
                 #tvec [x,y,z] to camera in meters calculated with marker and marker size
                 marker_pose=PoseStamped()
                 marker_pose.header.frame_id="camera_link" #change when real cam
                 marker_pose.header.stamp = self.get_clock().now().to_msg()
                 marker_pose.pose.position.x = tvec[0][0][0]
                 marker_pose.pose.position.y = tvec[0][0][1]
                 marker_pose.pose.position.z = tvec[0][0][2]
                 #rvec rotation vector convert to matrix by cv2.Rodrigues then to quaternion(what we need in pose)
                 rotation_matrix, _ = cv2.Rodrigues(rvec[0])  # Convert to rotation matrix
                 quaternion = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
                 marker_pose.pose.orientation.x = quaternion[0]
                 marker_pose.pose.orientation.y = quaternion[1]
                 marker_pose.pose.orientation.z = quaternion[2]
                 marker_pose.pose.orientation.w = quaternion[3]
                 
                 self.camera_translation = np.array([marker_pose.pose.position.x, marker_pose.pose.position.y, marker_pose.pose.position.z])  # Example tvec
                 self.camera_quaternion = np.array([marker_pose.pose.orientation.x , marker_pose.pose.orientation.y, marker_pose.pose.orientation.z,marker_pose.pose.orientation.w])
                 
                 try:
                     transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                 except tf2_ros.LookupException:
                     self.get_logger().error("Transform from 'camera' to 'map' not found")
                     return

        # Extract translation and rotation from the transform
                 map_translation = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]
                 map_rotation = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]       
                 
                 camera_matrix = tf.translation_matrix(self.camera_translation) @ tf.quaternion_matrix(self.camera_quaternion)

        # Convert map->camera transform to 4x4 matrix
                 map_matrix = tf.translation_matrix(map_translation) @ tf.quaternion_matrix(map_rotation)

        # Transform camera pose to map frame
                 map_pose_matrix = np.dot(map_matrix, camera_matrix)

        # Extract position and orientation
                 map_position = tf.translation_from_matrix(map_pose_matrix)
                 map_quaternion = tf.quaternion_from_matrix(map_pose_matrix)

        # Create goal PoseStamped message
                 goal_pose = PoseStamped()
                 goal_pose.header.frame_id = 'map'
                 
                 goal_pose.header.stamp = self.get_clock().now().to_msg()
                 goal_pose.pose.position.z = map_position[0]
                 goal_pose.pose.position.y = map_position[1]
                 goal_pose.pose.position.x = map_position[2]
                 goal_pose.pose.orientation.x = map_quaternion[0]
                 goal_pose.pose.orientation.y = map_quaternion[1]
                 goal_pose.pose.orientation.z = map_quaternion[2]
                 goal_pose.pose.orientation.w = map_quaternion[3]
                 print(map_position)
                 
                 
                 aruco_link = TransformStamped()
                 aruco_link.header.frame_id = 'map'
                 aruco_link.child_frame_id = 'aruco'
                 aruco_link.header.stamp = self.get_clock().now().to_msg()
                 aruco_link.transform.translation.x = map_position[0]
                 aruco_link.transform.translation.y = map_position[1]
                 aruco_link.transform.translation.z = map_position[2]
                 aruco_link.transform.rotation.x = map_quaternion[0]
                 aruco_link.transform.rotation.y = map_quaternion[1]
                 aruco_link.transform.rotation.z = map_quaternion[2]
                 aruco_link.transform.rotation.w = map_quaternion[3]
                 
                 self.tf_broadcaster.sendTransform(aruco_link)
                                   
                 
                 #self.cam_pose_pub.publish(marker_pose)
                 self.image=aruco.drawAxis(self.image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_size)
                 cv2.putText(self.image, f"Pos {tvec[0]}", (10,10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.imshow("Camera", self.image)
        
        
        
        cv2.waitKey(1)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

