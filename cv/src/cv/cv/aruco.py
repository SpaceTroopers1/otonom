#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


class CamNode(Node):
    def __init__(self):
        super().__init__("cam_node")
        self.cam_sub=self.create_subscription(Image, 'image', self.cam_callback, 10)
        self.cam_pose_pub=self.create_publisher(PoseStamped, 'marker_pose', 10)
        self.bridge=CvBridge()
        self.aruco_dict=aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.aruco_parameters=aruco.DetectorParameters_create()
        #camera calibration parameters
        fx, fy=600.0, 600.0 #genel degerler kameraya gore degistir olcmek lazim
        cx=320.0
        cy=240.0
        k1,k2,p1,p2,k3=0.0,0.0,0.0,0.0,0.0

        self.camera_matrix = np.array([[fx, 0.0, cx],
                                       [0.0, fy, cy],
                                       [0.0,0.0,1.0]])
        self.dist_coeffs = np.array([k1,k2,p1,p2,k3])
        self.marker_size=0.14 #in meter

    def cam_callback(self, msg):
        self.get_logger().info("got img")
        self.image=self.bridge.imgmsg_to_cv2(msg, 'bgr8')
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
                 self.cam_pose_pub.publish(marker_pose)
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