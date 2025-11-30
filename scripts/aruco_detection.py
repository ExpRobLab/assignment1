#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import imutils
import math

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformListener, TransformException, LookupException, ConnectivityException, ExtrapolationException

VERBOSE = False


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        
        # Listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.subscriber = self.create_subscription(ArucoDetection,'/aruco_detections',self.__detection_callback,1)
        self.detected_markers: dict = {}

        # Publishers cmd velocity
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',1)
        self.timer = self.create_timer(0.01, self.__control)

        # TODO Publisher img of the marker
        

        self.state = "detecting"
        self.treshold = 4.0  # radiants
        self.Kp = 1.0
        self.max_angular = 1.0

        self.robot_cmd_vel: Twist = Twist()
        self.robot_cmd_vel.angular.z = 0.5  # Initial angular velocity



    def __detection_callback(self, msg):
        if self.state != "detecting":
            return
        
        self.vel_pub.publish(self.robot_cmd_vel)

        for marker in msg.markers:
            frame_id = f"marker_{marker.marker_id}"

            try:
                odom_T_arucomarker = self.tf_buffer.lookup_transform(
                    'odom',
                    frame_id,
                    rclpy.time.Time()
                )
                self.detected_markers[frame_id] = odom_T_arucomarker
                
                if len(self.detected_markers) == 5:
                    self.detected_markers = dict(sorted(self.detected_markers.items()))
                    self.state = "centering"

                if frame_id not in self.detected_markers:
                    self.get_logger().info(f"Detected new marker: {frame_id}")

            except TransformException:
                continue
    
    
    def __control(self):
        if self.state != "centering":
            return
        
        marker_id, odom_T_arucomarker = next(iter(self.detected_markers.items()))

        odom_T_camera = self.tf_buffer.lookup_transform(
                    'odom',
                    'camera_link_optical',
                    rclpy.time.Time()
                )
        
        x = odom_T_arucomarker.transform.translation.x - odom_T_camera.transform.translation.x
        y = odom_T_arucomarker.transform.translation.y - odom_T_camera.transform.translation.y
        angle_to_marker = math.atan2(y, x)

        self.robot_cmd_vel.angular.z = max(-self.max_angular, min(self.max_angular, self.Kp * angle_to_marker))
        self.vel_pub.publish(self.robot_cmd_vel)

        # Remove the element from the dictionary
        if abs(angle_to_marker) < self.treshold:
            self.robot_cmd_vel.angular.z = 0.0
            self.vel_pub.publish(self.robot_cmd_vel)

            self.get_logger().info(f"Pointed marker: {marker_id}\n")
            # TODO self.get_logger().info("Pres <butto> to continue...\n")
            self.detected_markers.pop(marker_id)
        
        if not self.detected_markers:
            self.robot_cmd_vel.angular.z = 0.5
            self.vel_pub.publish(self.robot_cmd_vel)
            self.state = "detecting"

def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArucoDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ImageFeature node")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
