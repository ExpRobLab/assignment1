#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import imutils
import tf_transformations as tf

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

VERBOSE = False


class ImageFeature(Node):
    def __init__(self):
        super().__init__('image_feature')

        # Publishers
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        # Subscriber
        self.subscriber = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.callback,
            1
        )

        #Aruco Subscriber
        self.subscriber = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.detection_callback,
            1
        )

        self.detected_markers = []
        self.marker_msgs = []
        self.state = 'discover'
        self.current_tarket_index = 0

        self.timer = self.create_timer(0.1, self.control_loop)

         # Buffer TF + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)



    def detection_callback(self, msg):
        self.marker_msgs = msg.markers
        if self.state == 'discover':

            for marker in msg.markers:
                if marker.marker_id not in self.detected_markers and len(self.detected_markers) < 5:

                    self.get_logger().info(f"Detected Marker: {marker.marker_id}")
                    transform = self.tf_buffer.lookup_transform(
                        'odom',
                        'camera_link_optical',
                        rclpy.time.Time()
                    )
                    T = tf.quaternion_matrix()
                    self.detected_markers.append(marker.marker_id)
    
    def control_loop(self):
        twist = Twist()

        if self.state == 'discover':
            if len(self.detected_markers) < 5:
                twist.angular.z = 0.5
            else:
                self.get_logger().info("All Markers Detected!")
                self.detected_markers.sort()
                self.state = 'show_ordered'
                self.current_tarket_index = 0

        elif self.state == 'show_ordered':
            if not self.marker_msgs:
                twist.angular.z = 0.5
            else: 
                tarket_id = self.detected_markers[self.current_tarket_index]
                ids_visible = [m.marker_id for m in self.marker_msgs]
                if tarket_id in ids_visible:
                    twist.angular.z = 0.0
                    self.get_logger().info(f"Marker Visualized: {tarket_id}")

                    self.current_tarket_index += 1
                    if self.current_tarket_index >= len(self.detected_markers):
                        self.get_logger().info("Mission Complited")
                        self.timer.cancel()
                        twist.angular.z = 0.0
                    
                else:
                    twist.angular.z = 0.5
        self.vel_pub.publish(twist)




def main(args=None):
    rclpy.init(args=args)
    node = ImageFeature()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ImageFeature node")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
