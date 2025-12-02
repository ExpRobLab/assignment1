#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import numpy as np
import cv2
import imutils
import math

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformListener, TransformException, LookupException, ConnectivityException, ExtrapolationException
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
VERBOSE = True


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        
        current_path = os.getcwd()

        # Current script directory
        current_path = os.path.dirname(os.path.abspath(__file__))

        # Go up to the workspace root
        self.workspace_path = os.path.abspath(os.path.join(current_path, "../../../../images"))

        # Listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.image_sub = self.create_subscription(CompressedImage,'/camera/image/compressed',self.__image_callback,1)
        self.subscriber = self.create_subscription(ArucoDetection,'/aruco_detections',self.__detection_callback,1)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.__odom_callback,1)
        self.detected_markers: dict = {}

        # Publishers cmd velocity
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',1)
        self.timer_vel = self.create_timer(0.01, self.__control)
        
        # Publisher img of the marker
        self.final_image_pub = self.create_publisher( CompressedImage,'/final_marker_image',1)
        self.save_img = False

        self.state = "detecting"
        self.treshold = 0.1 # radiants
        self.Kp = 1.0
        self.max_angular = 1.0

        self.robot_cmd_vel: Twist = Twist()
        self.robot_cmd_vel.angular.z = 0.5  # Initial angular velocity
        self.robot_odom = None
        
        self.index_id =0
        self.angle_target = 0
        self.error = float('inf')
        self.target_marker = None 
        self.keys = None
        self.img : CompressedImage = CompressedImage()


    def __image_callback(self,msg: CompressedImage):
        self.img =  msg

    def __odom_callback(self, msg):
        self.robot_odom = msg

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
                    first_key = next(iter(self.detected_markers))
                    self.target_marker = self.detected_markers[first_key]
                    self.keys = list(self.detected_markers.keys())
                    self.__set_target_angle()
                    
                    self.get_logger().info(f"[DETECT] Detected 5 markers, {list(self.detected_markers.keys())}"f" starting centering on: {first_key}")
                    self.state = "centering"

                if frame_id not in self.detected_markers:
                    self.get_logger().info(f"[DETECT] New marker detected: {frame_id}")           
            except TransformException:
                continue

    
    def __control(self):
        if self.state != "centering" or len(self.detected_markers) < 5:
            return
        
        current_key = None
        try:
            current_key = self.keys[self.index_id]
            # self.get_logger().info(f"Marker: {current_key}, Index: {self.index_id}")
        except Exception as e:
            #self.get_logger().info(e)
            current_key = None
            # self.get_logger().info(f" Marker: {current_key}, Index: {self.index_id}")
        
        
        # retrrive the next target angle form the transformation between odom and the marker frames
        q = self.robot_odom.pose.pose.orientation
        (quat_x, quat_y, quat_z, quat_w) = (q.x, q.y, q.z, q.w)
        _, _, yaw = euler_from_quaternion((quat_x, quat_y, quat_z, quat_w))

        # error
        self.error = self.angle_target - yaw

        if self.error < self.treshold and self.index_id >= len(self.detected_markers) :

            self.get_logger().info("[DONE] All markers centered. Stopping robot.")
            self.detected_markers = {}
            self.index_id = 0
            self.robot_cmd_vel = Twist()
            self.vel_pub.publish(self.robot_cmd_vel)
            self.keys = None
            return 

        if  abs(self.error) < self.treshold :
            self.get_logger().info(f"[CENTERED] Marker {current_key} centered. eror = {self.error}")
            # increment the indiex to take the next marker
              
            self.__save_circle_img(self.img)
            self.index_id += 1 
            self.get_logger().info(f"")
            if self.index_id <len(self.detected_markers):
                
                
                key = self.keys[self.index_id] 
                self.target_marker = self.detected_markers[key]
                
                # obtain the quaterianion and the reference angle between, marker and aruco 
                self.__set_target_angle()

                self.robot_cmd_vel = Twist()
                self.vel_pub.publish(self.robot_cmd_vel)

           
            return 

        # velocity and control
        vel = Twist()
        vel.angular.z = max(-self.max_angular, min(self.Kp * self.error, self.max_angular))
        self.vel_pub.publish(vel)
        

    def __set_target_angle(self):
        dx = self.target_marker.transform.translation.x - self.robot_odom.pose.pose.position.x
        dy = self.target_marker.transform.translation.y - self.robot_odom.pose.pose.position.y
        self.angle_target = math.atan2(dy, dx)            
   
    def __save_circle_img(self, img: CompressedImage):
        try:
            # decode JPEG/PNG from CompressedImage.data
            np_arr = np.frombuffer(img.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn("Could not decode compressed image, nothing to save.")
                return

            # draw circle around marker
            h, w = cv_image.shape[:2]
            center = (w // 2, h // 2)
            radius = min(h, w) // 8
            cv2.circle(cv_image, center, radius, (0, 255, 0), 3)

            # save to disk
            
            key = self.keys[self.index_id ]
            image_path = os.path.join(self.workspace_path, f"{key}.png")
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f"PNG saved {image_path}, and published")


            # re-encode to JPEG before publishing as CompressedImage
            _, jpeg_data = cv2.imencode('.jpg', cv_image)
            img_msg = CompressedImage()
            img_msg.header = img.header
            img_msg.format = 'jpeg'
            img_msg.data = jpeg_data.tobytes()

            self.final_image_pub.publish(img_msg)

        

        except Exception as e:
            self.get_logger().warn(f"Error converting/saving/publishing image: {e}")



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
