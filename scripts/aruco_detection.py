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
from sensor_msgs.msg import CompressedImage, Image
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
        self.image_sub = self.create_subscription(Image,'/camera/rgb/image_raw',self.__image_callback,1)
        self.subscriber = self.create_subscription(ArucoDetection,'/aruco_detections',self.__detection_callback,1)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.__odom_callback,1)
        self.detected_markers: dict = {}

        # Publishers cmd velocity
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel',1)
        self.timer_vel = self.create_timer(0.01, self.__control)
        
        # Publisher img of the marker
        self.final_image_pub = self.create_publisher(Image,'/final_marker_image',1)
        self.save_img = False

        self.state = "detecting"
        self.treshold = 0.1 # radiants
        self.Kp = 0.1
        self.max_angular = 1.0

        self.robot_cmd_vel: Twist = Twist()
        self.robot_cmd_vel.angular.z = 0.3 # Initial angular velocity
        self.robot_odom = None
        
        self.index_id =0
        self.angle_target = 0
        self.error = float('inf')
        self.target_marker = None 
        self.keys = None
        self.img : Image = Image()


    def __image_callback(self,msg: Image):
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
                    
                    self.get_logger().info(f"[DETECT] Detected 5 markers, {list(self.detected_markers.keys())}")
                    self.state = "centering"

                if frame_id not in self.detected_markers:
                    self.get_logger().info(f"[DETECT] New marker detected: {frame_id}")           
            except TransformException:
                continue

    
    def __control(self):
        if self.state != "centering" or not self.keys or len(self.detected_markers) < 5:
            return
        #self.get_logger().info(f"processing {self.keys[self.index_id]}, index: {self.index_id}, error: {self.error}, len: {len(self.detected_markers)}")
        if self.index_id >= len(self.keys) :

            self.get_logger().info("[DONE] All markers centered. Stopping robot.")
            self.detected_markers = {}
            self.index_id = 0
            self.robot_cmd_vel = Twist()
            self.vel_pub.publish(self.robot_cmd_vel)
            self.keys = None
            return 

        current_key = self.keys[self.index_id]

        try:
            # express marker position in the robot frame
            # base_T_marker = self.tf_buffer.lookup_transform('base_footprint',current_key,rclpy.time.Time())
            base_T_marker = self.detected_markers[current_key]
        except Exception as e:
            self.get_logger().info(f"error in control: {e}, trabsf = {self.detected_markers[current_key]}")
            return  

        X = base_T_marker.transform.translation.x
        Y = base_T_marker.transform.translation.y
        angle_target = math.atan2(Y, X)
        # error
        self.error = max(-self.max_angular, min(self.Kp * angle_target, self.max_angular))



        
        if  abs(self.error) < self.treshold :
            self.get_logger().info(f"[CENTERED] Marker {current_key} centered. error = {self.error}")
            if self.img is not None:
                self.__save_circle_img(self.img)
            self.robot_cmd_vel = Twist()
            self.vel_pub.publish(self.robot_cmd_vel)
            self.index_id += 1
            return

            
        # velocity and control
        vel = Twist()
        vel.angular.z = self.error
        self.vel_pub.publish(vel)
        

             
   
    def __save_circle_img(self, img: Image):
       
        cv2.namedWindow('window', cv2.WINDOW_AUTOSIZE)
        self.get_logger().info('ImageFeature node started. Subscribed to /camera/image/compressed')


        try:
            # decode JPEG/PNG from CompressedImage.data
            # np_arr = np.frombuffer(img.data, np.uint8)
            # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            height = img.height
            width = img.width
            channels = 3
            cv_image = np.frombuffer(img.data,dtype=np.uint8 ).reshape(height,width,channels)
            cv_image = cv_image.reshape((height,width,3))
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
            # img_msg = CompressedImage()
            # img_msg.header = img.header
            # img_msg.format = 'jpeg'
            # img_msg.data = jpeg_data.tobytes()
            img_msg = Image()
            img_msg.header = img.header
            img_msg.height,img_msg.width = cv_image.shape[:2]
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width*3
            img_msg.data = cv_image.tobytes()

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
