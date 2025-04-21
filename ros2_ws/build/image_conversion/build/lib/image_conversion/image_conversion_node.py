#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')
        
        # Parameters
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/converted_image')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Mode (default: color - no conversion)
        self.mode = 2  # 1: grayscale, 2: color
        
        # CV Bridge
        self.bridge = CvBridge()
        
        #subscriber
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10)
        
        #publisher
        self.publisher = self.create_publisher(
            Image,
            output_topic,
            10)
        
        #service
        self.service = self.create_service(
            SetBool,
            'set_conversion_mode',
            self.set_mode_callback)
        
        self.get_logger().info(f"Image Conversion Node initialized")
        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")
        self.get_logger().info(f"Current mode: {'Grayscale' if self.mode == 1 else 'Color'}")

    def image_callback(self, msg):
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            
            if self.mode == 1:  #grayscalemode
                output_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                encoding = 'mono8'
            else:  #colormode
                output_image = cv_image
                encoding = 'bgr8'
            
            
            output_msg = self.bridge.cv2_to_imgmsg(output_image, encoding=encoding)
            output_msg.header = msg.header  # Keep the same header
            
            
            self.publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def set_mode_callback(self, request, response):

        if request.data:
            self.mode = 1  # Grayscale
            self.get_logger().info("Mode changed to Grayscale")
        else:
            self.mode = 2  # Color
            self.get_logger().info("Mode changed to Color")
        
        response.success = True
        response.message = f"Mode set to {'Grayscale' if self.mode == 1 else 'Color'}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
