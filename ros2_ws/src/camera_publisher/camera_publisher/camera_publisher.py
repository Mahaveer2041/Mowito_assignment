#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        #settimerfo30HI1/30 ≈ 0.033 seconds)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
       
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            raise RuntimeError("Cannot open camera")
            
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera publisher started at {actual_fps} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.publisher_.publish(msg)           
                self.get_logger().info('Publishing to /image_raw', throttle_duration_sec=1)
            except Exception as e:
                self.get_logger().error(f"Error converting image: {str(e)}")
        else:
            self.get_logger().error("Failed to capture frame")

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
