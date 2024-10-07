#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):

    def __init__(self):
        super().__init__('opencv_decoder_node')

        # Create a subscriber on the topic "image_raw"
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)

        # Create a subscriber on the topic "image_raw"
        self.windturbines_report_publisher = self.create_publisher(String, '/vrx/windturbinesinspection/windturbine_checkup', 5)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

    def image_callback(self, msg):

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        #self.get_logger().info(f"Image shape: {current_frame.shape}, dtype: {current_frame.dtype}")

        if current_frame is None:
            self.get_logger().error("The input image is empty")
            return
        
        # Decode image
        data,bbox,rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        if len(data) > 0:
            report_msg = String()
            report_msg.data = data
            self.get_logger().info('Decoded data: ' + data)
            self.windturbines_report_publisher.publish(report_msg)
        else:
            self.get_logger().info('No QR code detected')

def main(args=None):
    rclpy.init(args=args)
    opencv_decoder_node = OpenCvDecoder()
    rclpy.spin(opencv_decoder_node)
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
