#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):

    def __init__(self):
        super().__init__('wte_cv_node')

        # Create a subscriber on the topic "image_raw"
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 1)
        
        # Create a publisher on the topic "windturbine_checkup"
        self.windturbines_report_publisher = self.create_publisher(String, '/vrx/windturbinesinspection/windturbine_checkup', 5)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

    def image_callback(self, msg):

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)
        
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        current_frame = cv2.equalizeHist(current_frame)  # Enhance contrast

        #self.get_logger().info(f"Image shape: {current_frame.shape}, dtype: {current_frame.dtype}")

        if current_frame is None or current_frame.size == 0:
            self.get_logger().error("The input image is empty or invalid")
            return

        try:
            data, bbox, rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)
        except cv2.error as e:
            self.get_logger().error(f"OpenCV error: {e}")
            return

        if bbox is not None and len(bbox) > 0 and cv2.contourArea(bbox) > 0:
            
            if data:
                report_msg = String()
                report_msg.data = data
                #self.get_logger().info('Decoded data: ' + data)
                self.windturbines_report_publisher.publish(report_msg)
            else:
                #self.get_logger().info('No QR code detected in the image')
                return

        else:
            #self.get_logger().warning("No bounding box detected for QR code")
            return

def main(args=None):
    rclpy.init(args=args)
    opencv_decoder_node = OpenCvDecoder()
    rclpy.spin(opencv_decoder_node)
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
