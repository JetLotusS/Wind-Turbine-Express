#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

class LocalisationNode(Node):

    def __init__(self):
        super().__init__('localisation_node')

        self.get_logger().info('Localisation node started !')

        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10)

        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632
        
        self.aquabot_x = 0.0
        self.aquabot_y = 0.0

    def gps_callback(self, msg):
        #self.get_logger().info(f'Latitude:{msg.latitude} Longitude:{msg.longitude}')

        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        self.get_logger().info(f'x:{self.aquabot_x} y:{self.aquabot_y}')

def main(args=None):
    rclpy.init(args=args)
    localisation_node = LocalisationNode()
    rclpy.spin(localisation_node)
    localisation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
