#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class DriftNode(Node):

    def __init__(self):
        super().__init__('drift_node')

        self.left_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 5)
        self.right_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 5)
        self.left_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 5)
        self.right_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 5)
        
        # Create a timer that will call the timer_callback function every 500ms
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Declare variables
        self.i = 0

    def timer_callback(self):

        self.i += 1
        
        left_turn = 0.0
        right_turn = -0.78539816339 

        left_speed = 4000.0
        right_speed = 1000.0

        if self.i%2 == 0.0:
            left_speed = 0.0

        #self.get_logger().info('Publishing: "%s"' % left_speed_msg.data)
        #self.get_logger().info('Publishing: "%s"' % right_speed_msg.data)rtf
        #self.get_logger().info('Publishing: "%s"' % left_turn_msg.data)
        #self.get_logger().info('Publishing: "%s"' % right_turn_msg.data)

        left_speed_msg = Float64()
        left_turn_msg = Float64()
        left_speed_msg.data = left_speed
        left_turn_msg.data = left_turn

        right_speed_msg = Float64()
        right_turn_msg = Float64()
        right_speed_msg.data = right_speed
        right_turn_msg.data = right_turn
            
        self.left_speed_pub.publish(left_speed_msg)
        self.right_speed_pub.publish(right_speed_msg)
        self.left_turn_pub.publish(left_turn_msg)
        self.right_turn_pub.publish(right_turn_msg)


def main(args=None):
    rclpy.init(args=args)
    drift_node = DriftNode()
    rclpy.spin(drift_node)
    drift_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
