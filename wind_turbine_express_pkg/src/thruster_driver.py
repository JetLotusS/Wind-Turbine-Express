#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class ThrusterNode(Node):

    def __init__(self):
        super().__init__('thruster_node')
        
        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10)

        self.left_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 5)
        self.right_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 5)
        self.left_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 5)
        self.right_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 5)
        
        # Create a timer that will call the timer_callback function every 500ms
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.thruster_callback)

        # Declare variables

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def imu_callback(self, msg):
        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

        self.angular_vel_z = msg.angular_velocity.z
        self.linear_vel_x = msg.linear_acceleration.x
        self.linear_vel_y = msg.linear_acceleration.y

        self.get_logger().info(f"yaw: {self.yaw}")


    def thruster_callback(self):
        
        left_turn = 0.0
        right_turn = 0.0

        left_speed = 0.0
        right_speed = 0.0

        #self.get_logger().info('Publishing: "%s"' % left_speed_msg.data)
        #self.get_logger().info('Publishing: "%s"' % right_speed_msg.data)
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
    thruster_node = ThrusterNode()
    rclpy.spin(thruster_node)
    thruster_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
