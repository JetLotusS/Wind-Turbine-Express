#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from wind_turbine_express_interfaces.msg import Thruster

class ThrusterNode(Node):

    def __init__(self):
        super().__init__('thruster_node')
        
        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10)
        self.thruster_subscriber = self.create_subscription(Thruster, '/aquabot/thrusters/thruster_driver', self.thruster_callback, 10)

        self.left_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 5)
        self.right_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 5)
        self.left_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 5)
        self.right_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 5)
        
        # Create a timer that will call the timer_callback function every 500ms
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.driver_callback)

        # Declare variables
        self.thruster_speed = 0.0

        self.left_turn = 0.0
        self.right_turn = 0.0

        self.left_speed = 0.0
        self.right_speed = 0.0

        self.x_goal_pose = 0.0
        self.y_goal_pose = 0.0
        self.yaw_goal_pose = 0.0
        self.thruster_goal_speed = 0.0


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
        self.linear_accel_x = msg.linear_acceleration.x
        self.linear_accel_y = msg.linear_acceleration.y

        #self.get_logger().info(f"yaw: {self.yaw}")

    def thruster_callback(self, msg):
        
        self.x_goal_pose = msg.x
        self.y_goal_pose = msg.y
        self.yaw_goal_pose = msg.theta
        self.thruster_goal_speed = msg.speed
        self.get_logger().info(f"x_goal_pose: {self.x_goal_pose}, y_goal_pose: {self.y_goal_pose}")
        self.get_logger().info(f"yaw_goal_pose: {self.yaw_goal_pose}, thruster_goal_speed: {self.thruster_goal_speed}")


    def driver_callback(self):
        
        '''
        ------------------------------- /!\ En developpement /!\ -------------------------------
        '''

        if self.thruster_speed < self.thruster_goal_speed:
            self.thruster_speed += self.thruster_goal_speed*50
        else:
            self.thruster_speed = 0.0

        if self.yaw > self.yaw_goal_pose:
            self.left_turn -= 0.2 
            self.right_turn -= 0.2
        else:
            self.left_turn += 0.2 
            self.right_turn += 0.2

        self.left_speed = self.thruster_speed
        self.right_speed = self.thruster_speed

        left_speed_msg = Float64()
        left_turn_msg = Float64()
        left_speed_msg.data = self.left_speed
        left_turn_msg.data = self.left_turn

        right_speed_msg = Float64()
        right_turn_msg = Float64()
        right_speed_msg.data = self.right_speed
        right_turn_msg.data = self.right_turn
        
        #self.get_logger().info('Publishing: "%s"' % left_speed_msg.data)
        #self.get_logger().info('Publishing: "%s"' % right_speed_msg.data)
        #self.get_logger().info('Publishing: "%s"' % left_turn_msg.data)
        #self.get_logger().info('Publishing: "%s"' % right_turn_msg.data)

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
