#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class WTEAquabotNode(Node):

    def __init__(self):
        super().__init__('wte_aquabot_node')

        self.reentrant_group = ReentrantCallbackGroup()

        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10, callback_group=self.reentrant_group)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10, callback_group=self.reentrant_group)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10, callback_group=self.reentrant_group)

        self.left_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 5)
        self.right_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 5)
        self.left_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 5)
        self.right_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 5)
        
        # Create a timer that will call the timer_callback function every 500ms
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.thruster_callback, callback_group=self.reentrant_group)

        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632
        
        self.get_logger().info('Aquabot node started !')


    def euler_from_quaternion(self, quaternion):

        # Converts quaternion (w in last place) to Euler roll, pitch, yaw
        # Quaternion = [x, y, z, w]

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


    def distance_from_point(self, lat1, long1, lat2, long2):
        
        theta1 = lat1 * np.pi/180
        theta2 = lat2 * np.pi/180
        phi1 = long1 * np.pi/180
        phi2 = long2 * np.pi/180

        R = 6366354 # earth radius at latitude 48.04630

        # using haversine formula for a central angle and solving for distance
        a = np.sin((theta2 - theta1)/2)**2 + np.cos(theta1)*np.cos(theta2)*(np.sin((phi2 - phi1)/2)**2)
        c = 2*np.arctan2(np.sqrt(a), np.sqrt(1-a))
        distance = R*c # in m
        
        return distance


    def coordinates_from_point(self, lat1, long1, lat2, long2):
        
        R = 6366354 # earth radius at latitude 48.04630
        C = 40075017 # earth meridional circumference

        a = lat1 - lat2 # angle between the two latitudes in deg
        b = long1 - long2 # angle between the two longitudes in deg

        x = (a/360)*C # distance between the two latitudes in m
        y = (b/360)*(4*np.pi*R)/3 # distance between the two longitudes in m

        return x,y


    def gps_callback(self, msg):

        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        aquabot_center_distance = self.distance_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        

    def ais_callback(self, msg):

        self.eolienne_1_latitude = msg.poses[0].position.x
        self.eolienne_1_longitude = msg.poses[0].position.y

        self.eolienne_2_latitude = msg.poses[1].position.x
        self.eolienne_2_longitude = msg.poses[1].position.y

        self.eolienne_3_latitude = msg.poses[2].position.x
        self.eolienne_3_longitude = msg.poses[2].position.y
        
        eolienne_1_coordinate = self.coordinates_from_point(self.eolienne_1_latitude, self.eolienne_1_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_2_coordinate = self.coordinates_from_point(self.eolienne_2_latitude, self.eolienne_2_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_3_coordinate = self.coordinates_from_point(self.eolienne_3_latitude, self.eolienne_3_longitude, self.origine_latitude, self.origine_longitude)
        

    def imu_callback(self, msg):
        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

        self.angular_vel_z = msg.angular_velocity.z
        self.linear_vel_x = msg.linear_acceleration.x
        self.linear_vel_y = msg.linear_acceleration.y

        #self.get_logger().info(f"yaw: {self.yaw}")


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
    wte_aquabot_node = WTEAquabotNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(wte_aquabot_node)

    try:
        executor.spin()
    finally:
        wte_aquabot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
