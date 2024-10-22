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
from wind_turbine_express_interfaces.msg import Thruster

class WTEAquabotNode(Node):

    def __init__(self):
        super().__init__('wte_aquabot_node')

        self.reentrant_group = ReentrantCallbackGroup()

        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10, callback_group=self.reentrant_group)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10, callback_group=self.reentrant_group)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10, callback_group=self.reentrant_group)
        
        self.thruster_pub = self.create_publisher(Thruster, '/aquabot/thrusters/thruster_driver', 5)

        # Create a timer that will call the timer_callback function every 500ms
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.navigation_callback, callback_group=self.reentrant_group)

        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632

        self.obstacles_coordinates = [(-44, 222), (-94, 176), (98, 146), (-154, -3), (118, -48), (-45, -95), (10, -97), (-35, -150)]
        self.wt_coordinates_index = 0
        self.wind_turbines_coordinates = []
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

        long_angle = long1 - long2 # angle between the two longitudes in deg
        lat_angle = lat1 - lat2 # angle between the two latitudes in deg

        x = (long_angle/360)*C*(2/3) # distance between the two latitudes in m
        y = (lat_angle/360)*(2*np.pi*R) # distance between the two longitudes in m

        return x,y

    def xy_distance(self, x1, y1, x2, y2):

        distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return distance

    def gps_callback(self, msg):

        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        aquabot_center_distance = self.distance_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        self.aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        

    def ais_callback(self, msg):

        self.eolienne_1_latitude = msg.poses[0].position.x
        self.eolienne_1_longitude = msg.poses[0].position.y

        self.eolienne_2_latitude = msg.poses[1].position.x
        self.eolienne_2_longitude = msg.poses[1].position.y

        self.eolienne_3_latitude = msg.poses[2].position.x
        self.eolienne_3_longitude = msg.poses[2].position.y
        
        eolienne_A_coordinate = self.coordinates_from_point(self.eolienne_1_latitude, self.eolienne_1_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_B_coordinate = self.coordinates_from_point(self.eolienne_2_latitude, self.eolienne_2_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_C_coordinate = self.coordinates_from_point(self.eolienne_3_latitude, self.eolienne_3_longitude, self.origine_latitude, self.origine_longitude)
        
        self.wind_turbines_coordinates = [eolienne_A_coordinate, eolienne_B_coordinate, eolienne_C_coordinate]
        self.get_logger().info(f"{self.wind_turbines_coordinates}")

    def imu_callback(self, msg):
        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

        self.angular_vel_z = msg.angular_velocity.z
        self.linear_vel_x = msg.linear_acceleration.x
        self.linear_vel_y = msg.linear_acceleration.y

        self.get_logger().info(f"yaw: {self.yaw}")

    def navigation_callback(self):

        '''
        ------------------------------- /!\ En developpement /!\ -------------------------------
        '''
                
        thruster_msg = Thruster()
        
        if not self.wind_turbines_coordinates:
            self.get_logger().warning("Wind turbine coordinates not available yet!")
            return
        
        wind_turbine_to_aquabot_distance = self.xy_distance(self.wind_turbines_coordinates[0][0], self.wind_turbines_coordinates[0][1], self.aquabot_coordinate[0], self.aquabot_coordinate[1])

        if wind_turbine_to_aquabot_distance > 10.0:
            
            thruster_msg.x = self.wind_turbines_coordinates[self.wt_coordinates_index][0]
            thruster_msg.y = self.wind_turbines_coordinates[self.wt_coordinates_index][1]
            thruster_msg.speed = 3000.0

            thruster_msg.theta = np.arccos((self.wind_turbines_coordinates[self.wt_coordinates_index][0] - self.aquabot_coordinate[0])/wind_turbine_to_aquabot_distance)

            self.get_logger().info(f"a_x: {self.aquabot_coordinate[0]}, a_y: {self.aquabot_coordinate[0]}")
            self.get_logger().info(f"wt_x: {self.wind_turbines_coordinates[self.wt_coordinates_index][0]}, wt_y: {self.wind_turbines_coordinates[self.wt_coordinates_index][1]}")
            self.get_logger().info(f"wt_a_xd: {self.wind_turbines_coordinates[self.wt_coordinates_index][0] - self.aquabot_coordinate[0]}, wt_a_d: {wind_turbine_to_aquabot_distance}")
            
            if self.wind_turbines_coordinates[self.wt_coordinates_index][1] < self.aquabot_coordinate[1]:
                thruster_msg.theta = -thruster_msg.theta

            self.thruster_pub.publish(thruster_msg)

        else:
            self.get_logger().info(f"Point has been reached !")
            return
  
        
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
