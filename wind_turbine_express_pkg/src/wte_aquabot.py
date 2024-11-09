#!/usr/bin/env python3

# Copyright 2024 Wind Turbine Express.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, UInt32
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from wind_turbine_express_interfaces.msg import Thruster
from ros_gz_interfaces.msg import ParamVec

class WTEAquabotNode(Node):

    def __init__(self):
        super().__init__('wte_aquabot_node')

        self.reentrant_group = ReentrantCallbackGroup()

        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10, callback_group=self.reentrant_group)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10, callback_group=self.reentrant_group)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10, callback_group=self.reentrant_group)
        self.pinger_subscriber = self.create_subscription(ParamVec, '/aquabot/sensors/acoustics/receiver/range_bearing', self.pinger_callback, 10, callback_group=self.reentrant_group)
        self.thruster_subscriber = self.create_subscription(Thruster, '/aquabot/navigation/point', self.get_point_callback, 10, callback_group=self.reentrant_group)
        self.current_phase_subscriber = self.create_subscription(UInt32, '/vrx/windturbinesinspection/current_phase', self.current_phase_callback, 10, callback_group=self.reentrant_group)
        
        self.thruster_pub = self.create_publisher(Thruster, '/aquabot/thrusters/thruster_driver', 5)
        self.cam_goal_pos_pub = self.create_publisher(Float64, '/aquabot/main_camera_sensor/goal_pose', 5)

        # Create a timer that will call the timer_callback function every 100ms
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.navigation_callback, callback_group=self.reentrant_group)

        # Variables
        self.current_task = 1

        self.nav_point_x = Float64()
        self.nav_point_y = Float64()

        self.aquabot_coordinate = [0.0, 0.0]
        self.aquabot_close_to_wind_turbine = False
        self.closest_winturbine_angle = 0.0

        self.critical_wind_turbine_x = 0.0
        self.critical_wind_turbine_y = 0.0
        self.critical_wind_turbine_coordinates_calculated = False

        # Constants
        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632

        #Center point coordinate of each obstacle
        self.obstacles_coordinates = [(-44, 222), (-94, 176), (98, 146), (-154, -3), (118, -48), (-45, -95), (10, -97), (-35, -150)]
        
        self.wt_coordinates_index = 0
        self.wind_turbines_coordinates = []
        self.wind_turbines_distance = []

        #Speed PID Controller variables
        self.speed_controller_k_p = 0.05
        self.speed_controller_k_i = 0.001
        self.speed_controller_k_d = 0.0
        self.speed_controller_previous_error = 0.0
        self.speed_controller_integral = 0.0

        # Camera Controller variables
        self.camera_theta = Float64()

        self.get_logger().info('Aquabot node started !')


    def current_phase_callback(self, msg):
        """
        Get the current task number
        """
        self.current_task = msg.data
        self.get_logger().info(f'current_task: {self.current_task}')


    def pinger_callback(self, msg):
        """
        Calcul once the coordinates of the critical wind turbine from the pinger data when task phase update to 2
        """
        if self.current_task == 2 and self.critical_wind_turbine_coordinates_calculated == False:
            for param in msg.params:
                if param.name == "bearing":
                    pinger_bearing = param.value.double_value
                if param.name == "range":
                    pinger_range = param.value.double_value
            critical_wind_turbine_theta = self.yaw + pinger_bearing
            self.critical_wind_turbine_x = pinger_range*np.cos(critical_wind_turbine_theta)
            self.critical_wind_turbine_y = pinger_range*np.sin(critical_wind_turbine_theta)
            if critical_wind_turbine_theta < 0:
                self.critical_wind_turbine_y = -self.critical_wind_turbine_y
            if np.abs(critical_wind_turbine_theta) > np.pi:
                self.critical_wind_turbine_x = -self.critical_wind_turbine_x
            self.critical_wind_turbine_coordinates_calculated = True
        else:
            return

        self.get_logger().info(f'pinger : bearing: {pinger_bearing}, range: {pinger_range}')
        self.get_logger().info(f'critical_wind_turbine_x: {self.critical_wind_turbine_x}, critical_wind_turbine_y: {self.critical_wind_turbine_y}')


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to Euler roll, pitch, yaw \n
        Quaternion = [x, y, z, w]
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


    def distance_from_point(self, lat1, long1, lat2, long2):
        """
        Returns the Cartesian distance between two set of GPS coordinates
        """ 
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
        """
        Returns the Cartesian coordinates from point 1 to point 2
        """         
        R = 6366354 # Earth radius at latitude 48.04630
        C = 40075017 # Earth meridional circumference

        long_angle = long1 - long2 # Angle between the two longitudes in deg
        lat_angle = lat1 - lat2 # Angle between the two latitudes in deg

        x = (long_angle/360)*C*(2/3) # Distance between the two latitudes in m
        y = (lat_angle/360)*(2*np.pi*R) # Distance between the two longitudes in m

        return x,y


    def xy_distance(self, x1, y1, x2, y2):
        """
        Compute the distance between two x,y coordinates
        """        
        distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return distance


    def gps_callback(self, msg):
        """
        Receives aquabot's latitude and longitude coordinates and converts them into cartesian coordinates
        """       
        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        aquabot_center_distance = self.distance_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        self.aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        

    def ais_callback(self, msg):
        """
        Receives wind turbines gps position
        """
        self.eolienne_A_latitude = msg.poses[0].position.x
        self.eolienne_A_longitude = msg.poses[0].position.y

        self.eolienne_B_latitude = msg.poses[1].position.x
        self.eolienne_B_longitude = msg.poses[1].position.y

        self.eolienne_C_latitude = msg.poses[2].position.x
        self.eolienne_C_longitude = msg.poses[2].position.y
        
        eolienne_A_coordinate = self.coordinates_from_point(self.eolienne_A_latitude, self.eolienne_A_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_B_coordinate = self.coordinates_from_point(self.eolienne_B_latitude, self.eolienne_B_longitude, self.origine_latitude, self.origine_longitude)
        eolienne_C_coordinate = self.coordinates_from_point(self.eolienne_C_latitude, self.eolienne_C_longitude, self.origine_latitude, self.origine_longitude)
        
        self.wind_turbines_coordinates = [eolienne_A_coordinate, eolienne_B_coordinate, eolienne_C_coordinate]

        self.get_logger().info(f"wind_turbines_coordinates : {self.wind_turbines_coordinates}")
        self.get_logger().info(f"wind_turbines_distance : {self.wind_turbines_distance}")


    def imu_callback(self, msg):
        """
        Receives aquabot yaw
        """        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

        self.angular_vel_z = msg.angular_velocity.z
        self.linear_vel_x = msg.linear_acceleration.x
        self.linear_vel_y = msg.linear_acceleration.y

        #self.get_logger().info(f"yaw: {self.yaw}")


    def get_point_callback(self, msg):
        """
        Receives the waypoint for navigation
        """
        self.nav_point_x = Float64()
        self.nav_point_y = Float64()

        self.nav_point_x = msg.x
        self.nav_point_y = msg.y


    def navigation_callback(self):

        """
        ------------------------------- /!\ En developpement /!\ -------------------------------
        """
        point_x = Float64()
        point_y = Float64()
        thruster_msg = Thruster()
        thruster_max_speed = 6.17 #m/s

        point_x = self.nav_point_x
        point_y = self.nav_point_y

        if not self.wind_turbines_coordinates:
            self.get_logger().warning("Wind turbine coordinates not available yet!")
            return
    
        eolienne_A_distance = self.xy_distance(self.wind_turbines_coordinates[0][0], self.wind_turbines_coordinates[0][1], self.aquabot_coordinate[0], self.aquabot_coordinate[1])
        eolienne_B_distance = self.xy_distance(self.wind_turbines_coordinates[1][0], self.wind_turbines_coordinates[1][1], self.aquabot_coordinate[0], self.aquabot_coordinate[1])
        eolienne_C_distance = self.xy_distance(self.wind_turbines_coordinates[2][0], self.wind_turbines_coordinates[2][1], self.aquabot_coordinate[0], self.aquabot_coordinate[1])

        self.wind_turbines_distance = [eolienne_A_distance, eolienne_B_distance, eolienne_C_distance]

        if point_x == Float64(data=0.0) and point_y == Float64(data=0.0):
            self.get_logger().warning("No point published yet!")
            return
        
        #self.get_logger().info(f"p_x: {point_x}, p_y: {point_y}")

        goal_point_to_aquabot_distance = self.xy_distance(point_x, point_y, self.aquabot_coordinate[0], self.aquabot_coordinate[1]) # m

        thruster_msg.x = point_x
        thruster_msg.y = point_y
        
        # Compute the camera angle -> point the camera to the closest wind turbine
        closest_wind_turbine_dist = min(self.wind_turbines_distance[0], self.wind_turbines_distance[1], self.wind_turbines_distance[2])
        closest_wind_turbine_x_dist = (self.wind_turbines_coordinates[self.wind_turbines_distance.index(closest_wind_turbine_dist)][0] - self.aquabot_coordinate[0])
        closest_wind_turbine_y_co = self.wind_turbines_coordinates[self.wind_turbines_distance.index(closest_wind_turbine_dist)][1]

        # Avoid that closest_wind_turbine_x_dist > closest_wind_turbine_dist resulting in a NaN value from arcos dur to GPS drift
        if abs(closest_wind_turbine_x_dist/closest_wind_turbine_dist) > 1:
            closest_wind_turbine_x_dist = closest_wind_turbine_dist

        #if self.wind_turbines_coordinates[self.wind_turbines_distance.index(closest_wind_turbine_dist)][0] - self.aquabot_coordinate[0] < 0:
        #    closest_wind_turbine_x_dist = - closest_wind_turbine_x_dist
        
        self.closest_winturbine_angle = np.arccos(closest_wind_turbine_x_dist/closest_wind_turbine_dist)

        if closest_wind_turbine_y_co < self.aquabot_coordinate[1]:
            self.closest_winturbine_angle = -self.closest_winturbine_angle

        if not np.isnan(self.closest_winturbine_angle):

            # Compute the camera goal angle error
            camera_goal_error = self.closest_winturbine_angle - self.yaw

            if np.abs(camera_goal_error) > np.pi:
                if camera_goal_error > 0:
                    camera_goal_error -= 2*np.pi
                else:
                    camera_goal_error += 2*np.pi

            self.camera_theta.data = camera_goal_error
            self.cam_goal_pos_pub.publish(self.camera_theta)
        else:
            self.get_logger().warning("Invalid closest_winturbine_angle detected, skipping publish")
            #self.get_logger().warning(f"closest_wind_turbine_dist : {closest_wind_turbine_dist}")
            #self.get_logger().warning(f"self.aquabot_coordinate : {self.aquabot_coordinate}")

        if goal_point_to_aquabot_distance > 10.0:
                   
            # Speed PID Controller
            speed_controller_error = goal_point_to_aquabot_distance
            self.speed_controller_integral += speed_controller_error*self.timer_period
            speed_controller_derivative = (speed_controller_error - self.speed_controller_previous_error)/self.timer_period
            thruster_msg.speed = self.speed_controller_k_p*speed_controller_error + self.speed_controller_k_i*self.speed_controller_integral + self.speed_controller_k_d*speed_controller_derivative
            self.speed_controller_previous_error = speed_controller_error

            #self.get_logger().info(f"a_x: {self.aquabot_coordinate[0]}, a_y: {self.aquabot_coordinate[0]}")
            #self.get_logger().info(f"p_x: {point_x}, p_y: {point_y}")
            #self.get_logger().info(f"p_a_xd: {point_x - self.aquabot_coordinate[0]}, p_a_yd: {goal_point_to_aquabot_distance}")
            
            # Calculate the angle between the boat yaw and the objective
            thruster_msg.theta = np.arccos((point_x - self.aquabot_coordinate[0])/goal_point_to_aquabot_distance)

            if point_y < self.aquabot_coordinate[1]:
                thruster_msg.theta = -thruster_msg.theta

            #self.get_logger().info(f"thruster_msg.theta: {thruster_msg.theta}")

            # Uptade aquabot_close_to_wind_turbine to True if the distance between the aquabot and a wind turbine is less than 40m
            if self.wind_turbines_distance[0] < 40 or self.wind_turbines_distance[1] < 40 or self.wind_turbines_distance[2] < 40:
                self.aquabot_close_to_wind_turbine = True
            else:
                self.aquabot_close_to_wind_turbine = False

            # Reduce speed if aquabot close to wind turbine
            if self.aquabot_close_to_wind_turbine:
                thruster_msg.speed = thruster_msg.speed*0.5

            thruster_msg.speed = thruster_msg.speed*(5000/6.17)
            self.thruster_pub.publish(thruster_msg)

        else:
            thruster_msg.speed = 0.0
            thruster_msg.theta = 0.0

            self.thruster_pub.publish(thruster_msg)

            self.get_logger().info(f"Point ({point_x}, {point_y}) has been reached !")
            return
    
        
def main(args=None):
    rclpy.init(args=args)
    wte_aquabot_node = WTEAquabotNode()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(wte_aquabot_node)

    try:
        executor.spin()
    finally:
        wte_aquabot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
