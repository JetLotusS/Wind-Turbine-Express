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
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray

class LocalisationNode(Node):

    def __init__(self):
        super().__init__('localisation_node')

        self.get_logger().info('Localisation node started !')

        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10)
        self.ais_subscriber = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.ais_callback, 10)

        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632
        
        self.aquabot_x = 0.0
        self.aquabot_y = 0.0

        self.wind_turbines_coordinates = []
        self.wind_turbines_coordinates_calcualted = False
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

        b = lat1 - lat2 # angle between the two latitudes in deg
        a = long1 - long2 # angle between the two longitudes in deg

        x = (a/360)*C*(2/3) # distance between the two latitudes in m
        y = (b/360)*(2*np.pi*R) # distance between the two longitudes in m

        return x,y

    def gps_callback(self, msg):
        #self.get_logger().info(f'Latitude:{msg.latitude} Longitude:{msg.longitude}')

        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        aquabot_center_distance = self.distance_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        #self.get_logger().info(f'aquabot_center_distance: {aquabot_center_distance}')
        
        aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        #self.get_logger().info(f'x: {aquabot_coordinate[0]}, y: {aquabot_coordinate[1]}')


    def ais_callback(self, msg):
        """
        Receives wind turbines gps position
        """
        if not self.wind_turbines_coordinates_calcualted:
            poses = msg.poses
            for pose in poses:
                wt_latitude = pose.position.x
                wt_longitude = pose.position.y
                wt_coordinates = self.coordinates_from_point(wt_latitude, wt_longitude, self.origine_latitude, self.origine_longitude)
                self.wind_turbines_coordinates.append(wt_coordinates)

            self.wind_turbines_coordinates_calcualted = True

        self.get_logger().info(f"wind_turbines_coordinates : {self.wind_turbines_coordinates}")
        #self.get_logger().info(f"wind_turbines_distance : {self.wind_turbines_distance}")


def main(args=None):
    rclpy.init(args=args)
    localisation_node = LocalisationNode()
    rclpy.spin(localisation_node)
    localisation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
