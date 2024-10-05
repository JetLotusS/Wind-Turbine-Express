#!/usr/bin/env python3

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
        #self.get_logger().info(f'Latitude:{msg.latitude} Longitude:{msg.longitude}')

        self.aquabot_x = msg.latitude - self.origine_latitude
        self.aquabot_y = msg.longitude - self.origine_longitude

        aquabot_center_distance = self.distance_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        #self.get_logger().info(f'aquabot_center_distance: {aquabot_center_distance}')
        
        aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)
        self.get_logger().info(f'x: {aquabot_coordinate[0]}, y: {aquabot_coordinate[1]}')

    def ais_callback(self, msg):
        
        self.eolienne_1_latitude = msg.poses[0].position.x
        self.eolienne_1_longitude = msg.poses[0].position.y

        self.eolienne_2_latitude = msg.poses[1].position.x
        self.eolienne_2_longitude = msg.poses[1].position.y

        self.eolienne_3_latitude = msg.poses[2].position.x
        self.eolienne_3_longitude = msg.poses[2].position.y
        
        eolienne_1_coordinate = self.coordinates_from_point(self.eolienne_1_latitude, self.eolienne_1_longitude, self.origine_latitude, self.origine_longitude)
        self.get_logger().info(f'x_e1: {eolienne_1_coordinate[0]}, y_e1: {eolienne_1_coordinate[1]}')

        eolienne_2_coordinate = self.coordinates_from_point(self.eolienne_2_latitude, self.eolienne_2_longitude, self.origine_latitude, self.origine_longitude)
        self.get_logger().info(f'x_e2: {eolienne_2_coordinate[0]}, y_e2: {eolienne_2_coordinate[1]}')

        eolienne_3_coordinate = self.coordinates_from_point(self.eolienne_3_latitude, self.eolienne_3_longitude, self.origine_latitude, self.origine_longitude)
        self.get_logger().info(f'x_e3: {eolienne_3_coordinate[0]}, y_e3: {eolienne_3_coordinate[1]}')


def main(args=None):
    rclpy.init(args=args)
    localisation_node = LocalisationNode()
    rclpy.spin(localisation_node)
    localisation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
