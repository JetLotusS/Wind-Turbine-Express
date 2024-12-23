#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, UInt32, String
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
        self.chat_subscriber = self.create_subscription(String, '/aquabot/chat', self.chat_callback, 10)

        self.thruster_pub = self.create_publisher(Thruster, '/aquabot/thrusters/thruster_driver', 5, callback_group=self.reentrant_group)
        self.cam_goal_pos_pub = self.create_publisher(Float64, '/aquabot/main_camera_sensor/goal_pose', 5,callback_group=self.reentrant_group)
        self.qr_code_goal_pose_pub = self.create_publisher(Float64, '/aquabot/stabilisation/goal_pose', 5, callback_group=self.reentrant_group)
        self.critical_wind_turbine_pub = self.create_publisher(Thruster, '/aquabot/critical_wind_turbine_coordinates', 5, callback_group=self.reentrant_group)

        # Créé un timer qui appelle la fonction timer_callback toutes les 100 ms
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.navigation_callback, callback_group=self.reentrant_group)

        # Constantes
        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632

        # Variables
        self.current_task = 1

        self.yaw = 0.0 # orientation du bateau

        self.nav_point_x = Float64() # coordonnée x du point à atteindre
        self.nav_point_y = Float64() # coordonnée y du point à atteindre

        self.aquabot_coordinate = [0.0, 0.0] # coordonée cartésiennes du bateau
        self.aquabot_close_to_wind_turbine = False # Vrai si le bateau est à moins de 50m d'une éolienne
        self.aquabot_close_to_critical_wind_turbine = False # Vrai si le bateau est à moins de 50m de l'éolienne critique
        self.closest_winturbine_angle = 0.0 # L'angle entre le bateau et l'éolienne la plus proche
        
        self.wind_turbines_coordinates = [] # Liste des coordonnées cartésiennes de toutes les éoliennes
        self.wind_turbines_coordinates_calcualted = False # Vrai si les coordonnée des éoliennes ont été calculées pour la première fois
        self.wind_turbines_distance = [] # Liste des distances de toutes les éoliennes

        self.critical_wind_turbine_x = 0.0 # coordonnée x de l'éolienne critique
        self.critical_wind_turbine_y = 0.0 # coordonnée y de l'éolienne critique
        self.critical_wind_turbine_coordinates_calculated = False # Vrai si les coordonnée de l'éolienne critique ont été calculées pour la première fois
        self.its_time_to_stabilise = False # Vrai le wte_planner considère que le bateau est assez proche du point de stabilisation

        #Speed PID Controller variables
        self.speed_controller_k_p = 0.15 # Constante de la composante proportionnel du contrôleur de vitesse
        self.speed_controller_k_i = 0.0 # Non utilisée
        self.speed_controller_k_d = 0.0 # Non utilisée
        self.speed_controller_previous_error = 0.0
        self.speed_controller_integral = 0.0

        # Camera Controller variables
        self.camera_theta = Float64() # L'angle de la caméra à atteindre

        # Variables for stabilisation
        self.qr_code_theta = Float64() # L'angle du point de stabilisation par rapport à l'éolienne critique

        self.get_logger().info('Aquabot node started !')


    def current_phase_callback(self, msg):
        """
        Récupère la valeur de la tâche actuelle
        """
        self.current_task = msg.data
        #self.get_logger().info(f'current_task: {self.current_task}')


    def chat_callback(self, msg):
        """
        Reçoit un message du wte_planner pour enclencher la stabilisation
        """
        data = msg.data
        if data:
            if data == "OMG J'AI ATTEINT UNE SUPERBE EOLIENNE ! Elle est dans un état critique, il faut la réparer !" and self.its_time_to_stabilise == False:
                self.its_time_to_stabilise = True
                self.get_logger().info(f"chat : {data}")


    def pinger_callback(self, msg):
        """
        Calculer une fois les coordonnées de l'éolienne critique à partir des données capteur accoustique (pinger) lors de la mise à jour de la tâche à 2
        """
        if self.current_task >= 2 and self.critical_wind_turbine_coordinates_calculated == False:
            msg_cwt_cordinates = Thruster()

            for param in msg.params:
                if param.name == "bearing":
                    pinger_bearing = param.value.double_value
                if param.name == "range":
                    pinger_range = param.value.double_value

            critical_wind_turbine_theta = self.yaw + pinger_bearing
            
            self.critical_wind_turbine_x = pinger_range*np.cos(critical_wind_turbine_theta) + self.aquabot_coordinate[0]
            self.critical_wind_turbine_y = pinger_range*np.sin(critical_wind_turbine_theta) + self.aquabot_coordinate[1]
            self.critical_wind_turbine_coordinates_calculated = True

            msg_cwt_cordinates.x = self.critical_wind_turbine_x
            msg_cwt_cordinates.y = self.critical_wind_turbine_y
            self.critical_wind_turbine_pub.publish(msg_cwt_cordinates)

            wt_distance_to_cwt_list = []

            for wt_coordinates in self.wind_turbines_coordinates:
                wt_distance_to_ctw = self.xy_distance(wt_coordinates[0], wt_coordinates[1], self.critical_wind_turbine_x, self.critical_wind_turbine_y)
                wt_distance_to_cwt_list.append(wt_distance_to_ctw)
            
            closest_wind_turbine_dist = min(wt_distance_to_cwt_list)

            self.critical_wind_turbine_x = self.wind_turbines_coordinates[wt_distance_to_cwt_list.index(closest_wind_turbine_dist)][0]
            self.critical_wind_turbine_y = self.wind_turbines_coordinates[wt_distance_to_cwt_list.index(closest_wind_turbine_dist)][1]

        else:
            return
        

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
        if not self.wind_turbines_coordinates_calcualted:
            poses = msg.poses
            for pose in poses:
                wt_latitude = pose.position.x
                wt_longitude = pose.position.y
                wt_coordinates = self.coordinates_from_point(wt_latitude, wt_longitude, self.origine_latitude, self.origine_longitude)
                self.wind_turbines_coordinates.append(wt_coordinates)

            self.wind_turbines_coordinates_calcualted = True


    def imu_callback(self, msg):
        """
        Receives aquabot yaw
        """        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

        self.angular_vel_z = msg.angular_velocity.z


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
        Compute the speed and direction to give to the thruster driver \n
        Receive a waypoint
        """

        point_x = Float64()
        point_y = Float64()
        thruster_msg = Thruster()
        thruster_max_speed = 6.17 #m/s

        point_x = self.nav_point_x
        point_y = self.nav_point_y

        if not self.wind_turbines_coordinates:
            #self.get_logger().warning("Wind turbine coordinates not available yet!")
            return

        # Create a list of the distances between the aquabot and the wind turbines
        self.wind_turbines_distance = []
        for wt_coordinates in self.wind_turbines_coordinates:
            wt_distance_to_aquabot = self.xy_distance(wt_coordinates[0], wt_coordinates[1], self.aquabot_coordinate[0], self.aquabot_coordinate[1])
            self.wind_turbines_distance.append(wt_distance_to_aquabot)
        
        # Distance between the aquabot and the critical wind turbine
        self.critical_wind_turbines_distance = self.xy_distance(self.critical_wind_turbine_x, self.critical_wind_turbine_y, self.aquabot_coordinate[0], self.aquabot_coordinate[1])

        if point_x == Float64(data=0.0) and point_y == Float64(data=0.0):
            #self.get_logger().warning("No point published yet!")
            return
        

        goal_point_to_aquabot_distance = self.xy_distance(point_x, point_y, self.aquabot_coordinate[0], self.aquabot_coordinate[1]) # m

        thruster_msg.x = point_x
        thruster_msg.y = point_y
        
        # CAMERA CONTROLLER ----------------------------------------------------------------------------------------------------------------------
        # Compute the camera angle -> point the camera to the closest wind turbine
        closest_wind_turbine_dist = min(self.wind_turbines_distance)
        closest_wind_turbine_x_dist = (self.wind_turbines_coordinates[self.wind_turbines_distance.index(closest_wind_turbine_dist)][0] - self.aquabot_coordinate[0])
        closest_wind_turbine_y_co = self.wind_turbines_coordinates[self.wind_turbines_distance.index(closest_wind_turbine_dist)][1]

        # Avoid that closest_wind_turbine_x_dist > closest_wind_turbine_dist resulting in a NaN value from arcos dur to GPS drift
        if abs(closest_wind_turbine_x_dist/closest_wind_turbine_dist) > 1:
            closest_wind_turbine_x_dist = closest_wind_turbine_dist
        
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
        # END CAMERA CONTROLLER -------------------------------------------------------------------------------------------------------------------

        if not self.its_time_to_stabilise:
            # INSPECTION & RALLY
            if goal_point_to_aquabot_distance > 5.0:
                    
                # Speed PID Controller
                speed_controller_error = goal_point_to_aquabot_distance
                self.speed_controller_integral += speed_controller_error*self.timer_period
                speed_controller_derivative = (speed_controller_error - self.speed_controller_previous_error)/self.timer_period
                thruster_msg.speed = self.speed_controller_k_p*speed_controller_error + self.speed_controller_k_i*self.speed_controller_integral + self.speed_controller_k_d*speed_controller_derivative
                self.speed_controller_previous_error = speed_controller_error
                
                # Calculate the angle between the boat yaw and the objective
                thruster_msg.theta = np.arccos((point_x - self.aquabot_coordinate[0])/goal_point_to_aquabot_distance)

                if point_y < self.aquabot_coordinate[1]:
                    thruster_msg.theta = -thruster_msg.theta

                # Uptade aquabot_close_to_wind_turbine to True if the distance between the aquabot and a wind turbine is less than 40m
                self.aquabot_close_to_wind_turbine = False
                for wt_distance in self.wind_turbines_distance:
                    if wt_distance < 50:
                        self.aquabot_close_to_wind_turbine = True
                    
                if self.critical_wind_turbines_distance < 50:
                    self.aquabot_close_to_critical_wind_turbine = True
                else:
                    self.aquabot_close_to_critical_wind_turbine = False

                thruster_msg.speed = thruster_msg.speed*(5000/6.17)
                self.thruster_pub.publish(thruster_msg)
                return

            else:
                thruster_msg.speed = 0.0
                thruster_msg.theta = 0.0

                self.thruster_pub.publish(thruster_msg)

                return
        else:
            # STABILIZE
            # critical wind turbine (cwt) to aquabot distance needs to be 10m (+-1m)
            cwt_to_aquabot_distance = self.xy_distance(self.critical_wind_turbine_x, self.critical_wind_turbine_y, self.aquabot_coordinate[0], self.aquabot_coordinate[1])
            cwt_to_point_facing_the_qrcode_distance = self.xy_distance(self.critical_wind_turbine_x, self.critical_wind_turbine_y, point_x, point_y)
            aquabot_to_point_facing_the_qrcode_distance = self.xy_distance(self.aquabot_coordinate[0], self.aquabot_coordinate[1], point_x, point_y)

            # Aquabot needs to point toward the critical wind turbine
            cwt_orientation = np.arccos((self.critical_wind_turbine_x - self.aquabot_coordinate[0])/cwt_to_aquabot_distance)
            if self.critical_wind_turbine_y < self.aquabot_coordinate[1]:
                    cwt_orientation = -cwt_orientation
            
            thruster_msg.theta = cwt_orientation
            
            # Compute the angle for the aquabot to face the qr code during the stabilisation phase
            cwt_qrcode_orientation = np.arccos((self.critical_wind_turbine_x - point_x)/cwt_to_point_facing_the_qrcode_distance)
            if self.critical_wind_turbine_y < point_y:
                    cwt_qrcode_orientation = -cwt_qrcode_orientation

            self.qr_code_theta.data = cwt_qrcode_orientation
  
            # Speed P Controller
            speed_controller_error = (cwt_to_aquabot_distance - 10)

            thruster_msg.speed = self.speed_controller_k_p*speed_controller_error
            self.speed_controller_previous_error = speed_controller_error

            thruster_msg.speed = thruster_msg.speed
            thruster_msg.speed = thruster_msg.speed*(5000/6.17)

            #self.get_logger().info(f"speed: {thruster_msg.speed}")
            self.thruster_pub.publish(thruster_msg)
            self.qr_code_goal_pose_pub.publish(self.qr_code_theta)

        
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
