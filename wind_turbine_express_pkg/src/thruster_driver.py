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

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64, UInt32, String
from wind_turbine_express_interfaces.msg import Thruster
from rcl_interfaces.msg import SetParametersResult

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ThrusterNode(Node):

    def __init__(self):
        super().__init__('thruster_node')

        self.reentrant_group = ReentrantCallbackGroup()

        self.gps_subscriber = self.create_subscription(NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10)
        self.thruster_subscriber = self.create_subscription(Thruster, '/aquabot/thrusters/thruster_driver', self.thruster_callback, 10)
        self.cam_goal_pos_subscriber = self.create_subscription(Float64, '/aquabot/main_camera_sensor/goal_pose', self.cam_goal_pose_callback, 10)
        self.current_phase_subscriber = self.create_subscription(UInt32, '/vrx/windturbinesinspection/current_phase', self.current_phase_callback, 10)
        self.chat_subscriber = self.create_subscription(String, '/aquabot/chat', self.chat_callback, 10)

        # Create publishers for thruster control
        self.left_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 5, callback_group=self.reentrant_group)
        self.right_speed_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 5, callback_group=self.reentrant_group)
        self.left_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 5, callback_group=self.reentrant_group)
        self.right_turn_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 5, callback_group=self.reentrant_group)

        # Create a publisher for camera control
        self.cam_thruster_pub = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 5, callback_group=self.reentrant_group)
        
        # For the camera TF listener
        self.cam_tf_buffer = Buffer()
        self.cam_tf_listener = TransformListener(self.cam_tf_buffer, self)

        # Create a timer that will call the driver_callback function every 100ms
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.driver_callback, callback_group=self.reentrant_group)
        
        # Create a timer that will call the cam_tf_callback function every 100ms
        self.timer2 = self.create_timer(self.timer_period, self.cam_tf_callback, callback_group=self.reentrant_group)

        # Add a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Daclare Aquabot variables
        self.aquabot_coordinate = []
        self.current_task = 1
        self.its_time_to_stabilise = False

        # Declare thruster variables
        self.thruster_speed = 0.0
        self.yaw = Float64()

        self.left_turn = 0.0
        self.right_turn = 0.0

        self.left_speed = 0.0
        self.right_speed = 0.0

        self.x_goal_pose = Float64()
        self.y_goal_pose = Float64()
        self.prev_x_goal_pose = Float64()
        self.prev_y_goal_pose = Float64()
        self.yaw_goal_pose = Float64()
        self.thruster_goal_speed = Float64()

        # GPS coordinates of the center
        self.origine_latitude = 48.04630
        self.origine_longitude = -4.97632

        # Direction PID Controller variables

        self.declare_parameter('kp', 0.1)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.002)

        self.direction_controller_k_p = self.get_parameter('kp').get_parameter_value().double_value
        self.direction_controller_k_i = self.get_parameter('ki').get_parameter_value().double_value
        self.direction_controller_k_d = self.get_parameter('kd').get_parameter_value().double_value

        self.direction_controller_previous_error = 0.0
        self.direction_controller_integral = 0.0
        self.thruster_turn_angle = 0.0
        
        self.stabilisation_speed_controller_k_p = 100.0

        # Declare camera variable

        self.camera_angle = 0.0
        self.camera_controller_k_p = 0.2

        self.get_logger().info('Thruster driver node started !')

    def parameter_callback(self, params):
        successful = True
        for param in params:
            if param.name == 'kp':
                self.direction_controller_k_p = param.value
            elif param.name == 'ki':
                self.direction_controller_k_i = param.value
            elif param.name == 'kd':
                self.direction_controller_k_d = param.value
            else:
                successful = False  # Unknown parameter

        return SetParametersResult(successful=successful)


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
        
        R = 6366354 # earth radius at latitude 48.04630
        C = 40075017 # earth meridional circumference

        long_angle = long1 - long2 # angle between the two longitudes in deg
        lat_angle = lat1 - lat2 # angle between the two latitudes in deg

        x = (long_angle/360)*C*(2/3) # distance between the two latitudes in m
        y = (lat_angle/360)*(2*np.pi*R) # distance between the two longitudes in m

        return x,y


    def xy_distance(self, x1, y1, x2, y2):
        """
        Compute the distance between two x,y coordinates
        """
        distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return distance


    def current_phase_callback(self, msg):
        """
        Get the current task number
        """
        self.current_task = msg.data


    def chat_callback(self, msg):
        data = msg.data
        if data:
            if data == "OMG J'AI ATTEINT UNE SUPERBE EOLIENNE ! Elle est dans un état critique, il faut la réparer !" and self.its_time_to_stabilise == False:
                self.its_time_to_stabilise = True
                self.get_logger().info(f"STABILIZE")


    def gps_callback(self, msg):
        """
        Receives aquabot's latitude and longitude coordinates and converts them into cartesian coordinates
        """
        self.aquabot_coordinate = self.coordinates_from_point(msg.latitude, msg.longitude, self.origine_latitude, self.origine_longitude)


    def imu_callback(self, msg):
        """
        Receives aquabot yaw
        """
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.yaw = self.euler_from_quaternion(quaternion)

        self.angular_vel_z = msg.angular_velocity.z
        self.linear_accel_x = msg.linear_acceleration.x
        self.linear_accel_y = msg.linear_acceleration.y


    def thruster_callback(self, msg):
        """
        Receives objective x, y, theta and speed
        """
        self.x_goal_pose = msg.x
        self.y_goal_pose = msg.y
        self.yaw_goal_pose = msg.theta
        self.thruster_goal_speed = msg.speed
        #self.get_logger().info(f"x_goal_pose: {self.x_goal_pose}, y_goal_pose: {self.y_goal_pose}")
        #self.get_logger().info(f"yaw_goal_pose: {self.yaw_goal_pose}, thruster_goal_speed: {self.thruster_goal_speed}")


    def cam_goal_pose_callback(self, msg):
        """
        Receives camera goal position
        """
        self.cam_goal_pose = msg.data


    def cam_tf_callback(self):
        """
        Listen to the TF between the camera and the base_link to get the actual angle of the camera
        """
        from_frame_rel = 'wamv/wamv/base_link'
        to_frame_rel = 'wamv/wamv/main_camera_post_link'
    
        trans = None
        
        try:
            now = Time()
            trans = self.cam_tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        quaternion = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]

        self.camera_tf_angle = -self.euler_from_quaternion(quaternion) # + and - inverted between the TF and the sim
        #self.get_logger().info(f"camera_angle: {self.camera_tf_angle}")
        

    def driver_callback(self):
        """
        Publish to the aquabot thruster and camera thruster
        """

        if self.x_goal_pose == Float64(data=0.0) and self.y_goal_pose == Float64(data=0.0):
            #self.get_logger().warning("No goal received yet!")
            return

        # Reset the integral part of the direction PID controller when the goal point change
        #if self.prev_x_goal_pose != self.x_goal_pose and self.prev_y_goal_pose != self.y_goal_pose:
        #    self.direction_controller_integral = 0.0

        #self.prev_x_goal_pose == self.x_goal_pose
        #self.prev_y_goal_pose == self.y_goal_pose

        if np.isnan(self.yaw_goal_pose):
            #self.get_logger().warning("yaw_goal_pose nan")
            return
        
        #self.get_logger().info(f"x_goal_pose: {self.x_goal_pose}, y_goal_pose: {self.y_goal_pose}")

        turn_limit = 0.78539816339
        speed_limit = 5000.0

        # Direction PID controller -------------------------------------------------------------------------------------------------------------------------
        direction_controller_error = - (self.yaw_goal_pose - self.yaw)

        if np.abs(direction_controller_error) > np.pi:
            if direction_controller_error > 0:
                direction_controller_error -= 2*np.pi
            else:
                direction_controller_error += 2*np.pi

        if not self.its_time_to_stabilise:
            self.direction_controller_integral += direction_controller_error*self.timer_period
            direction_controller_derivative = (direction_controller_error - self.direction_controller_previous_error)/self.timer_period
            self.thruster_turn_angle = self.direction_controller_k_p*direction_controller_error + self.direction_controller_k_i*self.direction_controller_integral + self.direction_controller_k_d*direction_controller_derivative
            self.direction_controller_previous_error = direction_controller_error
        else:
            self.thruster_turn_angle = np.pi/4
            turning_speed_for_stabilisation = self.stabilisation_speed_controller_k_p*direction_controller_error
            self.get_logger().info(f"self.thruster_turn_angle {self.thruster_turn_angle}")
        
        # END Direction PID controller ---------------------------------------------------------------------------------------------------------------------

        #self.get_logger().info(f"thruster_turn_angle: {self.thruster_turn_angle}")
        #self.get_logger().info(f"direction_controller_error: {self.direction_controller_k_p*direction_controller_error}")
        #self.get_logger().info(f"direction_controller_integral: {self.direction_controller_k_i*self.direction_controller_integral}")
        #self.get_logger().info(f"direction_controller_derivative: {self.direction_controller_k_d*direction_controller_derivative}")

        #self.thruster_speed = self.thruster_speed/((1+np.abs(direction_controller_error))**8)
        #self.get_logger().info(f"self.thruster_speed: {self.thruster_speed})"
        
        # Speed adjustment
        #self.thruster_speed = min(speed_limit, np.abs(self.thruster_goal_speed))
        self.thruster_speed = self.thruster_goal_speed

        #self.get_logger().info(f"self.thruster_speed: {self.thruster_speed}")
                               
        # Camera P controller -------------------------------------------------------------------------------------------------------------------------
        cam_msg = Float64()
        camera_angle_error = self.cam_goal_pose - self.camera_tf_angle
        self.camera_angle += camera_angle_error*self.camera_controller_k_p
        cam_msg.data = float(self.camera_angle)
        # END Camera P controller -------------------------------------------------------------------------------------------------------------------------

        #self.get_logger().info(f"thruster_turn_angle: {self.thruster_turn_angle}")

        if not self.its_time_to_stabilise:
            self.left_speed = self.thruster_speed
            self.right_speed = self.thruster_speed

            self.left_turn = self.thruster_turn_angle
            self.right_turn = self.thruster_turn_angle
        else:
            self.left_speed = self.thruster_speed + turning_speed_for_stabilisation
            self.right_speed = self.thruster_speed - turning_speed_for_stabilisation

            self.left_turn = self.thruster_turn_angle
            self.right_turn = -self.thruster_turn_angle

        self.get_logger().info(f"left_turn {self.left_turn}, right_turn {self.right_turn}")

        left_speed_msg = Float64()
        left_turn_msg = Float64()
        left_speed_msg.data = self.left_speed
        left_turn_msg.data = self.left_turn

        right_speed_msg = Float64()
        right_turn_msg = Float64()
        right_speed_msg.data = self.right_speed
        right_turn_msg.data = self.right_turn

        self.left_speed_pub.publish(left_speed_msg)
        self.right_speed_pub.publish(right_speed_msg)
        self.left_turn_pub.publish(left_turn_msg)
        self.right_turn_pub.publish(right_turn_msg)

        self.cam_thruster_pub.publish(cam_msg)

def main(args=None):
    rclpy.init(args=args)
    thruster_node = ThrusterNode()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(thruster_node)

    try:
        executor.spin()
    finally:
        thruster_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()