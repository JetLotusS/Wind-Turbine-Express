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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    navigation_node = Node(
        package="wind_turbine_express_pkg",
        executable="wte_aquabot.py",
        output="screen",
    )

    thruster_driver_node = Node(
        package="wind_turbine_express_pkg",
        executable="thruster_driver.py",
        output="screen",
    )

    cv_node = Node(
        package="wind_turbine_express_pkg",
        executable="wte_cv_module.py",
        output="screen",
    )

    planner_node = Node(
        package="wind_turbine_express_pkg",
        executable="wte_planner.py",
        output="screen",
    )
    
    return LaunchDescription([

        navigation_node,
        thruster_driver_node,
        cv_node,
        planner_node,

    ])