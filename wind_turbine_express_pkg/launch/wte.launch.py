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
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    default_world_name = 'aquabot_windturbines_medium'
    #aquabot_regatta
    #aquabot_windturbines_easy
    #aquabot_windturbines_medium
    #aquabot_windturbines_hard
    world_arg = DeclareLaunchArgument(
            'world',
            default_value = default_world_name,
            description = 'World name')
    ld.add_action(world_arg)

    aquabot_competition_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aquabot_gz'), 
                         'launch/competition.launch.py')),
            launch_arguments={}.items()
    )

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

    ld.add_action(world_arg)
    ld.add_action(aquabot_competition_launch_file)
    ld.add_action(navigation_node)
    ld.add_action(thruster_driver_node)
    ld.add_action(cv_node)

    return ld