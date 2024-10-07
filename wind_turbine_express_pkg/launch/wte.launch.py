from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    default_world_name = 'aquabot_windturbines_easy'
    #aquabot_regatta
    #aquabot_windturbines_easy
    #aquabot_windturbines_medium
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

    ld.add_action(world_arg)
    ld.add_action(aquabot_competition_launch_file)
    #ld.add_action(navigation_node)
    #ld.add_action(thruster_driver_node)

    return ld