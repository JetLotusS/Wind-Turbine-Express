from launch_ros.actions import Node

import aquabot_gz.bridges
import vrx_gz.bridges

def get_windturbines_competition_worlds():
    return [
        f'aquabot_windturbines_competition_{i:02d}' for i in range(0, 10)
    ]

PATROLANDFOLLOW_WORLDS = [
  'aquabot_patrolandfollow_easy',
  'aquabot_patrolandfollow_medium',
  'aquabot_patrolandfollow_hard'
]
WINDTURBINES_INSPECTION_WORLDS = [
    'aquabot_windturbines_easy',
    'aquabot_windturbines_medium',
    'aquabot_windturbines_hard'
]
WINDTURBINES_INSPECTION_WORLDS.extend(get_windturbines_competition_worlds())

def competition_bridges(world_name, competition_mode=False):
    bridges = [
        vrx_gz.bridges.clock(),
        vrx_gz.bridges.task_info()
    ]

    if not competition_mode:
        bridges.extend([
            vrx_gz.bridges.usv_wind_speed(),
            vrx_gz.bridges.usv_wind_direction()
        ])

    task_bridges = []
    if world_name in PATROLANDFOLLOW_WORLDS:
        task_bridges = [
            aquabot_gz.bridges.patrolandfollow_current_phase(),
            aquabot_gz.bridges.patrolandfollow_ais_ennemy_position(),
            aquabot_gz.bridges.patrolandfollow_ais_ennemy_speed(),
            aquabot_gz.bridges.patrolandfollow_ais_allies_positions(),
            aquabot_gz.bridges.patrolandfollow_ais_allies_speeds(),
            aquabot_gz.bridges.patrolandfollow_alert_position(),
        ]
        if not competition_mode:
            task_bridges.extend([
                aquabot_gz.bridges.patrolandfollow_debug_buoy_pose_error(),
                aquabot_gz.bridges.patrolandfollow_debug_alert_pose_error(),
                aquabot_gz.bridges.patrolandfollow_debug_follow_pose_error(),
                aquabot_gz.bridges.patrolandfollow_debug_alert_mean_error(),
                aquabot_gz.bridges.patrolandfollow_debug_follow_mean_error(),
                aquabot_gz.bridges.patrolandfollow_debug_ennemy_distance(),
                aquabot_gz.bridges.patrolandfollow_debug_allies_distance(),
            ])
    elif world_name in WINDTURBINES_INSPECTION_WORLDS:
        task_bridges = [
            aquabot_gz.bridges.windturbinesinspection_current_phase(),
            aquabot_gz.bridges.windturbinesinspection_ais_windturbines_positions(),
            aquabot_gz.bridges.windturbinesinspection_windturbine_checkup(),
        ]
        if not competition_mode:
            task_bridges.extend([
                aquabot_gz.bridges.windturbinesinspection_debug_critical_windturbine_heading(),
                aquabot_gz.bridges.windturbinesinspection_debug_critical_windturbine_distance(),
                aquabot_gz.bridges.windturbinesinspection_debug_critical_windturbine_bearing_from_boat(),
                aquabot_gz.bridges.windturbinesinspection_debug_critical_target_pose_distance(),
                aquabot_gz.bridges.windturbinesinspection_debug_critical_turn_around_angle(),
                aquabot_gz.bridges.windturbinesinspection_debug_windturbines_distances(),
                aquabot_gz.bridges.windturbinesinspection_debug_windturbine_checkup_status_ok(),
                aquabot_gz.bridges.windturbinesinspection_debug_windturbine_checkup_status_not_received(),
            ])
            
    bridges.extend(task_bridges)

    nodes = []
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))
    return nodes
