#!/usr/bin/env python3
"""
Minimal Nav2 launch for AEMS arena.
- Excludes velocity_smoother + collision_monitor (crash on this install).
- Does NOT remap cmd_vel — controller publishes Twist on /cmd_vel,
  twist_relay (in AEMS) converts to TwistStamped on /cmd_vel_nav for Gazebo.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

MAP    = '/home/dsc/aems_ws/maps/arena_map.yaml'
PARAMS = '/home/dsc/aems_ws/nav2_params.yaml'
SIM    = {'use_sim_time': True}

LIFECYCLE_NODES = [
    'map_server', 'amcl', 'planner_server',
    'controller_server', 'bt_navigator', 'behavior_server',
]

def generate_launch_description():
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[SIM, {'yaml_filename': MAP}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[SIM, PARAMS]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[SIM, PARAMS]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[SIM, PARAMS]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[SIM, PARAMS]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[SIM, PARAMS]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': LIFECYCLE_NODES}]),
        Node(package='adaptive_episodic_memory', executable='twist_relay',
             name='twist_relay', output='screen', parameters=[SIM]),
    ])
