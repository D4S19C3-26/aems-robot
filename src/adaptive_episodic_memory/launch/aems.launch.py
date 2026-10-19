import os
from launch import LaunchDescription
from launch_ros.actions import Node

SIM_TIME = {'use_sim_time': True}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adaptive_episodic_memory',
            executable='landmark_manager',
            name='landmark_manager',
            output='screen',
            parameters=[SIM_TIME]
        ),
        Node(
            package='adaptive_episodic_memory',
            executable='yolo_perception',
            name='yolo_perception',
            output='screen',
            parameters=[SIM_TIME]
        ),
        Node(
            package='adaptive_episodic_memory',
            executable='perception_bridge',
            name='perception_bridge',
            output='screen',
            parameters=[SIM_TIME]
        ),
        Node(
            package='adaptive_episodic_memory',
            executable='twist_relay',
            name='twist_relay',
            output='screen',
            parameters=[SIM_TIME]
        ),
        Node(
            package='adaptive_episodic_memory',
            executable='route_replayer',
            name='route_replayer',
            output='screen',
            parameters=[SIM_TIME]
        ),
        # Dashboard NL command interface node
        Node(
            package='adaptive_episodic_memory',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            parameters=[SIM_TIME]
        ),
        # AEMS Mission Control Web & WebSocket Bridge (HTTP:8080, WS:9090)
        Node(
            package='adaptive_episodic_memory',
            executable='web_bridge',
            name='aems_web_bridge',
            output='screen',
            parameters=[SIM_TIME]
        ),
    ])

