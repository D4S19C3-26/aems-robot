#!/usr/bin/env python3
# Copyright 2024 AEMS Project
# Launch: custom aems_house world with TurtleBot3 waffle
# Targeting: ROS 2 Jazzy + Gazebo Harmonic

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Paths ────────────────────────────────────────────────────────────
    tb3_gazebo_dir  = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_dir  = get_package_share_directory('ros_gz_sim')
    launch_file_dir = os.path.join(tb3_gazebo_dir, 'launch')

    # World file lives in our workspace (absolute path)
    world = '/home/dsc/aems_ws/worlds/aems_house.sdf'

    # ── Launch arguments ─────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot spawns in Room 1 (NW), well clear of all walls.
    # Room 1 interior centre is (-1.25, 1.25). We nudge slightly
    # towards the south-east so the robot faces room 2's doorway.
    x_pose = LaunchConfiguration('x_pose', default='-1.5')
    y_pose = LaunchConfiguration('y_pose', default='1.0')

    # ── Gazebo server (headless physics) ─────────────────────────────────
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Gazebo GUI ────────────────────────────────────────────────────────
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Robot State Publisher (publishes TF from URDF) ────────────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ── Spawn TurtleBot3 + ROS bridges ───────────────────────────────────
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # ── Make sure Gazebo can find the TurtleBot3 model files ─────────────
    set_gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_dir, 'models')
    )

    ld = LaunchDescription()
    ld.add_action(set_gz_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    return ld
