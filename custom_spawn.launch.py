import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    
    # USE OUR CUSTOM MODEL
    urdf_path = os.path.join('/home/dsc/aems_ws/models', model_folder, 'model.sdf')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='-1.5708')

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw_pose
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder+'_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='0.0', description='X spawn pose')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='0.0', description='Y spawn pose')
    declare_yaw_pose_cmd = DeclareLaunchArgument('yaw_pose', default_value='-1.5708', description='Yaw spawn pose')

    ld = LaunchDescription()
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_yaw_pose_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    if TURTLEBOT3_MODEL != 'burger':
        ld.add_action(start_gazebo_ros_image_bridge_cmd)

    return ld
