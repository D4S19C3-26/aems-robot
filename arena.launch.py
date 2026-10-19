import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='7.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='-1.5708')

    world = '/home/dsc/aems_ws/worlds/arena.sdf'

    # MUST be first: local models dir overrides system so 12m-LiDAR waffle loads
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        '/home/dsc/aems_ws/models:'
        + os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
        + ':/home/dsc/aems_ws/worlds'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # spawn_turtlebot3 also starts the ROS-GZ parameter bridge.
    # The bridge maps /cmd_vel (TwistStamped) -> Gazebo cmd_vel (Twist).
    # Nav2 is configured with enable_stamped_cmd_vel:true (in nav2_params.yaml)
    # so it outputs TwistStamped directly -- no extra relay node needed.
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/dsc/aems_ws', 'custom_spawn.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose, 'yaw_pose': yaw_pose}.items()
    )

    ld = LaunchDescription()
    ld.add_action(set_resource_path)   # FIRST: env var before any process
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    return ld
