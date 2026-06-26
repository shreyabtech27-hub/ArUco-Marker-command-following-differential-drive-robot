import os
import shlex
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    my_robot_bringup_share = get_package_share_directory('my_robot_bringup')
    my_robot_description_share = get_package_share_directory('my_robot_description')

    # Environment variables
    models_path = os.path.join(my_robot_bringup_share, 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # File paths
    urdf_path = os.path.join(my_robot_description_share, 'urdf', 'my_robot.urdf.xacro')
    gazebo_config_path = os.path.join(my_robot_bringup_share, 'config', 'gazebo_bridge.yaml')
    rviz_config_path = os.path.join(my_robot_description_share, 'rviz', 'urdf_config.rviz')
    nav2_params_path = os.path.join(my_robot_bringup_share, 'config', 'nav2_params.yaml')
    slam_params_path = os.path.join(my_robot_bringup_share, 'config', 'slam_toolbox.yaml')
    world_path = os.path.join(my_robot_bringup_share, 'worlds', 'my_world.sdf')

    # Declare Launch Arguments
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    declare_use_slam = DeclareLaunchArgument('use_slam', default_value='true')
    declare_use_nav2 = DeclareLaunchArgument('use_nav2', default_value='true')
    declare_use_aruco_commands = DeclareLaunchArgument('use_aruco_commands', default_value='true')
    declare_headless = DeclareLaunchArgument('headless', default_value='true')

    # Launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_aruco_commands = LaunchConfiguration('use_aruco_commands')
    headless = LaunchConfiguration('headless')

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    # Gazebo Simulation Launch Description
    gz_sim_launch_py = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    # Quote the world path to handle potential spaces/parentheses in the directory path
    quoted_world_path = shlex.quote(world_path)

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_py),
        condition=IfCondition(headless),
        launch_arguments={
            'gz_args': f'-r -s {quoted_world_path}'
        }.items()
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_py),
        condition=UnlessCondition(headless),
        launch_arguments={
            'gz_args': f'-r {quoted_world_path}'
        }.items()
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-x', '0.0', '-y', '-5.0', '-z', '0.15', '-Y', '1.570796'],
        parameters=[{'use_sim_time': False}]
    )

    # ROS-Gazebo Parameter Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gazebo_config_path,
            'use_sim_time': False
        }]
    )

    # Image Bridge
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image'],
        parameters=[{
            'camera.image.compressed.jpeg_quality': 75,
            'use_sim_time': False
        }]
    )

    # Topic relay camera info
    relay_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        parameters=[{
            'input_topic': 'camera/camera_info',
            'output_topic': 'camera/image/camera_info',
            'use_sim_time': False
        }]
    )

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        )),
        condition=IfCondition(use_slam),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_path
        }.items()
    )

    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        )),
        condition=IfCondition(use_nav2),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    # ArUco Command Follower Node
    aruco_command_follower = Node(
        package='my_test_pkg',
        executable='aruco_command_follower',
        name='aruco_follower',  # Explicit unique name to avoid duplicates
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_aruco_commands),
        parameters=[{
            'use_sim_time': True,
            'show_camera': True,
            'cooldown_seconds': 2.0,
            'obstacle_stop_distance': 0.55,
            'search_for_markers': True,
            'search_angular_speed': 0.25
        }]
    )

    # RViz2 Node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',  # Explicit unique name
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        declare_use_rviz,
        declare_use_slam,
        declare_use_nav2,
        declare_use_aruco_commands,
        declare_headless,
        robot_state_publisher,
        gz_sim_headless,
        gz_sim_gui,
        spawn_robot,
        gz_bridge,
        image_bridge,
        relay_camera_info,
        slam_toolbox,
        nav2_bringup,
        aruco_command_follower,
        rviz2
    ])
