import os
import shlex
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package share directories ──────────────────────────────────────────
    pkg_description = get_package_share_directory("my_robot_description")
    pkg_bringup     = get_package_share_directory("my_robot_bringup")

    # ── File paths ─────────────────────────────────────────────────────────
    urdf_path         = os.path.join(pkg_description, "urdf", "my_robot.urdf.xacro")
    gazebo_config     = os.path.join(pkg_bringup,     "config", "gazebo_bridge.yaml")
    rviz_config       = os.path.join(pkg_description, "rviz",   "urdf_config.rviz")
    nav2_params       = os.path.join(pkg_bringup,     "config", "nav2_params.yaml")
    slam_params       = os.path.join(pkg_bringup,     "config", "slam_toolbox.yaml")
    world_path        = os.path.join(pkg_bringup,     "worlds", "my_world.sdf")

    # ── Run xacro to get robot_description (handles paths with spaces) ─────
    robot_description = subprocess.check_output(
        ["xacro", urdf_path], text=True
    )

    # ── Launch arguments ───────────────────────────────────────────────────
    headless_arg          = DeclareLaunchArgument("headless",          default_value="true")
    use_rviz_arg          = DeclareLaunchArgument("use_rviz",          default_value="true")
    use_slam_arg          = DeclareLaunchArgument("use_slam",          default_value="true")
    use_nav2_arg          = DeclareLaunchArgument("use_nav2",          default_value="true")
    use_aruco_arg         = DeclareLaunchArgument("use_aruco_commands",default_value="true")

    headless          = LaunchConfiguration("headless")
    use_rviz          = LaunchConfiguration("use_rviz")
    use_slam          = LaunchConfiguration("use_slam")
    use_nav2          = LaunchConfiguration("use_nav2")
    use_aruco         = LaunchConfiguration("use_aruco_commands")

    # ── Environment variable ───────────────────────────────────────────────
    set_gz_resource = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_bringup, "models"),
    )

    # ── robot_state_publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # ── Gazebo (headless = server only; no GUI) ────────────────────────────
    gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -s {shlex.quote(world_path)}"}.items(),
        condition=IfCondition(headless),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {shlex.quote(world_path)}"}.items(),
        condition=UnlessCondition(headless),
    )

    # ── Spawn robot from robot_description topic ───────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
        parameters=[{"use_sim_time": False}],
    )

    # ── ROS ↔ Gazebo bridge ────────────────────────────────────────────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": gazebo_config},
            {"use_sim_time": False},
        ],
    )

    # ── Camera image bridge (compressed) ──────────────────────────────────
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        parameters=[
            {"camera.image.compressed.jpeg_quality": 75},
            {"use_sim_time": False},
        ],
    )

    # ── Relay camera_info to match compressed image topic namespace ────────
    relay_camera_info = Node(
        package="topic_tools",
        executable="relay",
        name="relay_camera_info",
        parameters=[
            {"input_topic":  "camera/camera_info"},
            {"output_topic": "camera/image/camera_info"},
            {"use_sim_time": False},
        ],
    )

    # ── SLAM Toolbox ───────────────────────────────────────────────────────
    slam_toolbox_pkg = get_package_share_directory("slam_toolbox")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time":    "true",
            "slam_params_file": slam_params,
        }.items(),
        condition=IfCondition(use_slam),
    )

    # ── Nav2 ───────────────────────────────────────────────────────────────
    nav2_bringup_pkg = get_package_share_directory("nav2_bringup")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file":  nav2_params,
            "autostart":    "true",
        }.items(),
        condition=IfCondition(use_nav2),
    )

    # ── ArUco command follower ─────────────────────────────────────────────
    aruco_follower = Node(
        package="my_test_pkg",
        executable="aruco_command_follower",
        name="aruco_command_follower",
        output="screen",
        parameters=[
            {"use_sim_time":          True},
            {"show_camera":           True},
            {"cooldown_seconds":      2.0},
            {"obstacle_stop_distance":0.55},
            {"search_for_markers":    True},
            {"search_angular_speed":  0.25},
        ],
        condition=IfCondition(use_aruco),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # ── Assemble LaunchDescription ─────────────────────────────────────────
    return LaunchDescription([
        # args
        headless_arg,
        use_rviz_arg,
        use_slam_arg,
        use_nav2_arg,
        use_aruco_arg,
        # env
        set_gz_resource,
        # nodes / includes
        robot_state_publisher,
        gz_sim_headless,
        gz_sim_gui,
        spawn_robot,
        gz_bridge,
        image_bridge,
        relay_camera_info,
        slam,
        nav2,
        aruco_follower,
        rviz,
    ])
