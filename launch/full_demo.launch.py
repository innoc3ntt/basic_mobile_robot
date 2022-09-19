# Author: Addison Sears-Collins
# Date: September 2, 2021
# Description: Launch a basic mobile robot using the ROS 2 Navigation Stack
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode


def generate_launch_description():

    # Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    pkg_share = FindPackageShare(package="basic_mobile_robot").find(
        "basic_mobile_robot"
    )
    default_model_path = os.path.join(pkg_share, "models/basic_mobile_bot_v2.urdf")
    robot_localization_file_path = os.path.join(pkg_share, "config/ekf_gps_2.yaml")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/nav2_config.rviz")
    world_file_name = "basic_mobile_bot_world/smalltown.world"
    world_path = os.path.join(pkg_share, "worlds", world_file_name)
    nav2_dir = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_dir, "launch")
    static_map_path = os.path.join(pkg_share, "maps", "test.yaml")
    nav2_params_path = os.path.join(pkg_share, "params", "nav2_params.yaml")

    # Launch configuration variables specific to simulation
    autostart = LaunchConfiguration("autostart")
    headless = LaunchConfiguration("headless")
    map_yaml_file = LaunchConfiguration("map")
    model = LaunchConfiguration("model")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    slam = LaunchConfiguration("slam")
    use_namespace = LaunchConfiguration("use_namespace")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simulator = LaunchConfiguration("use_simulator")
    world = LaunchConfiguration("world")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name="namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        name="autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name="map",
        default_value=static_map_path,
        description="Full path to map file to load",
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name="params_file",
        default_value=nav2_params_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=default_rviz_config_path,
        description="Full path to the RVIZ config file to use",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Whether to execute gzclient",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name="slam", default_value="False", description="Whether to run SLAM"
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name="use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name="use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name="use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    declare_world_cmd = DeclareLaunchArgument(
        name="world",
        default_value=world_path,
        description="Full path to the world model file to load",
    )

    # ! PACKAGE SHARES

    gps_share = FindPackageShare(package="gps_waypoint_follower").find(
        "gps_waypoint_follower"
    )

    demo_share = FindPackageShare(package="nav2_gps_waypoint_follower_demo").find(
        "nav2_gps_waypoint_follower_demo"
    )

    # ! CALL OTHER LAUNCH FILES

    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_share, "/launch/basic_mobile_bot_v5.launch.py"]
        ),
        launch_arguments={}.items(),
    )

    gps_waypoint_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gps_share, "/launch/gps.launch.py"]),
        launch_arguments={}.items(),
    )

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [demo_share, "/launch/demo_gps_waypoint_follower.launch.py"]
        ),
        launch_arguments={}.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(system_launch)
    ld.add_action(gps_waypoint_follower_launch)

    return ld
