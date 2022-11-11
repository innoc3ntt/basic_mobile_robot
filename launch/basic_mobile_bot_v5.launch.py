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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Set the path to different files and folders.
    nav2_dir = get_package_share_directory("nav2_bringup")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_share = get_package_share_directory("basic_mobile_robot")

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                name="use_namespace",
                default_value="false",
                description="Whether to apply a namespace to the navigation stack",
            ),
            DeclareLaunchArgument(
                name="autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=os.path.join(
                    pkg_share, "models/basic_mobile_bot_v2.urdf"
                ),
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="params_file",
                default_value=os.path.join(pkg_share, "params/nav2_params.yaml"),
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                name="rviz_config_file",
                default_value=os.path.join(pkg_share, "rviz/nav2_config.rviz"),
                description="Full path to the RVIZ config file to use",
            ),
            DeclareLaunchArgument(
                name="headless",
                default_value="False",
                description="Whether to execute gzclient",
            ),
            DeclareLaunchArgument(
                name="slam", default_value="False", description="Whether to run SLAM"
            ),
            DeclareLaunchArgument(
                name="use_robot_state_pub",
                default_value="True",
                description="Whether to start the robot state publisher",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Whether to start RVIZ",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                name="use_simulator",
                default_value="True",
                description="Whether to start the simulator",
            ),
            DeclareLaunchArgument(
                name="world",
                default_value=os.path.join(pkg_share, "worlds/smalltown.world"),
                description="Full path to the world model file to load",
            ),
            DeclareLaunchArgument(
                name="map",
                default_value=os.path.join(pkg_share, "maps/smalltown_world.yaml"),
                description="Full path to map file to load",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                ),
                condition=IfCondition(use_simulator),
                launch_arguments={"world": world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                ),
                condition=IfCondition(
                    PythonExpression([use_simulator, " and not ", headless])
                ),
            ),
            Node(
                condition=IfCondition(use_robot_state_pub),
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": Command(["xacro ", model]),
                    }
                ],
                remappings=remappings,
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, "launch/bringup_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_namespace": use_namespace,
                    "slam": slam,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                }.items(),
            ),
        ]
    )
