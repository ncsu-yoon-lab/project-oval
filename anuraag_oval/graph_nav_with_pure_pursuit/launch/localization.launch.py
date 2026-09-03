from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution([
            FindPackageShare("graph_nav_with_pure_pursuit"),
            "config",
            "robot_localization.yaml",
        ]),
        description="Path to the robot_localization parameter file.",
    )

    config = LaunchConfiguration("config")

    ekf_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[config],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[config],
        remappings=[
            ("imu", "/imu"),
            ("gps/fix", "/navsatfix"),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
            ("gps/filtered", "/gps/filtered"),
        ],
    )

    ekf_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[config],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    return LaunchDescription([
        config_arg,
        ekf_odom,
        navsat_transform,
        ekf_map,
    ])
