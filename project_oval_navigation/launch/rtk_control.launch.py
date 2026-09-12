from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    localization_config_arg = DeclareLaunchArgument(
        "localization_config",
        default_value=PathJoinSubstitution([
            FindPackageShare("project_oval_navigation"),
            "config",
            "robot_localization.yaml",
        ]),
        description="Path to the robot_localization parameter file.",
    )

    localization_config = LaunchConfiguration("localization_config")

    ekf_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[localization_config],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[localization_config],
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
        parameters=[localization_config],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    driver_node = Node(
        package="project_oval_control",
        executable="driver",
        name="driver_node",
        output="screen",
    )

    pure_pursuit_node = Node(
        package="project_oval_navigation",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        output="screen",
        parameters=[
            {
                "pose_topic": "/odometry/filtered",
                "pose_message_type": "odometry",
                "steering_topic": "/gemini/steering",
                "throttle_topic": "/gemini/throttle",
            }
        ],
    )

    xbox_controller_node = Node(
        package="project_oval_control",
        executable="xbox_controller",
        name="xbox_controller_node",
        output="screen",
    )

    return LaunchDescription([
        localization_config_arg,
        ekf_odom,
        navsat_transform,
        ekf_map,
        driver_node,
        pure_pursuit_node,
        xbox_controller_node,
    ])
