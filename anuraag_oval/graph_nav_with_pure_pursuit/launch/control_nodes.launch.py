from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    driver_node = Node(
        package="graph_nav_with_pure_pursuit",
        executable="driver",
        name="driver_node",
        output="screen",
    )

    pure_pursuit_node = Node(
        package="graph_nav_with_pure_pursuit",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        output="screen",
        parameters=[
            {
                "pose_topic": "/zed/zed_node/pose",
                "pose_message_type": "pose_stamped",
                "steering_topic": "/gemini/steering",
                "throttle_topic": "/gemini/throttle",
            }
        ],
    )

    xbox_controller_node = Node(
        package="graph_nav_with_pure_pursuit",
        executable="xbox_controller",
        name="xbox_controller_node",
        output="screen",
    )

    return LaunchDescription([driver_node, pure_pursuit_node, xbox_controller_node])
