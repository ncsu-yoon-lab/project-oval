from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {"autorepeat_rate": 20.0},  # xbox_controller treats >1s of silence as a lost controller
        ],
    )

    xbox_controller_node = Node(
        package="project_oval_control",
        executable="xbox_controller",
        name="xbox_controller_node",
        output="screen",
    )

    driver_node = Node(
        package="project_oval_control",
        executable="driver",
        name="driver_node",
        output="screen",
    )

    return LaunchDescription([joy_node, xbox_controller_node, driver_node])
