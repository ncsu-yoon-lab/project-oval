from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    zed_camera = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "zed_wrapper",
            "zed_camera.launch.py",
            "camera_model:=zed2i",
        ],
        output="screen",
    )

    gps_driver = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "nmea_navsat_driver",
            "nmea_serial_driver",
            "--ros-args",
            "-p",
            "port:=/dev/ttyACM0",
            "-p",
            "baud:=9600",
            "-p",
            "frame_id:=gps",
        ],
        output="screen",
    )

    joy_node = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "joy",
            "joy_node",
            "--ros-args",
            "-p",
            "autorepeat_rate:=0.0",
        ],
        output="screen",
    )

    return LaunchDescription([zed_camera, gps_driver, joy_node])
