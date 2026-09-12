import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

bag_path = "/mnt/ext-vol/zed2i_camera_imu_20260608_160033"
topic_name = "/zed/zed_node/imu/data"
out_path = "/mnt/ext-vol/zed2i_imu.csv"

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    ),
)

with open(out_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)

    writer.writerow([
        "bag_time_ns",
        "msg_time_sec",
        "msg_time_nanosec",
        "orientation_x",
        "orientation_y",
        "orientation_z",
        "orientation_w",
        "angular_velocity_x",
        "angular_velocity_y",
        "angular_velocity_z",
        "linear_acceleration_x",
        "linear_acceleration_y",
        "linear_acceleration_z",
    ])

    count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic != topic_name:
            continue

        msg = deserialize_message(data, Imu)

        writer.writerow([
            timestamp,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        count += 1

print(f"Done. Wrote {count} IMU rows to {out_path}")
