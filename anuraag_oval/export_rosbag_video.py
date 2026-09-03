import os
import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

bag_path = "/mnt/ext-vol/zed2i_camera_imu_20260608_160033"
# topic_name = "/zed/zed_node/rgb/color/rect/image"
# out_path = "/mnt/ext-vol/zed2i_rgb.mp4"
topic_name = "/zed/zed_node/depth/depth_registered"
out_path = "/mnt/ext-vol/zed2i_depth.mp4"

# fps = 10.9
fps = 10.6
max_depth_m = 10.0

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    ),
)

bridge = CvBridge()
writer = None
count = 0

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic != topic_name:
        continue

    msg = deserialize_message(data, Image)
    
    ## for exporting color
    # frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    ## for exporting depth
    # depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # print_once = count == 0
    # if print_once:
    #     print(f"Depth encoding: {msg.encoding}")
    #     print(f"Depth dtype: {depth.dtype}")
    # if depth.dtype == np.uint16:
    #     depth_m = depth.astype(np.float32) / 1000.0
    # else:
    #     depth_m = depth.astype(np.float32)
    # depth_m = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
    # normalized = np.clip(depth_m / max_depth_m, 0.0, 1.0)
    # gray = (normalized * 255).astype(np.uint8)
    # frame = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)

    if writer is None:
        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        print(f"Writing {out_path} at {width}x{height}, {fps} fps")

    writer.write(frame)
    count += 1

    if count % 100 == 0:
        print(f"Wrote {count} frames")

if writer is not None:
    writer.release()

print(f"Done. Wrote {count} frames to {out_path}")
