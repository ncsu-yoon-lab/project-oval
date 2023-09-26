import rclpy
from sensor_msgs.msg import Image as sensor_img
import cv2
import PIL.Image as pil_img
import numpy as np
from cv_bridge import CvBridge
import threading
from transformers import pipeline


g_img = cv2.Mat()
bridge = CvBridge()

def zed_callback(i: sensor_img) -> None:
    global g_img

    # sensor image -> cv2 image
    g_img = bridge.imgmsg_to_cv2(i)

def main(args = None) -> None:
    global g_img, bridge

    # init ros2 env
    rclpy.init(args=args)

    # create node and sub to zed camera
    node = rclpy.Node('img_seg_node')
    node.create_subscription(
        sensor_img,
        '/zed2i/zed_node/rgb/image_rect_color',
        zed_callback,
        10
    )

    # spin thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    # make rate
    rate = node.create_rate(10, node.get_clock())

    # load model
    pipe = pipeline("image-segmentation", model="nvidia/segformer-b5-finetuned-cityscapes-1024-1024")

    while rclpy.ok():

        if g_img == None:
            continue
            
        # cv2 image into PIL image
        img = pil_img.fromarray(np.asarray(cv2.cvtColor(g_img, cv2.COLOR_BGR2RGB)))

        # feed to model
        segments: list[dict[str, str | pil_img.Image]] = pipe(img)

        # TODO
        # ...

        # maintain rate
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()