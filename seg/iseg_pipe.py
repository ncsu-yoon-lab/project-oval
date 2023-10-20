import torch
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as sensor_img
import PIL.Image as pil_img
import numpy as np
from cv_bridge import CvBridge
import threading
import tensorflow as tf
import keras
from transformers import TFSegformerForSemanticSegmentation, pipeline
import colorsys
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mplp
import time

setattr(torch.distributed, "is_initialized", lambda : False)
print(torch.cuda.device(0))  

g_img = None

labels = [
    "road",
    "sidewalk",
    "building",
    "wall",
    "fence",
    "pole",
    "traffic light",
    "traffic sign",
    "vegetation",
    "terrain",
    "sky",
    "person",
    "rider",
    "car",
    "truck",
    "bus",
    "train",
    "motorcycle",
    "bicycle"
]

def zed_callback(i: sensor_img) -> None:
    global g_img
    g_img = i

def fn(x):
    c = colorsys.hsv_to_rgb(*x)
    return [int(c[0] * 255), int(c[1] * 255), int(c[2] * 255)]

def main(args = None) -> None:
    global g_img

    # init ros2 env
    rclpy.init(args=args)

    # create node and sub to zed camera
    node = Node('img_seg_node')
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
    pipe = pipeline("image-segmentation", model="nvidia/segformer-b0-finetuned-cityscapes-1024-1024", device=0)

    # make bridge
    bridge = CvBridge()

    # generate n colors
    n = 19
    colors = [(x*1.0/n, 1, 1) for x in range(n)]
    colors = list(map(fn, colors))
    random.Random(38).shuffle(colors) # 38 27 107 117 132

    # prepare mpl fig
    fig, ax = plt.subplots()
    ax.set_title("segmented img")
    handles = []
    for i in range(n):
        handles.append(mplp.Patch(color=[x / 255 for x in colors[i]], label=labels[i], alpha=0.8))
    fig.legend(handles=handles, loc="upper right")
    plt.ion()

    plt.show()

    while rclpy.ok():

        if g_img == None:
            continue

        img = pil_img.fromarray(cv2.cvtColor(bridge.imgmsg_to_cv2(g_img), cv2.COLOR_BGR2RGB))

        bef_inf = time.time()

        # feed to model
        segs = pipe(img)

        aft_inf = time.time()
        print(aft_inf - bef_inf)

        bef_proc = time.time()

        img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

        consegs = img.copy()
        colsegs = img.copy()

        for i in range(len(segs)):

            # get mask as grayscale
            mask = np.asarray(segs[i]["mask"])

            # use grayscale mask to generate contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # draw new contours onto the collective contour img
            cv2.drawContours(consegs, contours, -1, (255, 255, 255), thickness=1)

            # make new color img
            color = np.full_like(colsegs, list(reversed(colors[labels.index(segs[i]["label"])])))

            # change mask to 3-channel img
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # use BGR mask to generate a color mask
            colsegs = np.where(mask > 0, color, colsegs)


        # weighted mask the original img with the color mask to make color segments
        final = cv2.addWeighted(img, 0.5, colsegs, 0.5, 0.0)

        # copy contours onto segmented img
        final = np.where(consegs == 255, consegs, final)

        aft_proc = time.time()
        print(aft_proc - bef_proc)

        # show img
        ax.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))
        plt.draw()
        plt.pause(0.001)

        # maintain rate
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()