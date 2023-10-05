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

g_img = None

labels = {
    "0": "road",
    "1": "sidewalk",
    "2": "building",
    "3": "wall",
    "4": "fence",
    "5": "pole",
    "6": "traffic light",
    "7": "traffic sign",
    "8": "vegetation",
    "9": "terrain",
    "10": "sky",
    "11": "person",
    "12": "rider",
    "13": "car",
    "14": "truck",
    "15": "bus",
    "16": "train",
    "17": "motorcycle",
    "18": "bicycle"
}

def zed_callback(i: sensor_img) -> None:
    global g_img
    g_img = i

def fn(x):
    c = colorsys.hsv_to_rgb(*x)
    print(c)
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
    model = keras.models.load_model("segformer")
    model.summary()
    model.compile()
    # pipe = pipeline(
    #     "image-segmentation",
    #     model="nvidia/segformer-b5-finetuned-cityscapes-1024-1024",
    #     framework="pt")

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
        handles.append(mplp.Patch(color=[x / 255 for x in colors[i]], label=labels[str(i)], alpha=0.8))
    fig.legend(handles=handles, loc="upper right")
    plt.ion()

    plt.show()

    while rclpy.ok():

        if g_img == None:
            continue

        img = cv2.cvtColor(
            cv2.resize(
                bridge.imgmsg_to_cv2(g_img),
                (512, 512)
            ),
            cv2.COLOR_BGRA2RGB
        )

        # feed to model
        segments = model({"pixel_values":
            tf.cast(
                tf.convert_to_tensor(
                    np.moveaxis(
                        np.asarray([
                            img
                        ]),
                        -1, 1
                    ),
                    name="pixel_values/pixel_values"
                ),
                tf.float32
            )
        })

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        consegs = img.copy()
        colsegs = img.copy()

        for i in range(n):

            # get mask as grayscale
            mask = cv2.resize(
                    cv2.normalize(
                        np.clip(
                            np.asarray(segments["logits"][0][i]),
                            a_min=0,
                            a_max=None
                        ),
                        None, 0, 255, cv2.NORM_MINMAX
                    ).astype(np.uint8),
                (512, 512)
            )

            print(mask)

            # use grayscale mask to generate contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # draw new contours onto the collective contour img
            cv2.drawContours(consegs, contours, -1, (255, 255, 255), thickness=1)

            # make new color img
            color = np.full_like(colsegs, list(reversed(colors[i])))

            # change mask to 3-channel img
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

            # use BGR mask to generate a color mask
            colsegs = np.where(mask > 0, color, colsegs)


        # weighted mask the original img with the color mask to make color segments
        final = cv2.addWeighted(img, 0.5, colsegs, 0.5, 0.0)

        # copy contours onto segmented img
        final = np.where(consegs == 255, consegs, final)

        # show img
        ax.imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))
        plt.draw()
        plt.pause(0.001)

        # maintain rate
        rate.sleep()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
