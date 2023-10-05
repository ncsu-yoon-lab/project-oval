from transformers import pipeline
from PIL import Image
import colorsys
from matplotlib import patches as mplp, pyplot as plt
import numpy as np
import random
import cv2

# load model
pipe = pipeline("image-segmentation", model="nvidia/segformer-b5-finetuned-cityscapes-1024-1024")

# open img
pilimg = Image.open("c.jpg") # .resize((1024,1024))

# segment img
segments = pipe(pilimg)
print(type(segments[0]['mask']))

# segs as np arrays
npsegs = [ (x["label"], np.asarray(x["mask"])) for x in segments ]

# get number of segments
n = len(segments)

# convert img to the better format
img = cv2.cvtColor(np.asarray(pilimg), cv2.COLOR_RGB2BGR)

def fn(x):
    c = colorsys.hsv_to_rgb(*x)
    print(c)
    return [int(c[0] * 255), int(c[1] * 255), int(c[2] * 255)]

# generate n colors
colors = [(x*1.0/n, 1, 1) for x in range(n)]
colors = list(map(fn, colors))
random.Random(38).shuffle(colors) # 38 27 107 117 132

# prepare figure
fig, axes = plt.subplots(nrows=2, ncols=3)
handles = []

consegs = img.copy()
colsegs = img.copy()
for i in range(0, n):
    label = segments[i]["label"]

    if label == "road":
        road_idx = i
    elif label == "traffic light":
        tl_idx = i
    elif label == "traffic sign":
        ts_idx = i
    elif label == "car":
        car_idx = i

    # get mask as grayscale
    mask = np.asarray(segments[i]["mask"])

    # use grayscale mask to generate contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # draw new contours onto the collective contour img
    cv2.drawContours(consegs, contours, -1, (255, 255, 255), thickness=1)

    # make new color img
    color = np.full_like(colsegs, list(reversed(colors[i])))

    # change mask to 3-channel img
    mask = cv2.cvtColor(np.asarray(segments[i]["mask"]), cv2.COLOR_GRAY2BGR)
    
    # use BGR mask to generate a color mask
    colsegs = np.where(mask == 255, color, colsegs)

    # append legend to list
    handles.append(mplp.Patch(color=[x / 255 for x in colors[i]], label=segments[i]["label"], alpha=0.8))

# weighted mask the original img with the color mask to make color segments
final = cv2.addWeighted(img, 0.5, colsegs, 0.5, 0.0)

# copy contours onto segmented img
final = np.where(consegs == 255, consegs, final)

# set color legend
fig.legend(handles=handles, loc="upper right")

# show
axes[0][0].set_title("original img")
axes[0][1].set_title("segmented img")
axes[0][2].set_title("car mask")
axes[1][0].set_title("road mask")
axes[1][1].set_title("traffic light mask")
axes[1][2].set_title("traffic sign mask")
axes[0][0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
axes[0][1].imshow(cv2.cvtColor(final, cv2.COLOR_BGR2RGB))
axes[0][2].imshow(cv2.cvtColor(cv2.copyTo(img, np.asarray(segments[car_idx]["mask"])), cv2.COLOR_BGR2RGB))
axes[1][0].imshow(cv2.cvtColor(cv2.copyTo(img, np.asarray(segments[road_idx]["mask"])), cv2.COLOR_BGR2RGB))
axes[1][1].imshow(cv2.cvtColor(cv2.copyTo(img, np.asarray(segments[tl_idx]["mask"])), cv2.COLOR_BGR2RGB))
axes[1][2].imshow(cv2.cvtColor(cv2.copyTo(img, np.asarray(segments[ts_idx]["mask"])), cv2.COLOR_BGR2RGB))


plt.show()
