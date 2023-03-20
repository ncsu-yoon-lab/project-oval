import cv2 as cv
import numpy as np
import tensorflow as tf
keras = tf.keras

PI = 3.1415926

raw_img = cv.imread("test_img.jpg")
median_blur_img = cv.medianBlur(raw_img, 3)
grayscale_img = cv.cvtColor(median_blur_img, cv.COLOR_BGR2GRAY)
hsv_img = cv.cvtColor(median_blur_img, cv.COLOR_BGR2HSV)
red_mask = cv.bitwise_or(
    cv.inRange(hsv_img, (0, 50, 100), (10, 255, 255)),
    cv.inRange(hsv_img, (160, 100, 100), (180, 255, 255))
)
circles = cv.HoughCircles(
    image=grayscale_img,
    method=cv.HOUGH_GRADIENT,
    dp=1,
    minDist=len(grayscale_img) // 5,
    param1=200,
    param2=20,
    minRadius=len(grayscale_img) // 20,
    maxRadius=len(grayscale_img) // 4
)
masked_img = cv.cvtColor(cv.bitwise_and(raw_img, raw_img, mask=red_mask), cv.COLOR_BGR2GRAY)
best_num = 0
best = [0, 0, 0]
for circle in circles[0,:]:
    empty_mask = np.zeros_like(masked_img)
    cv.circle(
        img=empty_mask,
        center=(int(circle[0]), int(circle[1])),
        radius=int(circle[2]),
        color=(255, 255, 255),
        thickness=-1
    )
    double_masked_img = cv.bitwise_and(masked_img, empty_mask)
    percent_area = cv.countNonZero(double_masked_img) / (PI * (circle[2] ** 2))
    if percent_area > best_num:
        best = circle
        best_num = percent_area

img_y, img_x, chan = raw_img.shape
buffer = best[2] * 1.5

crop_x1 = max(int(best[0] - buffer), 0)
crop_x2 = min(int(best[0] + buffer), img_x)
crop_y1 = max(int(best[1] - buffer), 0)
crop_y2 = min(int(best[1] + buffer), img_y)

cropped_img = raw_img[crop_y1:crop_y2, crop_x1:crop_x2]

resized_img = cv.resize(cropped_img, (128, 128))

cv.imshow("display", resized_img)
cv.waitKey(0)

input_img = np.array([cv.cvtColor(resized_img, cv.COLOR_BGR2RGB)])

model = keras.models.load_model("stop_sign_model")
print(model(input_img).numpy().argmax())
