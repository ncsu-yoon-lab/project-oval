from ultralytics import YOLO
import numpy as np
import cv2

model = YOLO("yolo11n.pt") 
results = model.predict("bus.jpg")  # Predict the image
frame = cv2.imread("bus.jpg")

for pred in results:
    boxes = pred.boxes
    for box in boxes:
        box = box.numpy()
        # print(box[0]) 
        label = model.names.get(box.cls.item())
        (x,y,w,h) = box.xywh[0]
        conf = box.conf.item()
        cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0,255,0), 2)
        cv2.putText(frame, label, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        print(f"Label: {label}, x: {x}, y: {y}, w: {w}, h: {h}")
        print(f"Confidence: {conf}")

        cv2.imwrite("detections.jpg", frame)

