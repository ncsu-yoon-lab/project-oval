import cv2
import numpy as np
from ultralytics import YOLO
import time
import cv2 
from PIL import Image
import numpy as np
import time
import json
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os
import torch.nn.functional as F  # Make sure this import is here

class LatLonModel(nn.Module):
    def __init__(self):
        super(LatLonModel, self).__init__()
        
        # LSTM layer
        self.lstm = nn.LSTM(input_size=72, hidden_size=128, num_layers=2, batch_first=True, bidirectional=True, dropout=0.2)
        
        # Fully connected layers with consistent sizes
        self.fc1 = nn.Linear(256, 256)  # Keep the output size same as input for skip connection
        self.fc2 = nn.Linear(256, 128)  # Reduce size in the second layer
        self.fc3 = nn.Linear(128, 2)    # Final layer for output

        # Batch normalization layers
        self.bn1 = nn.BatchNorm1d(256)
        
        # Dropout
        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        x = x.unsqueeze(1)  # Add sequence dimension for LSTM
        lstm_out, _ = self.lstm(x)
        x = lstm_out[:, -1, :]  # Take the output of the last time step
        
        # First fully connected layer with skip connection
        residual = x  # Save input for the skip connection
        x = F.leaky_relu(self.bn1(self.fc1(x)))
        x += residual  # Add skip connection
        
        x = self.dropout(x)
        
        # Second fully connected layer (no skip connection here)
        x = F.leaky_relu(self.fc2(x))
        x = self.fc3(x)
        
        return x

class LatLonPrediction:
    def __init__(self, model_path, model_object):
        self.model = self.load_model(model_object, model_path)

    def load_model(self, model_object, model_path):
        model_object.load_state_dict(torch.load(model_path))
        return model_object

class YOLOModel:

    def __init__(self, model_path, lat_lon_model):
        self.model = self.load_model(model_path)
        self.CUSTOM_CLASS_NAMES = ["FW", "HUNT", "OVAL", "engineering-building", "security-station", "sign", "street-lamp", "trashcan"]
        self.lat_lon_model = lat_lon_model

    def load_model(self, model_path):
        model = YOLO(model_path)
        return model
    
    def predict_from_frame(self, frame):
        predictions = self.model(frame)

        return predictions
    
    def create_bbox_tensor(self, bbox, class_id):
        """
        Convert bounding box and class_id to a tensor matching the input size of LatLonModel.
        """
        print(f"Bounding box as param{bbox}")
        print(class_id)
        max_instances_per_class = 2
        num_classes = 9
        data_points = 4
        input_size = num_classes * max_instances_per_class * data_points
        input_vector = [0.0] * input_size  # Initialize input_vector with float values

        length = 1280.0
        width = 720.0

        # The value receiving from bbox is in tensor format so we need to convert to list to be able to manipulate it
        bbox_list = bbox.tolist()
        print(f"Should be list: {bbox_list}")

        # Normalize the bounding box coordinates
        bbox_list[0] = bbox_list[0] / length
        bbox_list[1] = bbox_list[1] / width
        bbox_list[2] = bbox_list[2] / length
        bbox_list[3] = bbox_list[3] / width

        instance_index = 0  # Assuming a single instance per class
        start_index = int((class_id * max_instances_per_class + instance_index) * 4)
        print(f"Start index: {start_index}")

        # Assign the normalized bounding box coordinates to the input_vector slice
        input_vector[start_index:start_index + 4] = bbox_list[:4] 
        print(input_vector)

        # Convert to a tensor and add a batch dimension
        bbox_tensor = torch.tensor(input_vector, dtype=torch.float32).unsqueeze(0)  # Batch size 1
        return bbox_tensor

    
    def start_camera(self):
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error opening the camera")
            return
        
        start = time.time()  # Start time
        frame_count = 0

        while True:

            ret, frame = cap.read()
            
            if not ret:
                print("Failed to grab frame")
                break

            # Convert frame to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            predictions = self.predict_from_frame(frame_rgb)
            
            
            for result in predictions:
                boxes = result.boxes.xyxy
                confidences = result.boxes.conf
                class_ids = result.boxes.cls


                for box, confidence, class_id in zip(boxes, confidences, class_ids):
                    x_min, y_min, x_max, y_max = map(int, box)
                    label = self.CUSTOM_CLASS_NAMES[int(class_id)]
                    score = float(confidence)
                    text = f'{label}: {score:.2f}'
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.putText(frame, text, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    
                    # (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

                    # cv2.rectangle(frame, (x_min, y_min - text_height - 10), (x_min + text_width, y_min), (0, 255, 0), -1)
                    # cv2.putText(frame, text, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                    # Predicting the lat lon model
                    bbox_tensor = self.create_bbox_tensor(box, class_id)
                    predicted_lat_lon = self.predict_lat_lon(bbox_tensor)
                    
                    print(f"Predicted Lat/Lon: {predicted_lat_lon}")

            frame_count += 1

            end = time.time()  # End time
            elapsed_time = end - start  # Time taken for one loop iteration
            
            if elapsed_time >= 60:
                print(f"Total Inference Time: {frame_count}")
                print(f"Frames per second: {frame_count / 60}")
                break 
            # print(f"Inference Time: {inference_time:.4f} seconds")  # Print inference time
            
            cv2.imshow('Object Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 
        
        cap.release()
        cv2.destroyAllWindows()

    def predict_lat_lon(self, bbox_tensor):
        self.lat_lon_model.eval()
        with torch.no_grad():
            lat_lon = self.lat_lon_model(bbox_tensor)
        return lat_lon.squeeze().numpy()



model_path = "best.pt"
lat_lon_path = "lat_lon_model_gps.pt"

# Loading the lat and lon model
lat_lon_model = LatLonModel()
lat_lon_model.load_state_dict(torch.load(lat_lon_path))

# Loading the yolo model
bbox_detector = YOLOModel(model_path, lat_lon_model)

bbox_detector.start_camera()