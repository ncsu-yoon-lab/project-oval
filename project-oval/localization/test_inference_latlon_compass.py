import cv2
import csv
import numpy as np
from ultralytics import YOLO
import time
from PIL import Image
import json
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
import matplotlib.pyplot as plt
import os
import torch.nn.functional as F  # Make sure this import is here
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.backbone_utils import resnet_fpn_backbone
import torchvision.transforms as transforms
from torch.utils.data import DataLoader, Dataset

class LatLonModelLSTM(nn.Module):
    def __init__(self):
        super(LatLonModelLSTM, self).__init__()
        
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

class CompassModelLSTM(nn.Module):
    def __init__(self):
        super(CompassModelLSTM, self).__init__()
        
        self.lstm = nn.LSTM(input_size=75, hidden_size=256, num_layers=2, batch_first=True, bidirectional=True, dropout=0.2)
        
        self.fc1 = nn.Linear(512, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 1)
        
        self.bn1 = nn.BatchNorm1d(256)
        self.bn2 = nn.BatchNorm1d(128)
        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        x = x.unsqueeze(1)
        lstm_out, _ = self.lstm(x)
        x = lstm_out[:, -1, :]
        
        
        x = F.leaky_relu(self.bn1(self.fc1(x)))
        
        
        x = F.leaky_relu(self.bn2(self.fc2(x)))
        x = self.dropout(x)
        x = F.leaky_relu(self.fc3(x))
        x = self.fc4(x)
        return x

class CompassModelFFNN(nn.Module):
    def __init__(self):
        super(CompassModelFFNN, self).__init__()

        self.fc1 = nn.Linear(64, 256)
        self.bn1 = nn.BatchNorm1d(256)

        self.fc2 = nn.Linear(256, 128)
        self.bn2 = nn.BatchNorm1d(128)

        self.fc3 = nn.Linear(128, 64)
        self.bn3 = nn.BatchNorm1d(64)

        self.fc4 = nn.Linear(64, 32)
        self.bn4 = nn.BatchNorm1d(32)

        self.fc5 = nn.Linear(32, 1)
        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        x = F.leaky_relu(self.bn1(self.fc1(x)))
        x = self.dropout(x)
        x = self.fc2(x)
        x = self.fc3(x)
        x = self.fc4(x)
        x = self.fc5(x)

        return x

class LatLonModelFFNN(nn.Module):
    def __init__(self):
        super(LatLonModelFFNN, self).__init__()
        self.fc1 = nn.Linear(72, 256)
        self.bn1 = nn.BatchNorm1d(256)

        self.fc2 = nn.Linear(256, 128)
        self.bn2 = nn.BatchNorm1d(128)

        self.fc3 = nn.Linear(128, 64)
        self.bn3 = nn.BatchNorm1d(64)

        self.fc4 = nn.Linear(64, 32)
        self.bn4 = nn.BatchNorm1d(32)

        self.fc5 = nn.Linear(32, 2)

        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        x = F.leaky_relu(self.bn1(self.fc1(x)))
        x = self.dropout(x)
        x = self.fc2(x)
        x = self.fc3(x)
        x = self.fc4(x)
        x = self.fc5(x)

        return x

class LSTMCompassModel(nn.Module):
    def __init__(self):
        super(LSTMCompassModel, self).__init__()
    
        self.lstm = nn.LSTM(input_size=64, hidden_size=128, num_layers=2, batch_first=True, bidirectional=True)

        self.fc1 = nn.Linear(256, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 1)

        self.bn1 = nn.BatchNorm1d(256)

        self.dropout = nn.Dropout(p=0.3)

    def forward(self, x):
        x = x.unsqueeze(1)
        lstm_out, _ = self.lstm(x)
        x = lstm_out[:, -1, :]

        residual = x
        x = F.leaky_relu(self.bn1(self.fc1(x)))
        x += residual

        x = self.dropout(x)

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

    def __init__(self, model_path, lat_lon_model, compass_model):
        self.model = self.load_model(model_path)
        self.CUSTOM_CLASS_NAMES = ["FW", "HUNT", "OVAL", "engineering-building", "security-station", "sign", "street-lamp", "trashcan"]
        self.lat_lon_model = lat_lon_model
        self.compass_model = compass_model

    def load_model(self, model_path):
        model = YOLO(model_path)
        return model
    
    def predict_from_frame(self, frame):
        predictions = self.model(frame)

        return predictions

    def start_image_inference(self, image_path):
        # Load the image from the provided path
        frame = cv2.imread(image_path)
        if frame is None:
            print("Error loading image")
            return

        start = time.time()  # Start time

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

                # Predicting the lat lon model
                bbox_tensor = self.create_bbox_tensor(box, class_id)
                predicted_lat_lon = self.predict_lat_lon(bbox_tensor)

                print(f"Predicted Lat/Lon: {predicted_lat_lon}")

        end = time.time()  # End time
        elapsed_time = end - start  # Time taken for inference
        print(start)
        print(end)
        print(f"Inference time: {elapsed_time:.4f} seconds")

        # Show the result
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def start_camera(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error opening the camera")
            return
        
        start = time.time()  # Start time
        frame_count = 0
        i = 0
        
        # List to store the time taken for each image processing
        processing_times = []

        while True:
            ret, frame = cap.read()
            if i == 0:
                first_image_start = time.time()
                i += 1

            if not ret:
                print("Failed to grab frame")
                break

            # Start the timer for this frame
            frame_start_time = time.time()

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

                    # Predicting the lat lon model
                    bbox_tensor = self.create_bbox_tensor(box, class_id)
                    predicted_lat_lon = self.predict_lat_lon(bbox_tensor)
                    
                    print(f"Predicted Lat/Lon: {predicted_lat_lon}")

            frame_count += 1
            first_image_end = time.time()
            inference_first_image = first_image_end - first_image_start

            # End the timer for this frame and store the elapsed time
            frame_end_time = time.time()
            time_taken = frame_end_time - frame_start_time
            processing_times.append(time_taken)

            # Calculate the total elapsed time
            elapsed_time = frame_end_time - start

            if elapsed_time >= 60:
                print(f"Total Inference Time: {frame_count}")
                print(f"Time it took to predict the first image: {inference_first_image}")
                print(f"Frames per second: {frame_count / 60}")
                break

            cv2.imshow('Object Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        # Print out the list of times taken to process each image
        print("Processing times for each frame (seconds):")
        print(f"Number of frames Predicted: {len(processing_times)}")
        print(f"On Average it took {sum(processing_times) / len(processing_times)} seconds")
        print(processing_times)

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
        return bbox_tensor, input_vector

    def predict_lat_lon(self, bbox_tensor):
        self.lat_lon_model.eval()
        with torch.no_grad():
            lat_lon = self.lat_lon_model(bbox_tensor)
        return lat_lon.squeeze().numpy()

    def predict_compass(self, bbox_tensor):
        self.compass_model.eval()
        with torch.no_grad():
            compass = self.compass_model(bbox_tensor)
        return compass.squeeze().numpy()

    def predict_record_inferences_for_images_in_folder(self, folder_path, output_csv):
        processing_times=[]

        with open(output_csv, mode="w", newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Image Name', 'Image Processing', 'Lat Lon Processing', 'Compass Processing', 'Total Time'])

            # Iterarte over all files 
            for filename in os.listdir(folder_path):
                print(f"Filename is {filename}")
                if filename.endswith(('.jpg')):
                    image_path = os.path.join(folder_path, filename)
                    image = cv2.imread(image_path)

                    if image is None:
                        print(f"IMage not found {image}")
                        continue
                        
                    start_time = time.time()

                    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                    # Image Inference
                    image_start_time = time.time()
                    predictions = self.predict_from_frame(image_rgb)
                    image_end_time = time.time()
                    final_image_time = image_end_time - image_start_time


                    for result in predictions:
                        boxes = result.boxes.xyxy
                        confidences = result.boxes.conf
                        class_ids = result.boxes.cls

                        for box, confidence, class_id in zip(boxes, confidences, class_ids):
                            x_min, y_min, x_max, y_max = map(int, box)
                            label = self.CUSTOM_CLASS_NAMES[int(class_id)]
                            score = float(confidence)
                            text = f'{label}: {score:.2f}'
                            cv2.rectangle(image_rgb, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                            cv2.putText(image_rgb, text, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                            # Predicting the lat lon model
                            bbox_tensor, bbox_list = self.create_bbox_tensor(box, class_id)
                            bbox_list_sliced = bbox_list[8:]
                            bbox_tensor_compass = torch.tensor(bbox_list_sliced, dtype=torch.float32).unsqueeze(0)  # Batch size 1

                            # Lat Lon inference
                            lat_lon_start_time = time.time()
                            predicted_lat_lon = self.predict_lat_lon(bbox_tensor)
                            lat_lon_end_time = time.time()
                            final_lat_lon_time = lat_lon_end_time - lat_lon_start_time

                            # Compass Inference
                            compass_start_time = time.time()
                            predict_compass = self.predict_compass(bbox_tensor_compass)
                            compass_end_time = time.time()
                            final_compass_time = compass_end_time - compass_start_time

                            print(f"Predicted Lat/Lon: {predicted_lat_lon}")
                            print(f"Predicted Compass: {predict_compass}")

                    end_time = time.time()
                    time_taken = end_time - start_time 

                    processing_times.append(time_taken)

                    writer.writerow([filename,final_image_time, final_lat_lon_time, final_compass_time, time_taken])
            print("Processing times for each image (seconds):")
            print(f"Number of images processed: {len(processing_times)}")
            print(f"Average processing time: {sum(processing_times) / len(processing_times)} seconds")

    

class BBoxDetectionRCNN:
    def __init__(self, model_path, lat_lon_model, compass_model):
        """Constructor that loads the pre-trained Faster RCNN model using torch

        Args:
            model_path (string): Path to the model 
        """
        self.backbone = resnet_fpn_backbone('resnet18', pretrained=True)

        # Loads the pretrained model RCNN
        self.model = torchvision.models.detection.FasterRCNN(self.backbone, num_classes=91)  # Default COCO classes

        # Get the number of input features from the classifier
        self.in_feature = self.model.roi_heads.box_predictor.cls_score.in_features
        
        self.num_classes = 9 + 1 # 9 classes + 1 for background class
        
        # Changing the head of the box_predictor to our dataset class
        self.model.roi_heads.box_predictor = FastRCNNPredictor(self.in_feature, self.num_classes)
        
        self.model_path = model_path
        self.lat_lon_model = lat_lon_model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # Move the model to the device
        self.model.to(self.device)
        self.CUSTOM_CLASS_NAMES = ["sign", "FW", "HUNT", "OVAL", "engineering-building", "security-station", "sign", "street-lamp", "trashcan"]
        self.compass_model = compass_model
        self.compass_model.to(self.device)



    def load_RCNN_model(self): 
        """
        Function that loads the Trained Model that is used to identify 9 different classes
        """
        
        # Load the train model  
        self.model.load_state_dict(torch.load(self.model_path))
        self.model.eval()
    
    def quantize_model(self):
        """
        Function that applies quantization to the model
        """
        self.model = torch.quantization.quantize_dynamic(self.model, {torch.nn.Linear}, dtype=torch.qint8)

    
    
    def create_bbox_tensor(self, box, class_id):
        """
        Converts a bounding box and class ID into a tensor.
        box: A list or array of [x_min, y_min, x_max, y_max].
        class_id: An integer representing the class of the object.
        
        Returns:
            bbox_tensor: A tensor containing the normalized bounding box and class ID.
        """
        # Extract bounding box coordinates
        x_min, y_min, x_max, y_max = box
        
        # Optionally normalize the bounding box coordinates (if needed)
        # For simplicity, we'll just convert the coordinates without normalization
        bbox_data = [x_min, y_min, x_max, y_max, class_id]

        # Ensure the tensor is of shape (1, 72)
        bbox_tensor = torch.zeros((1, 72), dtype=torch.float32).to(self.device)
        bbox_tensor[0, :len(bbox_data)] = torch.tensor(bbox_data, dtype=torch.float32)
        
        return bbox_tensor


    def start_camera(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error opening the camera")
            return
        
        start = time.time()  # Start time
        frame_count = 0
        i = 0
        
        # List to store the time taken for each image processing
        processing_times = []

        while True:
            ret, frame = cap.read()
            if i == 0:
                first_image_start = time.time()
                i += 1

            if not ret:
                print("Failed to grab frame")
                break

            # Start the timer for this frame
            frame_start_time = time.time()

            # Convert frame to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            predictions = self.predict_from_frame(frame_rgb)

            for result in predictions:
                boxes = result["boxes"]
                confidences = result["scores"]
                class_ids = result["labels"]

                for box, confidence, class_id in zip(boxes, confidences, class_ids):
                    x_min, y_min, x_max, y_max = map(int, box)
                    label = self.CUSTOM_CLASS_NAMES[int(class_id)]
                    score = float(confidence)
                    text = f'{label}: {score:.2f}'
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.putText(frame, text, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                    # Predicting the lat lon model
                    bbox_tensor = self.create_bbox_tensor(box, class_id)
                    predicted_lat_lon = self.predict_lat_lon(bbox_tensor)
                    
                    print(f"Predicted Lat/Lon: {predicted_lat_lon}")

            frame_count += 1
            first_image_end = time.time()
            inference_first_image = first_image_end - first_image_start

            # End the timer for this frame and store the elapsed time
            frame_end_time = time.time()
            time_taken = frame_end_time - frame_start_time
            processing_times.append(time_taken)

            # Calculate the total elapsed time
            elapsed_time = frame_end_time - start

            if elapsed_time >= 60:
                print(f"Total Inference Time: {frame_count}")
                print(f"Time it took to predict the first image: {inference_first_image}")
                print(f"Frames per second: {frame_count / 60}")
                break

            cv2.imshow('Object Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        # Print out the list of times taken to process each image
        print("Processing times for each frame (seconds):")
        print(f"Number of frames Predicted: {len(processing_times)}")
        print(f"On Average it took {sum(processing_times) / len(processing_times)} seconds")
        print(processing_times)
    
    def predict_lat_lon(self, bbox_tensor):
        self.lat_lon_model.eval()
        bbox_tensor = bbox_tensor.to(self.device)
    
        # Make sure the lat_lon_model is on the same device
        self.lat_lon_model = self.lat_lon_model.to(self.device)
        
        # Make the prediction
        lat_lon = self.lat_lon_model(bbox_tensor)
    
        return lat_lon

    def predict_compass(self, bbox_tensor):
        self.compass_model.eval()
        bbox_tensor = bbox_tensor.to(self.device)

        self.compass_model = self.compass_model.to(self.device)

        compass = self.compass_model(bbox_tensor)

        return compass

    def predict_from_frame(self, frame):
        """
        Function that first creates a pipeline to convert normal nparry -> tensor array. 
        Then makes prediction using the model 
        """
        self.model.eval()

        # Convert the frame (NumPy array) to a PyTorch tensor
        # First convert it from HWC format (Height, Width, Channels) to CHW (Channels, Height, Width)
        frame_tensor = torch.tensor(frame).permute(2, 0, 1).float()

        frame_tensor = frame_tensor / 255.0

        # Add batch dimension (B, C, H, W)
        frame_tensor = frame_tensor.unsqueeze(0)

        # Move the tensor to the device (CPU or GPU) the model is on
        frame_tensor = frame_tensor.to(self.device)

        # Perform prediction
        with torch.no_grad():
            predictions = self.model(frame_tensor)

        return predictions

    def predict_record_inferences_for_images_in_folder(self, folder_path, output_csv):
        # List to store the time taken for each image processing
        processing_times = []

        # Open CSV file for writing
        with open(output_csv, mode='w', newline='') as file:
            print("Opening CSV file")
            writer = csv.writer(file)
            writer.writerow(['Image Name', 'Image Processing', 'Lat Lon Processing', 'Compass Processing', 'Total Time'])

            # Iterate over all files in the folder
            for filename in os.listdir(folder_path):
                if filename.endswith(('.png', '.jpg', '.jpeg')):  # Ensure it's an image file
                    image_path = os.path.join(folder_path, filename)
                    print(f"Processing image: {filename}")
                    
                    # Read and process the image
                    image = cv2.imread(image_path)
                    if image is None:
                        print(f"Failed to load image: {filename}")
                        continue

                    # Timing the image processing
                    image_start_time = time.time()
                    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    predictions = self.predict_from_frame(image_rgb)
                    image_end_time = time.time()
                    final_image_time = image_end_time - image_start_time

                    # Process predictions
                    bbox_data = self.extract_bbox_data(predictions)
                    bbox_sliced = bbox_data[8:]  # Adjust slicing as needed
                    bbox_tensor = torch.tensor(bbox_data).unsqueeze(0).to(self.device)
                    bbox_sliced_tensor = torch.tensor(bbox_sliced).unsqueeze(0).to(self.device)

                    # Lat/Lon prediction
                    lat_lon_start_time = time.time()
                    lat_lon = self.predict_lat_lon(bbox_tensor)
                    lat_lon_end_time = time.time()
                    final_lat_lon_time = lat_lon_end_time - lat_lon_start_time

                    # Compass prediction
                    compass_start_time = time.time()
                    compass_prediction = self.predict_compass(bbox_sliced_tensor)
                    compass_end_time = time.time()
                    final_compass_time = compass_end_time - compass_start_time

                    # Total processing time
                    total_time = image_end_time - image_start_time
                    processing_times.append(total_time)

                    # Write to CSV
                    writer.writerow([filename, final_image_time, final_lat_lon_time, final_compass_time, total_time])

        # Print out the list of times taken to process each image
        print("Processing times for each image (seconds):")
        print(f"Number of images processed: {len(processing_times)}")
        print(f"Average processing time: {sum(processing_times) / len(processing_times)} seconds")
        print(processing_times)


    # Method to extract and format bounding box data for lat/lon prediction
    def extract_bbox_data(self, predictions, threshold=0.5):
        bbox_data = []
        label_counts = {}

        # Iterate over predictions (bounding boxes, scores, and labels)
        for score, label, box in zip(predictions[0]['scores'], predictions[0]['labels'], predictions[0]['boxes']):
            if score >= threshold:
                if label not in label_counts:
                    label_counts[label] = 0
                if label_counts[label] < 2:  # Limit to 2 boxes per class
                    xmin, ymin, xmax, ymax = box.cpu().numpy()
                    width = xmax - xmin
                    height = ymax - ymin

                    # Append bounding box data (xmin, ymin, width, height)
                    bbox_data.extend([xmin, ymin, width, height])
                    label_counts[label] += 1

        # Pad bbox_data with zeros if there are fewer than 2 boxes per class
        for label in range(1, 10):  # Assuming labels 1 to 9
            if label not in label_counts:
                label_counts[label] = 0
            while label_counts[label] < 2:  # Pad with zeros if less than 2 boxes
                bbox_data.extend([0.0, 0.0, 0.0, 0.0])
                label_counts[label] += 1

        return bbox_data   


model_path = "models/best.pt"
lat_lon_path = "lat_lon_model_gps.pt"
test_image_path = "test_img.jpg"
RCNN_model_path = "models/faster_rcnn.pth"
FFNN_model_path = "models/RCNN_FFNN_simplified_shallow.pt"
test_image_path = "test_7/captured_images_test7"

RCNN_FFNN_Normal_Shallow = "models/RCNN_FFNN_normal_shallow.pt"
RCNN_FFNN_Normal_Shallow_MODEL_CSV_PATH = "new_test_inferences/RCNN+LSTMLatLon+LSTMCompass.csv"
# RCNN_LSTM_LATLON_COMPASS_CSV = "new_test_inferences/RCNN+LSTMLatLon+LSTMCompass.csv"
# RCNN_FFNN_LATLON_COMPASS_CSV = "new_test_inferences/RCNN+FFNNLatLon+FFNNCompass.csv"
YOLO_FFNN_compass_path = "models/YOLO_FFNN_compass.pt"
YOLO_FFNN_LATLON_COMPASS_CSV = "new_test_inferences/YOLO+FFNNLatLon+FFNNCompass.csv"
YOLO_LSTM_LATLON_COMPASS_CSV = "new_test_inferences/YOLO+LSTMLatLon+LSTMCompass.csv"
YOLO_FFNN_Normal_Shallow_MODEL = "models/YOLO_FFNN_normal_shallow.pt"
# YOLO_FFNN_Normal_Shallow_MODEL_CSV_PATH = "test_inferences/YOLO_FFNN_normal_shallow.csv"
# YOLO_FFNN_simplified_shallow_MODEL = "models/YOLO_FFNN_simplified_shallow.pt"
# YOLO_FFNN_simplified_shallow_MODEL_CSV_PATH = "test_inferences/YOLO_FFNN_Simplified_Shallow.csv"
YOLO_LSTM_normal_shallow_MODEL = "models/YOLO_LSTM_normal_shallow.pt"
# YOLO_LSTM_normal_shallow_MODEL_CSV_PATH = "test_inferences/YOLO_LSTM_normal_shallow.csv"
# YOLO_LSTM_simplified_shallow_MODEL = "models/YOLO_LSTM_simplified_shallow.pt"
# YOLO_LSTM_simplified_shallow_MODEL_CSV_PATH = "test_inferences/YOLO_LSTM_Simplified_Shallow.csv"
# RCNN_FFNN_Simplified_Shallow = "test_inferences/RCNN_FFNN_Simplified_Shallow.csv"
RCNN_LSTM_NORMAL_SHALLOW_MODEL_PATH = "models/RCNN_LSTM_normal_shallow.pt"
# RCNN_LSTM_NORMAL_CSV_PATH = "test_inferences/RCNN_LSTM_Normal_Shallow.csv"
# RCNN_LSTM_SIMPLIFIED_SHALLOW_PATH = "models/RCNN_LSTM_simplified_shallow.pt"
# RCNN_LSTM_SIMPLIFIED_CSV_PATH = "test_inferences/RCNN_LSTM_Simplified_shallow.csv"

# Loading the lat and lon model
# lat_lon_model = LatLonModelFFNN()
# lat_lon_model.load_state_dict(torch.load(YOLO_LSTM_normal_shallow_MODEL))

# Loading the compass model
compass_model_path = "models/RCNN_LSTM_compass.pt"
# compass_model_path_FFNN_compass = "models/RCNN_FFNN_compass.pt"
compass_model = LSTMCompassModel()
compass_model.load_state_dict(torch.load(compass_model_path))
# compass_model = CompassModelFFNN()
# compass_model.load_state_dict(torch.load(compass_model_path))
# Loading the lat and lon model for LSTM 
lat_lon_model = LatLonModelLSTM()
lat_lon_model.load_state_dict(torch.load(RCNN_LSTM_NORMAL_SHALLOW_MODEL_PATH))

# Loading the yolo model
# bbox_detector = YOLOModel(model_path, lat_lon_model, compass_model)
# bbox_detector.start_camera()
# bbox_detector.predict_record_inferences_for_images_in_folder(test_image_path, YOLO_LSTM_LATLON_COMPASS_CSV)


## For RCNN model
final_csv = "new_test_inferences/RCNN+LSTMLatLon+LSTMCompass.csv"
rcnn = BBoxDetectionRCNN(RCNN_model_path, lat_lon_model, compass_model)
rcnn.predict_record_inferences_for_images_in_folder(test_image_path, final_csv)
# rcnn.start_camera()