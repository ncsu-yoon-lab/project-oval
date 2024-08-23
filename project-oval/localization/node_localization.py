# Imports
import json
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os
import torch.nn.functional as F

class COCOGeolocationDataset(Dataset):
    """Class that loads the data and creates a dataset which is used for Training and Testing

    Args:
        coco_file: Path to the cocoDataSet annotations
        images_dir: Path to the directory that contains all the images
    """
    def __init__(self, coco_file, images_dir):
        self.images_dir = images_dir
        self.coco_annotations = self.load_coco_annotations(coco_file)
        self.bounding_boxes, self.geolocations, self.image_files = self.process_data(self.coco_annotations)

    def load_coco_annotations(self, file_path):
        with open(file_path, 'r') as f:
            coco_data = json.load(f)
        return coco_data

    def process_data(self, coco_data):
        max_instances_per_class = 2
        num_classes = 9
        data_points = 4
        lat_lon = 2
        input_size = num_classes * max_instances_per_class * data_points
        bounding_boxes = []
        geolocations = []
        image_files = []

        for image_info in coco_data['images']:
            image_id = image_info['id']
            image_lat = float(image_info['cheap_latitude'])
            image_lon = float(image_info['cheap_longitude'])
            cheap_location = [image_lat, image_lon]
            
            expensive_latitude = float(image_info['expensive_latitude']) % 0.1 * 1000.0
            expensive_longitude = float(image_info['expensive_longitude']) % 0.1 * 1000.0
            geolocation = [expensive_latitude, expensive_longitude]
            
            input_vector = [0] * input_size
            annotations = [ann for ann in coco_data['annotations'] if ann['image_id'] == image_id]
            for ann in annotations:
                class_id = ann['category_id']
                bbox = ann['bbox']
                instance_index = sum(1 for a in annotations if a['category_id'] == class_id) - 1
                if instance_index < max_instances_per_class:
                    start_index = (class_id * max_instances_per_class + instance_index) * 4
                    length = 1280.0
                    width = 720.0
                    bbox[0] = bbox[0] / length
                    bbox[1] = bbox[1] / width
                    bbox[2] = bbox[2] / length
                    bbox[3] = bbox[3] / width
                    input_vector[start_index:start_index + 4] = bbox

            bounding_boxes.append(input_vector)
            geolocations.append(geolocation)
            image_files.append(image_info['file_name'])

        return bounding_boxes, geolocations, image_files

    def __len__(self):
        return len(self.bounding_boxes)

    def __getitem__(self, idx):
        bounding_box = torch.tensor(self.bounding_boxes[idx], dtype=torch.float32)
        geolocation = torch.tensor(self.geolocations[idx], dtype=torch.float32)
        image_file = self.image_files[idx]
        image_path = os.path.join(self.images_dir, image_file)
        return bounding_box, geolocation, image_path

class LatLonModel(nn.Module):
    """
        LatLonModel class the contains the architecture of the LSTM model. 
        Here, we used 3 hidden layers where 1st layer has 256 node, 2nd layer has 128 nodes and 3rd layer has 128 nodes
        The input size is 72 -> [9 * 2 * 4] -> 9 class identification, only allowed 2 per class, 4 representing the bounding box [x, y, w, h]
    Args:
        
    """
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

