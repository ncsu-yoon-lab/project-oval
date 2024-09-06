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
    def __init__(self, model_path, LatLonModelObject):
        """
        model_path: Path to the model
        latlonmodelObject: Model object 
        """
        self.model = self.load_model(LatLonModelObject, model_path)

    def load_model(self, model_object, model_path):
        return model_object.load_state_dict(torch.load(model_path, weights_only=True))

    