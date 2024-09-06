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


