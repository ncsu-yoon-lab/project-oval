import torch
from torch.utils.data import Dataset
import glob
import os
from PIL import Image, ImageDraw
import cv2
from torchvision import transforms
from tqdm import tqdm
from torchvision.utils import draw_bounding_boxes
import math
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from eval_helper import calculate_mAP

labels_dict = {'UNKNOWN': 0, 'VEHICLE': 1, 'PEDESTRIAN': 2, 'SIGN': 3, 'CYCLIST': 4}
labels_dict = { 0 : 'UNKNOWN', 1 : 'VEHICLE', 2 : 'PEDESTRIAN', 3 : 'SIGN', 4:'CYCLIST'}

class WaymoDataset(torch.utils.data.Dataset):
    def __init__(self, root, transforms=None):
        self.root = root
        self.transforms = transforms
        # load all image files, sorting them to
        # ensure that they are aligned
        self.imgs = list(sorted(os.listdir(self.root)))
        self.labels = list(sorted(os.listdir(self.root + '_labels')))

    def __getitem__(self, idx):
        # load images and labels
        tmp = self.root + '_labels'
        img_path = os.path.join(self.root, self.imgs[idx])
        label_path = os.path.join(tmp, self.labels[idx])
        img = Image.open(img_path).convert("RGB")

        # get bounding box coordinates for each mask
        boxes = []
        labels = []
        with open(label_path) as f:
            lines = f.readlines()

            for line in lines:
                data = [item.strip() for item in line.split(' ')]
                if (data[5] == "0"):
                    labels.append(int(data[0]))
                    x1 = float(data[1])
                    y1 = float(data[2])
                    x2 = float(data[3])
                    y2 = float(data[4])
                    boxes.append([x1, y1, x2, y2])

        # convert everything into a torch.Tensor
        if len(boxes) == 0:
            return self.__getitem__(np.random.randint(0,10))

        image_id = torch.tensor([idx])
        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = np.asarray(labels)
        labels = torch.from_numpy(labels.astype('long'))

        target = {}
        target["boxes"] = boxes
        target["labels"] = labels
        target["image_id"] = image_id
        
        image = torch.from_numpy(np.array(img).copy())
        image = image.permute(2, 0, 1)
#         if self.transforms is not None:
#             img = self.transforms(img)
        
        return image.div(255), target

    def __len__(self):
        return len(self.imgs)