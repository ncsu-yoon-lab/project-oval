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
from torchvision import models
from torch.utils.data import random_split
from cocodataset import Compose, ToTensor, CocoDataset
import sys

def train_one_epoch(model, optimizer, loader, device, epoch, lr_scheduler):
    model.to(device)
    model.train()
    
    all_losses = []
    all_losses_dict = []
    
    for images, targets in tqdm(loader):
        # Skip batches with None
        if images is None or targets is None or any(img is None for img in images) or any(tar is None for tar in targets):
            continue

        images = list(image.to(device) for image in images)
        targets = [{k: torch.tensor(v).to(device) for k, v in t.items()} for t in targets]
        
        loss_dict = model(images, targets) # the model computes the loss automatically if we pass in targets
        losses = sum(loss for loss in loss_dict.values())
        loss_dict_append = {k: v.item() for k, v in loss_dict.items()}
        loss_value = losses.item()
        
        all_losses.append(loss_value)
        all_losses_dict.append(loss_dict_append)
        
        if not math.isfinite(loss_value):
            print(f"Loss is {loss_value}, stopping training") # train if loss becomes infinity
            print(loss_dict)
            sys.exit(1)
        
        optimizer.zero_grad()
        losses.backward()
        optimizer.step()
        
        if lr_scheduler is not None:
            lr_scheduler.step() # 
        
    all_losses_dict = pd.DataFrame(all_losses_dict) # for printing
    print("Epoch {}, lr: {:.6f}, loss: {:.6f}, loss_classifier: {:.6f}, loss_box: {:.6f}, loss_rpn_box: {:.6f}, loss_object: {:.6f}".format(
        epoch, optimizer.param_groups[0]['lr'], np.mean(all_losses),
        all_losses_dict['loss_classifier'].mean(),
        all_losses_dict['loss_box_reg'].mean(),
        all_losses_dict['loss_rpn_box_reg'].mean(),
        all_losses_dict['loss_objectness'].mean()
    ))

def evaluate(val_loader, model, device, label_threshold = 0.8, iou_threshold = 0.5):
    # Make sure it's in eval mode
    device1 = torch.device('cpu')
    model.to(device)
    model.eval()

    # Lists to store detected and true boxes, labels, scores
    det_boxes = list()
    det_labels = list()
    det_scores = list()
    true_boxes = list()
    true_labels = list()
    true_difficulties = list()  # it is necessary to know which objects are 'difficult', see 'calculate_mAP' in utils.py
    
    with torch.no_grad():
        for images, targets in tqdm(val_loader):
            if images is None or targets is None or any(img is None for img in images) or any(tar is None for tar in targets):
                continue

            images = list(image.to(device) for image in images)
            predictions = model(images)
            for i in range(len(predictions)):
                pred = predictions[i]
                det_boxes.append(pred['boxes'][pred['scores']>label_threshold].to(device1))
                det_labels.append(pred['labels'][pred['scores']>label_threshold].to(device1))
                det_scores.append(torch.tensor([sc for sc in pred['scores'].tolist() if sc > label_threshold], dtype=torch.float32).to(device1))
                true_boxes.append(targets[i]['boxes'].to(device1))
                true_labels.append(targets[i]['labels'].to(device1))
                true_difficulties.append(torch.zeros(len(targets[i]['labels'])).to(device1))   
        APs, mAP = calculate_mAP(det_boxes, det_labels, det_scores, true_boxes, true_labels, true_difficulties, device1, iou_threshold)
    return APs, mAP

def get_model():
    n_classes = 2

    # lets load the faster rcnn model
    model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
    in_features = model.roi_heads.box_predictor.cls_score.in_features # we need to change the head
    model.roi_heads.box_predictor = models.detection.faster_rcnn.FastRCNNPredictor(in_features, n_classes)
    return model

def collate_fn(batch):
    return tuple(zip(*batch))

def get_data_loader(img_folder, ann_file, train_flag=True):
    transforms = Compose([ToTensor()])
    dataset = CocoDataset(img_folder, ann_file, transforms=transforms)
    batch_size = 4  # batch size
    workers = 4
    if train_flag == False:
        workers = 1 

    data_loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=train_flag, num_workers=workers,
                                                pin_memory=True,collate_fn=collate_fn)

    return data_loader

def main(train_img_folder, train_ann_file, val_img_folder, val_ann_file):
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    n_gpu = torch.cuda.device_count()

    model = get_model()
    train_loader = get_data_loader(train_img_folder, train_ann_file, train_flag=True)
    val_loader = get_data_loader(val_img_folder, val_ann_file, train_flag=False)

    print("Number of GPUs: "+str(n_gpu))

    train_losses = []
    n_epochs = 3
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    scheduler = None
    loss_list = []

    for epoch in range(n_epochs+1):
        train_one_epoch(model, optimizer, train_loader, device, epoch, scheduler)
        print(evaluate(val_loader, model, device))
        model.eval()
        torch.save(model.state_dict(), 'model_epoch' + str(epoch) + '.pth')

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python script_name.py <train_img_folder> <train_ann_file> <val_img_folder> <val_ann_file>")
    else:
        main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

# python train.py C:/Users/yujim/Documents/Project_Oval/project-oval/ML/DeepLearning/ImageAnnotation/building_structures.v1i.coco/train/images C:/Users/yujim/Documents/Project_Oval/project-oval/ML/DeepLearning/ImageAnnotation/building_structures.v1i.coco/train/_annotations.coco.json C:/Users/yujim/Documents/Project_Oval/project-oval/ML/DeepLearning/ImageAnnotation/building_structures.v1i.coco/valid/images C:/Users/yujim/Documents/Project_Oval/project-oval/ML/DeepLearning/ImageAnnotation/building_structures.v1i.coco/valid/_annotations.coco.json

