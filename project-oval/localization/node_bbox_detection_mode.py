import cv2 
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.backbone_utils import resnet_fpn_backbone
import torchvision.transforms as transforms
from torch.utils.data import DataLoader, Dataset
from PIL import Image
import numpy as np
import time

class BBoxDetectionRCNN:
    def __init__(self, model_path):
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

    def predict_from_frame(self, frame):
        """
        Function that first creates a pipeline to convert normal nparry -> tensor array. 
        Then makes prediction using the model 
        """
        
        # Need to first transform the frame to tensor data structure 
        transform = transforms.Compose([transforms.ToTensor()]) # This line creates a pipeline to convert to tensor data structure
        frame_tensor = transform(frame).unsqueeze(0) # Adds extra dimensional, since the model expects a batch of images shape [1, C, H, W]
        
        # Make predictions
        with torch.no_grad():
            predictions = self.model(frame_tensor)

        return predictions 
    
    def start_camera(self):
        
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error opening the camera")
            return
        
        while True:
            start = time.time()  # Start time
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to grab frame")
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(frame_rgb)
            predictions = self.predict_from_frame(pil_image)
            
            for i, box in enumerate(predictions[0]['boxes']):
                score = predictions[0]['scores'][i]
                if score > 0.5:
                    x1, y1, x2, y2 = box.int().tolist()
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"Class {predictions[0]['labels'][i].item()} | {score:.2f}"
                    cv2.putText(frame, label, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            end = time.time()  # End time
            inference_time = end - start  # Time taken for one loop iteration
            print(f"Inference Time: {inference_time:.4f} seconds")  # Print inference time
            
            cv2.imshow('Object Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 
            
        cap.release()
        cv2.destroyAllWindows()
            
model_path = "model_resnet18_epoch_6.pth"
bbox_detector = BBoxDetectionRCNN(model_path)
bbox_detector.load_RCNN_model()
bbox_detector.start_camera()