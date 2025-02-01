import torch
import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import torchvision.transforms as transforms
from safetensors.torch import load_file
from transformers import AutoModelForSemanticSegmentation
from dataclasses import dataclass
from typing import Tuple, Optional, List

@dataclass
class LaneDetectionConfig:
    """Configuration parameters for lane detection"""
    # PID coefficients
    kp: float = 0.3
    kd: float = 0.05
    ki: float = 0.0
    
    # Image processing parameters
    gaussian_kernel: Tuple[int, int] = (5, 5)
    canny_low: int = 30
    canny_high: int = 100
    
    # Hough transform parameters
    hough_rho: int = 1
    hough_theta: float = np.pi/180
    hough_threshold: int = 15
    hough_min_line_length: int = 20
    hough_max_line_gap: int = 5
    
    # Lane detection parameters
    min_slope: float = 0.1

class SegmentationModel:
    """Handles image segmentation using a pre-trained model"""
    def __init__(self, model_path: str):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model_state = load_file(model_path)
        self.model = AutoModelForSemanticSegmentation.from_pretrained("./sidewalk_segmentation_model")
        self.model.load_state_dict(self.model_state)
        self.model.to(self.device)
        self.model.eval()
        
        self.transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])

    def preprocess_image(self, image_path: str) -> Tuple[torch.Tensor, Image.Image]:
        """Preprocess image for model input"""
        image = Image.open(image_path).convert('RGB')
        input_tensor = self.transform(image)
        input_batch = input_tensor.unsqueeze(0).to(self.device)
        return input_batch, image

    @torch.no_grad()
    def segment_image(self, image_path: str) -> np.ndarray:
        """Perform segmentation on input image"""
        input_batch, _ = self.preprocess_image(image_path)
        
        pixel_values = input_batch if not isinstance(input_batch, dict) else input_batch["pixel_values"]
        output = self.model(pixel_values=pixel_values)
        
        segmentation_mask = output.logits.squeeze(0).cpu().numpy()
        if len(segmentation_mask.shape) > 2:
            segmentation_mask = np.argmax(segmentation_mask, axis=0)
            
        return segmentation_mask

import torch
import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import torchvision.transforms as transforms
from safetensors.torch import load_file
from transformers import AutoModelForSemanticSegmentation
from dataclasses import dataclass
from typing import Tuple, Optional, List

# ... [Previous config and SegmentationModel class remain the same] ...

class LaneDetector:
    """Processes segmentation mask to detect lanes and calculate steering"""
    def __init__(self, config: LaneDetectionConfig = LaneDetectionConfig()):
        self.config = config

    def preprocess_mask(self, mask: np.ndarray) -> np.ndarray:
        """Convert segmentation mask to binary image for lane detection"""
        mask[mask == 0] = 1
        processed = np.zeros_like(mask, dtype=np.uint8)
        
        # Mark road classes
        road_classes = [1, 3, 4]
        for class_id in road_classes:
            processed[mask == class_id] = 255
            
        return processed

    def detect_edges(self, img: np.ndarray) -> np.ndarray:
        """Apply edge detection to binary image"""
        blurred = cv2.GaussianBlur(img, self.config.gaussian_kernel, 0)
        edges = cv2.Canny(blurred, self.config.canny_low, self.config.canny_high)
        kernel = np.ones((3,3), np.uint8)
        return cv2.dilate(edges, kernel, iterations=1)

    def detect_lines(self, edges: np.ndarray) -> Tuple[List, List, List, List]:
        """Detect and categorize lines using Hough transform"""
        lines = cv2.HoughLinesP(
            edges,
            rho=self.config.hough_rho,
            theta=self.config.hough_theta,
            threshold=self.config.hough_threshold,
            minLineLength=self.config.hough_min_line_length,
            maxLineGap=self.config.hough_max_line_gap
        )
        
        if lines is None:
            return [], [], [], []
            
        left_x, left_y = [], []
        right_x, right_y = [], []
        
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1)/(x2 - x1) if x2 != x1 else float('inf')
                if abs(slope) >= self.config.min_slope:
                    if slope > 0:
                        left_x.extend([x1, x2])
                        left_y.extend([y1, y2])
                    else:
                        right_x.extend([x1, x2])
                        right_y.extend([y1, y2])
                        
        return left_x, left_y, right_x, right_y

    def get_average_lines(self, img_shape: Tuple[int, int], 
                         left_x: List[int], left_y: List[int],
                         right_x: List[int], right_y: List[int]) -> Tuple[Optional[Tuple[int, int, int, int]], 
                                                                         Optional[Tuple[int, int, int, int]]]:
        """Calculate average left and right lines"""
        left_line = right_line = None
        
        if left_x and left_y:
            left_fit = np.polyfit(left_y, left_x, deg=1)
            left_slope = left_fit[0]
            left_intercept = left_fit[1]
            
            # Calculate line points
            y1 = img_shape[0]  # Bottom of image
            y2 = int(y1 * 0.6)  # Adjust this value to change line length
            x1 = int(left_slope * y1 + left_intercept)
            x2 = int(left_slope * y2 + left_intercept)
            left_line = (x1, y1, x2, y2)
            
        if right_x and right_y:
            right_fit = np.polyfit(right_y, right_x, deg=1)
            right_slope = right_fit[0]
            right_intercept = right_fit[1]
            
            y1 = img_shape[0]
            y2 = int(y1 * 0.6)
            x1 = int(right_slope * y1 + right_intercept)
            x2 = int(right_slope * y2 + right_intercept)
            right_line = (x1, y1, x2, y2)
            
        return left_line, right_line

    def process_image(self, mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Optional[Tuple[int, int, int, int]], Optional[Tuple[int, int, int, int]]]:
        """Main processing pipeline for lane detection"""
        processed = self.preprocess_mask(mask)
        edges = self.detect_edges(processed)
        left_x, left_y, right_x, right_y = self.detect_lines(edges)
        
        # Create a colored image for visualization
        line_img = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)
        
        # Get and draw average lines
        left_line, right_line = self.get_average_lines(processed.shape, left_x, left_y, right_x, right_y)
        
        return line_img, edges, left_line, right_line

def extend_line_coordinates(line: Tuple[int, int, int, int], 
                         original_size: Tuple[int, int],
                         new_size: Tuple[int, int]) -> Optional[Tuple[int, int, int, int]]:
    """Create a line that extends to the top and bottom of the image"""
    if line is None:
        return None
    
    x1, y1, x2, y2 = line
    
    # First normalize the coordinates to be between 0 and 1
    x1_norm = x1 / original_size[1]
    y1_norm = y1 / original_size[0]
    x2_norm = x2 / original_size[1]
    y2_norm = y2 / original_size[0]
    
    # Scale to new image size
    x1_scaled = x1_norm * new_size[1]
    y1_scaled = y1_norm * new_size[0]
    x2_scaled = x2_norm * new_size[1]
    y2_scaled = y2_norm * new_size[0]
    
    # Calculate slope and intercept
    if x2_scaled - x1_scaled == 0:  # Vertical line
        return (int(x1_scaled), new_size[0], int(x1_scaled), 0)
    
    slope = (y2_scaled - y1_scaled) / (x2_scaled - x1_scaled)
    b = y1_scaled - slope * x1_scaled
    
    # Find x coordinates at y = 0 and y = height
    x_top = int((0 - b) / slope)
    x_bottom = int((new_size[0] - b) / slope)
    
    return (x_bottom, new_size[0], x_top, 0)
    

def find_line_intersection(line1, line2):
    """Find intersection point of two lines given in the form (x1, y1, x2, y2)"""
    if line1 is None or line2 is None:
        return None
        
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    
    # Calculate denominator
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:  # Lines are parallel
        return None
        
    # Calculate intersection point
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    
    # Get intersection coordinates
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    
    return (int(x), int(y))

def find_line_intersection(line1, line2):
    """Find intersection point of two lines given in the form (x1, y1, x2, y2)"""
    if line1 is None or line2 is None:
        return None
        
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    
    # Calculate denominator
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:  # Lines are parallel
        return None
        
    # Calculate intersection point
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    
    # Get intersection coordinates
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)
    
    return (int(x), int(y))

def create_colored_mask(mask: np.ndarray) -> np.ndarray:
    """Convert segmentation mask to a colored visualization"""
    # Define colors for different classes (RGB format)
    color_map = {
        0: [0, 0, 0],      # Background (black)
        1: [128, 64, 128], # Road (purple-ish)
        2: [244, 35, 232], # Sidewalk (pink)
        3: [70, 70, 70],   # Building (dark gray)
        4: [102, 102, 156],# Wall (blue-gray)
        # Add more classes as needed
    }
    
    # Create empty RGB image
    height, width = mask.shape
    colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Fill in colors for each class
    for class_id, color in color_map.items():
        colored_mask[mask == class_id] = color
    
    return colored_mask

def visualize_results(original_image: Image.Image, 
                     segmentation_mask: np.ndarray,
                     lines_image: Optional[np.ndarray] = None,
                     left_line=None,
                     right_line=None):
    """Visualize processing steps with center line and intersection point"""
    # Create figure with larger size and adjusted spacing
    plt.figure(figsize=(20, 8))
    plt.subplots_adjust(wspace=0.3)  # Adjust spacing between subplots
    
    # Convert original image to numpy array and get its size
    original_np = np.array(original_image)  # This will be in RGB already
    height, width = original_np.shape[:2]
    mask_shape = segmentation_mask.shape
    
    # Extend lines to image edges
    extended_left = extend_line_coordinates(left_line, mask_shape, (height, width)) if left_line else None
    extended_right = extend_line_coordinates(right_line, mask_shape, (height, width)) if right_line else None
    
    # Calculate center line coordinates
    center_x = width // 2
    center_line = (center_x, height, center_x, 0)  # Vertical line from bottom to top
    
    # Find intersection point of extended lines
    intersection_point = find_line_intersection(extended_left, extended_right) if extended_left and extended_right else None
    
    # Create line overlays for each image type
    def create_line_overlay(base_image, is_bgr=False):
        if is_bgr:
            base_image = cv2.cvtColor(base_image, cv2.COLOR_BGR2RGB)
            
        line_img = np.zeros_like(base_image)
        
        # Draw detected lines in green (RGB format)
        if extended_left:
            cv2.line(line_img, 
                    (extended_left[0], extended_left[1]), 
                    (extended_left[2], extended_left[3]), 
                    (0, 255, 0), 3)
        if extended_right:
            cv2.line(line_img, 
                    (extended_right[0], extended_right[1]), 
                    (extended_right[2], extended_right[3]), 
                    (0, 255, 0), 3)
            
        # Draw center line in red (RGB format)
        cv2.line(line_img,
                (center_line[0], center_line[1]),
                (center_line[2], center_line[3]),
                (255, 0, 0), 2)
        
        # Draw intersection line in yellow (RGB format)
        if intersection_point:
            cv2.line(line_img,
                    (intersection_point[0], height),
                    (intersection_point[0], 0),
                    (255, 255, 0), 2)
            
        return cv2.addWeighted(base_image, 1.0, line_img, 1.0, 0.0)
    
    # Process images
    original_with_lines = create_line_overlay(original_np, is_bgr=False)
    
    if lines_image is not None:
        lines_image_resized = cv2.resize(lines_image, (width, height))
        processed_with_lines = create_line_overlay(lines_image_resized, is_bgr=True)
    else:
        processed_with_lines = None
    
    # Display images in a 1x2 grid
    plt.subplot(1, 2, 1)
    plt.imshow(original_with_lines)  # Original is already in RGB
    plt.title('Original Image with Lines', pad=20, fontsize=14)
    plt.axis('off')
    
    if processed_with_lines is not None:
        plt.subplot(1, 2, 2)
        plt.imshow(processed_with_lines)  # Already converted to RGB in create_line_overlay
        plt.title('Processed Image with Lines', pad=20, fontsize=14)
        plt.axis('off')
    
    # Ensure layout is tight and centered
    plt.tight_layout()
    plt.show()

def main():
    model_path = "./sidewalk_segmentation_model/model.safetensors"
    test_image_path = "test_image.jpg"
    
    # Initialize components
    segmentation_model = SegmentationModel(model_path)
    lane_detector = LaneDetector()
    
    # Process image
    mask = segmentation_model.segment_image(test_image_path)
    processed, edges, left_line, right_line = lane_detector.process_image(mask)
    
    # Debug prints
    print("Lines:")
    print(f"Left line: {left_line}")
    print(f"Right line: {right_line}")
    print(f"Mask shape: {mask.shape}")
    
    # Visualize results
    _, original_image = segmentation_model.preprocess_image(test_image_path)
    visualize_results(original_image, mask, processed, left_line, right_line)

if __name__ == "__main__":
    main()