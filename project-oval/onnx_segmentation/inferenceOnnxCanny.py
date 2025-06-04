#!/usr/bin/env python
import cv2

import threading
import time
import numpy as np

import onnxruntime as ort
from PIL import Image as PILImage
from dataclasses import dataclass
from typing import Tuple, Optional, List
import math
import matplotlib.pyplot as plt

IMAGE_TOPIC = "/zed/zed_node/left/image_rect_color"
EDGE_DISTANCE_TOPIC = '/segmentation/edge_distance'
EDGE_DETECTED_TOPIC = '/segmentation/edge_detected'

# ONNX model path - update this to your model
MODEL_PATH = "../sidewalk_segmentation/sidewalk_segmentation_model/model.onnx"

# Camera parameters
VFOV = 68  # degrees
HFOV = 101  # degrees
HEIGHT = 0.6  # meters

frame_count = 0
start_time = None
inference_times = []

image = None
onnx_session = None

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
        # print(left_line)
        # print(right_line)
        return line_img, edges, left_line, right_line

def extend_line_coordinates(line: Tuple[int, int, int, int], 
                         original_size: Tuple[int, int],
                         new_size: Tuple[int, int]) -> Optional[Tuple[int, int, int, int]]:
    """Create a line that extends to the edges of the image, handling all possible intersections"""
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
    
    # Find all possible intersection points
    # With top edge (y = 0)
    x_top = (0 - b) / slope
    # With bottom edge (y = height)
    x_bottom = (new_size[0] - b) / slope
    # With left edge (x = 0)
    y_left = b
    # With right edge (x = width)
    y_right = slope * new_size[1] + b
    
    # Initialize points array
    points = []
    
    # Check each intersection point and add valid ones
    if 0 <= x_top <= new_size[1]:
        points.append((int(x_top), 0))
    if 0 <= x_bottom <= new_size[1]:
        points.append((int(x_bottom), new_size[0]))
    if 0 <= y_left <= new_size[0]:
        points.append((0, int(y_left)))
    if 0 <= y_right <= new_size[0]:
        points.append((new_size[1], int(y_right)))
    
    # Sort points by y-coordinate to maintain bottom-to-top order
    points.sort(key=lambda p: p[1], reverse=True)
    
    if len(points) >= 2:
        return (points[0][0], points[0][1], points[1][0], points[1][1])
    
    return None

def pixel_to_yaw(x_pixel: int, image_width: int = 1280, horizontal_fov_degrees: float = HFOV) -> float:
    """
    Calculate yaw angle to a point given its x-pixel coordinate
    
    Args:
        x_pixel: x coordinate in the image (0 is left, image_width is right)
        image_width: width of the image in pixels
        horizontal_fov_degrees: horizontal field of view of the camera in degrees
    
    Returns:
        yaw_angle: angle in radians (negative is left, positive is right)
    """
    # Convert FOV to radians
    horizontal_fov = math.radians(horizontal_fov_degrees)
    
    # Calculate the angle from the camera's center line to the pixel
    # First, find the angle for each pixel
    angle_per_pixel = horizontal_fov / image_width
    
    # Find center pixel
    center_x = image_width / 2
    
    # Calculate angle to the point (positive to the right, negative to the left)
    yaw_angle = angle_per_pixel * (x_pixel - center_x)
    
    return yaw_angle

def pixel_to_distance(y_pixel=None, image_height=720, camera_height=0.4, 
                     vertical_fov_degrees=68) -> float:
    """
    Calculate distance to a point on the ground given its y-pixel coordinate
    
    Args:
        y_pixel: y coordinate in the image (0 is top, image_height is bottom)
        image_height: height of the image in pixels
        camera_height: height of the camera from the ground in meters
        vertical_fov_degrees: vertical field of view of the camera in degrees
    
    Returns:
        distance: distance to the point in meters
    """
    # Convert FOV to radians
    vertical_fov = math.radians(vertical_fov_degrees)
    
    # Calculate the angle from the camera's horizontal plane to the pixel
    # First, find the angle for each pixel
    angle_per_pixel = vertical_fov / image_height
    
    # Find center pixel (horizon line when pitch = 0)
    center_y = image_height / 2
    
    # Calculate angle to the point (negative because pixel coordinates increase downward)
    pixel_angle = -angle_per_pixel * (y_pixel - center_y)
    
    # Now we can use basic trigonometry
    # distance = height / tan(angle)
    if pixel_angle <= 0:  # Must be below horizon
        return float('inf')
    
    distance = camera_height / math.tan(pixel_angle)
    return distance

def create_colored_mask(mask: np.ndarray) -> np.ndarray:
    """Convert segmentation mask to a colored visualization"""
    # Define colors for different classes (RGB format)
    color_map = {
        # 0: [0, 0, 0],      # Background (black)
        1: [128, 64, 128], # Road (purple-ish)
        # 2: [244, 35, 232], # Sidewalk (pink)
        # 3: [70, 70, 70],   # Building (dark gray)
        # 4: [102, 102, 156],# Wall (blue-gray)
        # Add more classes as needed
    }
    
    # Create empty RGB image
    height, width = mask.shape
    colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Fill in colors for each class
    for class_id, color in color_map.items():
        colored_mask[mask == class_id] = color
    
    return colored_mask

# Initialize lane detector globally
lane_detector = LaneDetector()

def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]  

def preprocess_image(img):
    """
    Preprocess image for ONNX model inference
    Adjust this based on your model's requirements
    """
    img_resized = cv2.resize(img, (512, 512))
    
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
    
    pil_img = PILImage.fromarray(img_rgb)
    img_array = np.array(pil_img).astype(np.float32)
    
    img_array = img_array / 255.0
    
    img_array = img_array.transpose(2, 0, 1)
    img_array = np.expand_dims(img_array, axis=0) 
    return img_array

def run_inference(img):
    """
    Run ONNX model inference and measure time
    """
    global frame_count, start_time, inference_times, onnx_session
    
    if onnx_session is None:
        print("ONNX session not initialized!")
        return None, None, None, None
    
    inference_start = time.time()
    
    processed_img = preprocess_image(img)
    
    try:
        outputs = onnx_session.run(None, {'pixel_values': processed_img})
        
        if len(outputs) > 0:
            result = outputs[0]
            if len(result.shape) > 2:
                # For segmentation models
                mask = np.argmax(result.squeeze(0), axis=0)
            else:
                mask = result
                
            # Process the mask for lane detection
            lines_image, edges, left_line, right_line = lane_detector.process_image(mask.copy())
            
        else:
            mask = None
            lines_image = None
            edges = None
            left_line = None
            right_line = None
            
    except Exception as e:
        print(f"Inference error: {e}")
        return None, None, None, None
    
    inference_end = time.time()
    inference_time = inference_end - inference_start
    inference_times.append(inference_time)
    
    frame_count += 1
    
    if start_time is None:
        start_time = time.time()
    
    if frame_count % 30 == 0:
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        avg_inference_time = np.mean(inference_times[-30:]) * 1000  # Convert to ms
        
        print(f"Frames processed: {frame_count}")
        print(f"Average FPS: {fps:.2f}")
        print(f"Average inference time: {avg_inference_time:.2f} ms")
        print(f"Frames per minute: {fps * 60:.0f}")
        
        # Print lane detection info if lines are detected
        if left_line or right_line:
            print(f"Left line: {left_line}")
            print(f"Right line: {right_line}")
            
            # Calculate distances and angles if right line is detected
            if right_line is not None:
                height, width = img.shape[:2]
                extended_right = extend_line_coordinates(right_line, mask.shape, (height, width))
                if extended_right is not None:
                    distance = pixel_to_distance(extended_right[3])
                    angle = pixel_to_yaw(width)
                    distance_to_edge = math.tan(angle) * distance
                    print(f"Distance to path edge: {distance_to_edge:.2f}m")
        
        print("-" * 40)
    
    return mask, lines_image, left_line, right_line

def visualize_result(original_img, mask, lines_image=None, left_line=None, right_line=None):
    """
    Visualize the inference result with lane detection
    """
    if mask is not None:
        height, width = original_img.shape[:2]
        mask_shape = mask.shape
        
        # Resize mask to match original image
        mask_resized = cv2.resize(mask.astype(np.uint8), 
                                (width, height), 
                                interpolation=cv2.INTER_NEAREST)
        
        # Create colored segmentation mask
        segmentation_colored = create_colored_mask(mask_resized)
        
        # Extend lines to image edges
        extended_left = extend_line_coordinates(left_line, mask_shape, (height, width)) if left_line else None
        extended_right = extend_line_coordinates(right_line, mask_shape, (height, width)) if right_line else None

        if extended_right is not None:
            print("Left: ", extended_left)
            print("Right: ", extended_right)
            distance = pixel_to_distance(extended_right[3])
            print("Distance: ", distance)
            print("X point: ", 1280)
            angle = pixel_to_yaw(1280)
            print(angle)
            distance_to_edge = math.tan(angle) * distance
            print("Distance to edge of path: ", distance_to_edge)
        
        # Calculate center line coordinates
        center_x = width // 2
        center_line = (center_x, height, center_x, 0)
        
        # Create line overlay on original image
        line_overlay = original_img.copy()
        
        if extended_left:
            cv2.line(line_overlay, 
                    (extended_left[0], extended_left[1]), 
                    (extended_left[2], extended_left[3]), 
                    (0, 255, 0), 3)
        if extended_right:
            cv2.line(line_overlay, 
                    (extended_right[0], extended_right[1]), 
                    (extended_right[2], extended_right[3]), 
                    (0, 255, 0), 3)
            
        cv2.line(line_overlay,
                (center_line[0], center_line[1]),
                (center_line[2], center_line[3]),
                (0, 0, 255), 2)
        
        # Create segmentation overlay
        segmentation_overlay = cv2.addWeighted(segmentation_colored, 0.7, original_img, 0.3, 0)
        
        # Show results
        cv2.imshow('Lane Detection Result', line_overlay)
        cv2.imshow('Segmentation Result', segmentation_overlay)
        cv2.imshow('Original', original_img)
        
        # Show edges if available
        if lines_image is not None:
            lines_resized = cv2.resize(lines_image, (width, height))
            cv2.imshow('Processed Binary', lines_resized)
        
        cv2.waitKey(1)

def extend_line_coordinates(line, original_size, new_size):
    """Create a line that extends to the edges of the image"""
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
    
    # Find all possible intersection points
    # With top edge (y = 0)
    x_top = (0 - b) / slope
    # With bottom edge (y = height)
    x_bottom = (new_size[0] - b) / slope
    # With left edge (x = 0)
    y_left = b
    # With right edge (x = width)
    y_right = slope * new_size[1] + b
    
    # Initialize points array
    points = []
    
    # Check each intersection point and add valid ones
    if 0 <= x_top <= new_size[1]:
        points.append((int(x_top), 0))
    if 0 <= x_bottom <= new_size[1]:
        points.append((int(x_bottom), new_size[0]))
    if 0 <= y_left <= new_size[0]:
        points.append((0, int(y_left)))
    if 0 <= y_right <= new_size[0]:
        points.append((new_size[1], int(y_right)))
    
    # Sort points by y-coordinate to maintain bottom-to-top order
    points.sort(key=lambda p: p[1], reverse=True)
    
    if len(points) >= 2:
        return (points[0][0], points[0][1], points[1][0], points[1][1])
    
    return None

def create_colored_mask(mask):
    """Convert segmentation mask to a colored visualization"""
    # Define colors for different classes (BGR format for OpenCV)
    color_map = {
        # 0: [0, 0, 0],      # Background (black)
        1: [128, 64, 128], # Road (purple-ish)
        # 2: [232, 35, 244], # Sidewalk (pink)
        # 3: [70, 70, 70],   # Building (dark gray)
        # 4: [156, 102, 102],# Wall (blue-gray)
    }
    
    # Create empty RGB image
    height, width = mask.shape
    colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Fill in colors for each class
    for class_id, color in color_map.items():
        colored_mask[mask == class_id] = color
    
    return colored_mask

def test_onnx_with_image_debug(image_path, model_path):
    """
    Test ONNX model with a single image and debug lane detection
    """
    # Initialize ONNX session
    onnx_session = ort.InferenceSession(model_path)
    
    # Initialize lane detector
    from inferenceOnnxCanny import LaneDetector  # Import your lane detector
    lane_detector = LaneDetector()
    
    # Load original image
    img_original = cv2.imread(image_path)
    print(f"Original image shape: {img_original.shape}")
    
    # Preprocess for model (resize to 512x512)
    img_resized = cv2.resize(img_original, (512, 512))
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
    
    pil_img = PILImage.fromarray(img_rgb)
    img_array = np.array(pil_img).astype(np.float32)
    img_array = img_array / 255.0
    img_array = img_array.transpose(2, 0, 1)
    img_array = np.expand_dims(img_array, axis=0)
    
    # Run inference
    outputs = onnx_session.run(None, {'pixel_values': img_array})
    
    # Process output
    result = outputs[0]
    if len(result.shape) > 2:
        mask = np.argmax(result.squeeze(0), axis=0)
    else:
        mask = result
    
    print(f"Mask shape: {mask.shape}")
    print(f"Mask unique values: {np.unique(mask)}")
    
    # Run lane detection on the 512x512 mask
    lines_image, edges, left_line, right_line = lane_detector.process_image(mask.copy())
    
    print("=== LANE DETECTION RESULTS ===")
    print(f"Left line: {left_line}")
    print(f"Right line: {right_line}")
    print(f"Lines image shape: {lines_image.shape}")
    print(f"Edges detected: {np.sum(edges > 0)} pixels")
    
    # Create visualizations
    original_height, original_width = img_original.shape[:2]
    mask_height, mask_width = mask.shape
    
    # Resize mask to original image size for visualization
    mask_resized = cv2.resize(mask.astype(np.uint8), 
                            (original_width, original_height), 
                            interpolation=cv2.INTER_NEAREST)
    
    # Create colored segmentation mask
    segmentation_colored = create_colored_mask(mask_resized)
    
    # Extend lines to original image size
    extended_left = extend_line_coordinates(left_line, (mask_height, mask_width), (original_height, original_width)) if left_line else None
    extended_right = extend_line_coordinates(right_line, (mask_height, mask_width), (original_height, original_width)) if right_line else None
    
    print(f"Extended left line: {extended_left}")
    print(f"Extended right line: {extended_right}")
    print(lines_image.shape)

    # Create line overlay on original image
    line_overlay = img_original.copy()
    
    # Calculate center line
    center_x = original_width // 2
    center_line = (center_x, original_height, center_x, 0)
    
    # Draw lines on original image
    if extended_left:
        cv2.line(line_overlay, 
                (extended_left[0], extended_left[1]), 
                (extended_left[2], extended_left[3]), 
                (0, 255, 0), 3)  # Green for left line
        print(f"Drew left line: ({extended_left[0]}, {extended_left[1]}) -> ({extended_left[2]}, {extended_left[3]})")
    
    if extended_right:
        cv2.line(line_overlay, 
                (extended_right[0], extended_right[1]), 
                (extended_right[2], extended_right[3]), 
                (0, 255, 0), 3)  # Green for right line
        print(f"Drew right line: ({extended_right[0]}, {extended_right[1]}) -> ({extended_right[2]}, {extended_right[3]})")
    
    # Draw center line
    cv2.line(line_overlay,
            (center_line[0], center_line[1]),
            (center_line[2], center_line[3]),
            (0, 0, 255), 2)  # Red for center line
    
    # Create segmentation overlay
    segmentation_overlay = cv2.addWeighted(segmentation_colored, 0.7, img_original, 0.3, 0)
    
    # Resize processed images for display
    lines_resized = cv2.resize(lines_image, (original_width, original_height))
    edges_resized = cv2.resize(edges, (original_width, original_height))
    
    # Display results using matplotlib for better control
    plt.figure(figsize=(20, 12))
    
    plt.subplot(2, 3, 1)
    plt.imshow(cv2.cvtColor(img_original, cv2.COLOR_BGR2RGB))
    plt.title('Original Image')
    plt.axis('off')
    
    plt.subplot(2, 3, 2)
    plt.imshow(mask_resized, cmap='tab10')
    plt.title(f'Segmentation Mask\nUnique values: {np.unique(mask)}')
    plt.axis('off')
    
    plt.subplot(2, 3, 3)
    plt.imshow(cv2.cvtColor(segmentation_overlay, cv2.COLOR_BGR2RGB))
    plt.title('Segmentation Overlay')
    plt.axis('off')
    
    plt.subplot(2, 3, 4)
    plt.imshow(cv2.cvtColor(lines_resized, cv2.COLOR_BGR2RGB))
    plt.title('Processed Binary')
    plt.axis('off')
    
    plt.subplot(2, 3, 5)
    plt.imshow(edges_resized, cmap='gray')
    plt.title(f'Canny Edges\n{np.sum(edges > 0)} pixels')
    plt.axis('off')
    
    plt.subplot(2, 3, 6)
    plt.imshow(cv2.cvtColor(line_overlay, cv2.COLOR_BGR2RGB))
    plt.title(f'Lane Detection Result\nLeft: {left_line is not None}, Right: {right_line is not None}')
    plt.axis('off')
    
    plt.tight_layout()
    plt.show()
    
    # Also show with OpenCV for interaction
    cv2.imshow('Original', img_original)
    cv2.imshow('Lane Detection Result', line_overlay)
    cv2.imshow('Segmentation Overlay', segmentation_overlay)
    cv2.imshow('Processed Binary', lines_resized)
    cv2.imshow('Canny Edges', edges_resized)
    
    print("Press any key to close windows...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return mask, lines_image, left_line, right_line


def main():
    global onnx_session
    
    rclpy.init()
    node = rclpy.create_node('onnx_inference_node')
    
    try:
        
        available_providers = ort.get_available_providers()
        providers = []
        if 'CUDAExecutionProvider' in available_providers:
            providers.append(('CUDAExecutionProvider', {
                'device_id': 0,
                'arena_extend_strategy': 'kNextPowerOfTwo',
                'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # 2GB limit
                'cudnn_conv_algo_search': 'EXHAUSTIVE',
                'do_copy_in_default_stream': True,
            }))
            print("CUDA provider configured")
        else:
            print("WARNING: CUDA provider not available!")
        
        providers.append('CPUExecutionProvider')
        
        onnx_session = ort.InferenceSession(MODEL_PATH, providers=providers)
        
        session_providers = onnx_session.get_providers()
    
        
        input_names = [input.name for input in onnx_session.get_inputs()]
        output_names = [output.name for output in onnx_session.get_outputs()]
        print(f"Model inputs: {input_names}")
        print(f"Model outputs: {output_names}")
        
    except Exception as e:
        print(f"Failed to load ONNX model: {e}")
        return
    
    # Create subscription to camera topic
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
    # Publishers for performance metrics
    distance_pub = node.create_publisher(Float64, EDGE_DISTANCE_TOPIC, 10)
    detected_pub = node.create_publisher(Bool, EDGE_DETECTED_TOPIC, 10)
    
    # Start ROS2 spinning in separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    FREQ = 30 
    rate = node.create_rate(FREQ, node.get_clock())
    
    while rclpy.ok() and image is None:
        print("Waiting for image topic...")
        rate.sleep()
    
    print("Starting ONNX inference with lane detection...")
    print("Press Ctrl+C to stop and see final results")
    
    try:
        while rclpy.ok():
            if image is not None:
                mask, lines_image, left_line, right_line = run_inference(image)
                
                visualize_result(image, mask, lines_image, left_line, right_line)

                if mask is not None:
                    height, width, _ = image.shape
                    mask_shape = mask.shape
                    
                    # Resize mask to match original image
                    mask_resized = cv2.resize(mask.astype(np.uint8), 
                                            (width, height), 
                                            interpolation=cv2.INTER_NEAREST)
                    
                    # Create colored segmentation mask
                    segmentation_colored = create_colored_mask(mask_resized)
                    
                    # Extend lines to image edges
                    extended_left = extend_line_coordinates(left_line, mask_shape, (height, width)) if left_line else None
                    extended_right = extend_line_coordinates(right_line, mask_shape, (height, width)) if right_line else None
                    
                    if extended_right is not None:
                        print("Left: ", extended_left)
                        print("Right: ", extended_right)
                        distance = pixel_to_distance(extended_right[3])
                        print("Distance: ", distance)
                        print("X point: ", 1280)
                        angle = pixel_to_yaw(1280)
                        print(angle)
                        distance_to_edge = math.tan(angle) * distance
                        print("Distance to edge of path: ", distance_to_edge)
                
                msg = Float64()
                msg.data = distance_to_edge
                distance_pub.publish(msg)

                msg = Bool()
                msg.data = extended_right != None
                detected_pub.publish(msg)

            rate.sleep()
            
    except KeyboardInterrupt:
        print("\nStopping inference test...")
    
    if start_time is not None and frame_count > 0:
        total_time = time.time() - start_time
        final_fps = frame_count / total_time
        frames_per_minute = final_fps * 60
        avg_inference_time = np.mean(inference_times) * 1000
        
        print("\n" + "="*50)
        print("FINAL PERFORMANCE REPORT")
        print("="*50)
        print(f"Total frames processed: {frame_count}")
        print(f"Total time: {total_time:.2f} seconds")
        print(f"Average FPS: {final_fps:.2f}")
        print(f"Frames per minute: {frames_per_minute:.0f}")
        print(f"Average inference time: {avg_inference_time:.2f} ms")
        print(f"Min inference time: {np.min(inference_times)*1000:.2f} ms")
        print(f"Max inference time: {np.max(inference_times)*1000:.2f} ms")
        print("="*50)
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from std_msgs.msg import Int64, Float32
    br = CvBridge()
    main()

