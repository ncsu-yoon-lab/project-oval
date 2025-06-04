import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import glob
from pathlib import Path
import math


MIN_POINT_DISTANCE = 200

class LaneDetector():

    def __init__(self):
        self.first_time = True
        self.previous_point1 = None
        self.previous_point2 = None
        self.previous_point3 = None
        self.previous_point4 = None
        self.previous_percent = 0.2
        self.current_percent = 1.0 - self.previous_percent

    def _crop_image(self, image: np.ndarray) -> np.ndarray:
        num_rows, num_columns, _ = image.shape

        cropped_top = int(num_rows / 2)
        
        cropped_image = image[cropped_top:, 0:num_columns]

        num_rows, num_columns, _ = cropped_image.shape

        left_point = (0, num_rows - 1)
        right_point = (num_columns - 1, num_rows - 1)

        # Move the points up until they reach the first black pixel
        while(any(cropped_image[int(left_point[1]), int(left_point[0])]) and any(cropped_image[int(right_point[1]), int(right_point[0])]) and (right_point[1] > 0 and left_point[1] > 0)):
            left_point = (left_point[0], left_point[1] - 1)
            right_point = (right_point[0], right_point[1] - 1)
        
        cropped_bottom = num_rows - int(left_point[1])
        # Crops the image to show from the top of the image to the y value of the left point/right point
        cropped_image = cropped_image[0:int(left_point[1]), 0:num_columns]

        return cropped_image, cropped_top, cropped_bottom
    
    def _find_main_path(self, image: np.ndarray) -> tuple[float, float]:

        num_rows, num_columns, _ = image.shape

        boxes = 5

        box_width = num_columns / boxes

        largest_sum = 0
        largest_box_idx = -1

        for i in range(boxes):
            box_start_x = int(i * box_width)
            box_end_x = int((i + 1) * box_width)

            box = image[:, box_start_x:box_end_x]

            sum_of_white_pixels = np.sum(box == 255)

            if sum_of_white_pixels > largest_sum:
                largest_sum = sum_of_white_pixels
                largest_box_idx = i

        center_point = (largest_box_idx * box_width + box_width / 2, num_rows - 1)

        return center_point
    
    def _find_edges(self, image: np.ndarray, path_center: tuple[float, float]) -> tuple[tuple[float, float], tuple[float, float]]:
        # Create a copy for visualization
        image_with_edges = image.copy()
        
        # Point 1 goes to the left, point 2 goes up, point 3 goes to the right, point 4 goes up
        point1 = point2 = point3 = point4 = path_center

        point1_on_path = point2_on_path = point3_on_path = point4_on_path = True

        direction = "left"

        # Move point1 to the left until it is not on the path
        while point1_on_path:

            # Checks the movement of the point based on the current direction
            if direction == "left":
                point1 = (point1[0] - 1, point1[1])
            elif direction == "up":
                point1 = (point1[0], point1[1] - 1)

            # Checks if the point reached the left edge of the image and changes direction to up
            if point1[0] == 0 and direction == "left":
                direction = "up"
            # Checks if it reached black and changes direction to up
            elif not any(image[int(point1[1]), int(point1[0])]) and direction == "left":
                point1 = (point1[0] + 1, point1[1])
                direction = "up"

            
            
            # Checks to see if point1 is still within the image bounds and prevents the point from exceeding the image
            if point1[0] < 0:
                point1 = (point1[0] + 1, point1[1])
                point1_on_path = False
            elif point1[1] < 0:
                point1 = (point1[0], point1[1] + 1)
                point1_on_path = False
            elif point1[0] > image.shape[1] - 1:
                point1 = (image.shape[1] - 1, point1[1])
                point1_on_path = False
            elif point1[1] > image.shape[0] - 1:
                point1 = (point1[0], image.shape[0] - 1)
                point1_on_path = False
            elif not any(image[int(point1[1]), int(point1[0])]) and direction == "up":
                point1 = (point1[0], point1[1] + 1)
                point1_on_path = False

        direction = "up"
        
        # Move point2 up until it is not on the path
        while point2_on_path:
             # Checks the movement of the point based on the current direction
            if direction == "up":
                point2 = (point2[0], point2[1] - 1)
            elif direction == "left":
                point2 = (point2[0] - 1, point2[1])
                
            # Checks if the point reached the top edge of the image and changes direction to left
            if point2[1] == 0 and direction == "up":
                direction = "left"
            # Checks if it reached black and changes direction to left
            elif not any(image[int(point2[1]), int(point2[0])]) and direction == "up":
                point2 = (point2[0], point2[1] + 1)
                direction = "left"

            
            
            # Checks to see if point1 is still within the image bounds and prevents the point from exceeding the image
            if point2[0] < 0:
                point2 = (point2[0] + 1, point2[1])
                point2_on_path = False
            elif point2[1] < 0:
                point2 = (point2[0], point2[1] + 1)
                point2_on_path = False
            elif point2[0] > image.shape[1] - 1:
                point2 = (image.shape[1] - 1, point2[1])
                point2_on_path = False
            elif point2[1] > image.shape[0] - 1:
                point2 = (point2[0], image.shape[0] - 1)
                point2_on_path = False
            elif not any(image[int(point2[1]), int(point2[0])]) and direction == "left":
                point2 = (point2[0] + 1, point2[1])
                point2_on_path = False
        
        # Move point3 to the right until it is not on the path
        direction = "right"

        # Move point to the right until it is not on the path
        while point3_on_path:

            # Checks the movement of the point based on the current direction
            if direction == "right":
                point3 = (point3[0] + 1, point3[1])
            elif direction == "up":
                point3 = (point3[0], point3[1] - 1)

            # Checks if it reached black and changes direction to up
            # Checks if the point reached the right edge of the image and changes direction to up
            if point3[0] == image.shape[1] - 1 and direction == "right":
                direction = "up"
            elif not any(image[int(point3[1]), int(point3[0])]) and direction == "right":
                point3 = (point3[0] - 1, point3[1])
                direction = "up"
            
            # Checks to see if point3 is still within the image bounds and prevents the point from exceeding the image
            if point3[0] < 0:
                point3 = (point3[0] + 1, point3[1])
                point3_on_path = False
            elif point3[1] < 0:
                point3 = (point3[0], point3[1] + 1)
                point3_on_path = False
            elif point3[0] > image.shape[1] - 1:
                point3 = (image.shape[1] - 1, point3[1])
                point3_on_path = False
            elif point3[1] > image.shape[0] - 1:
                point3 = (point3[0], image.shape[0] - 1)
                point3_on_path = False
            elif not any(image[int(point3[1]), int(point3[0])]) and direction == "up":
                point3 = (point3[0], point3[1] + 1)
                point3_on_path = False
        
        direction = "up"
        
        # Move point4 up until it is not on the path
        while point4_on_path:
             # Checks the movement of the point based on the current direction
            if direction == "up":
                point4 = (point4[0], point4[1] - 1)
            elif direction == "right":
                point4 = (point4[0] + 1, point4[1])
                
            # Checks if the point reached the top edge of the image and changes direction to left
            if point4[1] == 0 and direction == "up":
                direction = "right"
            # Checks if it reached black and changes direction to left
            elif not any(image[int(point4[1]), int(point4[0])]) and direction == "up":
                point4 = (point4[0], point4[1] + 1)
                direction = "right"

            
            
            # Checks to see if point1 is still within the image bounds and prevents the point from exceeding the image
            if point4[0] < 0:
                point4 = (point4[0] + 1, point4[1])
                point4_on_path = False
            elif point4[1] < 0:
                point4 = (point4[0], point4[1] + 1)
                point4_on_path = False
            elif point4[0] > image.shape[1] - 1:
                point4 = (image.shape[1] - 1, point4[1])
                point4_on_path = False
            elif point4[1] > image.shape[0] - 1:
                point4 = (point4[0], image.shape[0] - 1)
                point4_on_path = False
            elif not any(image[int(point4[1]), int(point4[0])]) and direction == "right":
                point4 = (point4[0] - 1, point4[1])
                point4_on_path = False
        
        # Add the edge points to the image
        cv2.circle(image_with_edges, (int(point1[0]), int(point1[1])), 5, (0, 255, 0), -1)
        cv2.circle(image_with_edges, (int(point2[0]), int(point2[1])), 5, (0, 255, 0), -1)
        cv2.circle(image_with_edges, (int(point3[0]), int(point3[1])), 5, (0, 255, 0), -1)
        cv2.circle(image_with_edges, (int(point4[0]), int(point4[1])), 5, (0, 255, 0), -1)
        
        return image_with_edges, point1, point2, point3, point4

    def filter_points(self, p1, p2, p3, p4):
        
        filtered_p1 = (p1[0] * self.current_percent + self.previous_point1[0] * self.previous_percent, p1[1] * self.current_percent + self.previous_point1[1] * self.previous_percent)
        filtered_p2 = (p2[0] * self.current_percent + self.previous_point2[0] * self.previous_percent, p2[1] * self.current_percent + self.previous_point2[1] * self.previous_percent)
        filtered_p3 = (p3[0] * self.current_percent + self.previous_point3[0] * self.previous_percent, p3[1] * self.current_percent + self.previous_point3[1] * self.previous_percent)
        filtered_p4 = (p4[0] * self.current_percent + self.previous_point4[0] * self.previous_percent, p4[1] * self.current_percent + self.previous_point4[1] * self.previous_percent)

        self.previous_point1 = filtered_p1
        self.previous_point2 = filtered_p2
        self.previous_point3 = filtered_p3
        self.previous_point4 = filtered_p4

        return filtered_p1, filtered_p2, filtered_p3, filtered_p4

    def create_overlay_image(self, original_image: np.ndarray, segmented_image: np.ndarray, opacity=0.3):
        """Create an overlay of the segmented image on the original image with purple transparent pixels"""
        # Create a copy of the original image
        overlay_image = original_image.copy()
        
        # Handle different segmented image formats
        if len(segmented_image.shape) == 3:
            # If segmented image is colored, convert to grayscale
            seg_gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
        else:
            seg_gray = segmented_image
        
        # Create a purple overlay where white pixels exist in segmented image
        purple_overlay = np.zeros_like(original_image)
        purple_color = [128, 0, 128]  # Purple in BGR format
        
        # Find white pixels (assuming 255 is white)
        white_pixels = seg_gray == 255
        
        # Set purple color for white pixels
        purple_overlay[white_pixels] = purple_color
        
        # Blend the overlay with the original image using weighted addition
        overlay_image = cv2.addWeighted(original_image, 1.0, purple_overlay, opacity, 0)
        
        return overlay_image
    
    def check_points(self, p1, p2, p3, p4):

        def dist(point1, point2):
            return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

        # Checks the distance between the points
        try:
            
            if (dist(p1, p2) < MIN_POINT_DISTANCE):
                p1 = self.previous_point1
                p2 = self.previous_point2
                print("Point 1 and 2 are too close")
            elif ((p1[1] - p2[1]) / (p1[0] - p2[0])) > 0:
                p1 = self.previous_point1
                p2 = self.previous_point2
                print("Point 1 and 2 are making a positive slope")
            elif ((p1[1] - p2[1]) / (p1[0] - p2[0])) > -0.05:
                p1 = self.previous_point1
                p2 = self.previous_point2
                print("Point 1 and 2 are too horizontal")
        except ZeroDivisionError:
            print("Point 1 and 2 are vertical")
        
        try:
            if (dist(p3, p4) < MIN_POINT_DISTANCE):
                p3 = self.previous_point3
                p4 = self.previous_point4
                print("Point 3 and 4 are too close")
            elif ((p3[1] - p4[1]) / (p3[0] - p4[0])) < 0:
                p3 = self.previous_point3
                p4 = self.previous_point4
                print("Point 3 and 4 are making a negative slope")
            elif ((p3[1] - p4[1]) / (p3[0] - p4[0])) < 0.05:
                p3 = self.previous_point3
                p4 = self.previous_point4
                print("Point 3 and 4 are too horizontal")
        except ZeroDivisionError:
            print("Point 3 and 4 are vertical")

        return p1, p2, p3, p4

    def process_single_image(self, original_image: np.ndarray, segmented_image: np.ndarray):
        """Process a single image and return the result with lane lines drawn"""
        # Crop the segmented image
        cropped_image, cropped_top, cropped_bottom = self._crop_image(segmented_image)
        
        # Find the main path center
        path_center = self._find_main_path(cropped_image)
        
        # Find edges and get image with edge points
        image_with_edges, point1, point2, point3, point4 = self._find_edges(cropped_image, path_center)
        
        # Adjust points back to original image coordinates
        point1 = (point1[0], point1[1] + cropped_top)
        point2 = (point2[0], point2[1] + cropped_top)
        point3 = (point3[0], point3[1] + cropped_top)
        point4 = (point4[0], point4[1] + cropped_top)

        if self.first_time:
            self.first_time = False
            self.previous_point1 = point1
            self.previous_point2 = point2
            self.previous_point3 = point3
            self.previous_point4 = point4
        
        point1, point2, point3, point4 = self.check_points(point1, point2, point3, point4)
        
        point1, point2, point3, point4 = self.filter_points(point1, point2, point3, point4)

        # Create overlay image with purple segmentation
        overlay_image = self.create_overlay_image(original_image, segmented_image)
        
        # Draw lane lines on the overlay image
        cv2.line(overlay_image, (int(point1[0]), int(point1[1])), (int(point2[0]), int(point2[1])), (255, 0, 0), 4)
        cv2.line(overlay_image, (int(point3[0]), int(point3[1])), (int(point4[0]), int(point4[1])), (255, 0, 0), 4)

        return overlay_image, cropped_image, image_with_edges

    def create_video_visualization(self, image_folder='test_images', output_video='lane_detection_simple.mp4', fps=10):
        """Create a video visualization without matplotlib - should eliminate squishing"""
        from path_segmentation import PathSegmentation
        path_segmenter = PathSegmentation('../sidewalk_segmentation/sidewalk_segmentation_model/model.onnx')
        
        # Get all image files
        image_extension = '*.jpg'
        image_files = []
        image_files.extend(glob.glob(os.path.join(image_folder, image_extension)))
        image_files.extend(glob.glob(os.path.join(image_folder, image_extension.upper())))
        
        image_files.sort()
        
        if not image_files:
            print(f"No images found in {image_folder}")
            return
        
        print(f"Found {len(image_files)} images")
        
        # Process first image to get dimensions
        first_image = cv2.imread(image_files[0])
        height, width = first_image.shape[:2]
        print(f"Video dimensions: {width}x{height}")
        
        # Set up video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_video, fourcc, fps, (width, height))
        
        for i, image_path in enumerate(image_files):
            print(f"Processing image {i+1}/{len(image_files)}")
            
            try:
                # Load and process image
                original_image = cv2.imread(image_path)
                if original_image is None:
                    continue
                
                seg_image = path_segmenter.segment_image(image_path=image_path, show_image=False)
                overlay_result, _, _ = self.process_single_image(original_image, seg_image)
                
                # Add simple text overlay using OpenCV
                cv2.putText(overlay_result, f'Frame {i+1}', (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                # Write directly to video - no matplotlib conversion
                out.write(overlay_result)
                
            except Exception as e:
                print(f"Error: {e}")
                continue
        
        out.release()
        cv2.destroyAllWindows()
        print(f"Simple video saved as: {output_video}")

if __name__ == "__main__":
    lane_detector = LaneDetector()
    
    # Create video visualization at 10 FPS
    lane_detector.create_video_visualization(
        image_folder='test_images',
        output_video='lane_detection_video.mp4',
        fps=10
    )