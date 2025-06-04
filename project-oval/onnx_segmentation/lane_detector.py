import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
import random

class LaneDetector():

    def __init__(self):
        pass

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

    def detect_lane_edges(self, original_image: np.ndarray, segmented_image: np.ndarray, image_name: str = ""):
        # Crop the segmented image
        cropped_image, cropped_top, cropped_bottom = self._crop_image(segmented_image)
        
        # Find the main path center
        path_center = self._find_main_path(cropped_image)
        
        # Find edges and get image with edge points
        image_with_edges, point1, point2, point3, point4 = self._find_edges(cropped_image, path_center)
        print(cropped_bottom)
        print(cropped_top)
        print(point1, point2, point3, point4)
        point1 = (point1[0], point1[1] + cropped_top + cropped_bottom)
        point2 = (point2[0], point2[1] + cropped_top + cropped_bottom)
        point3 = (point3[0], point3[1] + cropped_top + cropped_bottom)
        point4 = (point4[0], point4[1] + cropped_top + cropped_bottom)  
        print(point1, point2, point3, point4)

        cv2.line(original_image, (int(point1[0]), int(point1[1])), (int(point2[0]), int(point2[1])), (255, 0, 0), 4)
        cv2.line(original_image, (int(point3[0]), int(point3[1])), (int(point4[0]), int(point4[1])), (255, 0, 0), 4)

        # Create the visualization
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'Lane Detection Results - {image_name}', fontsize=16)
        
        # Original image
        if len(original_image.shape) == 3:
            axes[0, 0].imshow(cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB))
        else:
            axes[0, 0].imshow(original_image, cmap='gray')
        axes[0, 0].set_title('Original Image')
        axes[0, 0].axis('off')
        
        # Segmented image
        if len(segmented_image.shape) == 3:
            axes[0, 1].imshow(cv2.cvtColor(segmented_image, cv2.COLOR_BGR2RGB))
        else:
            axes[0, 1].imshow(segmented_image, cmap='gray')
        axes[0, 1].set_title('Segmented Image')
        axes[0, 1].axis('off')
        
        # Cropped image
        if len(cropped_image.shape) == 3:
            axes[1, 0].imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
        else:
            axes[1, 0].imshow(cropped_image, cmap='gray')
        axes[1, 0].set_title('Cropped Image')
        axes[1, 0].axis('off')
        
        # Image with lane edges
        if len(image_with_edges.shape) == 3:
            axes[1, 1].imshow(cv2.cvtColor(image_with_edges, cv2.COLOR_BGR2RGB))
        else:	
            axes[1, 1].imshow(image_with_edges, cmap='gray')
        axes[1, 1].set_title('Lane Edges Detected')
        axes[1, 1].axis('off')
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    lane_detector = LaneDetector()
    from path_segmentation import PathSegmentation
    path_segmenter = PathSegmentation('../sidewalk_segmentation/sidewalk_segmentation_model/model.onnx')

    for image in os.listdir('test_images'):
        image_path = os.path.join('test_images', image)
        print(f"Processing {image_path}")
        
        # Load the original image
        original_image = cv2.imread(image_path)
        
        # Segment the image to get the path
        seg_image = path_segmenter.segment_image(image_path=image_path, show_image=False)

        # Detect lane edges and display all images in one plot
        lane_detector.detect_lane_edges(original_image, seg_image, image)