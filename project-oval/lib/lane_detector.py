import numpy as np
import math

MIN_POINT_DISTANCE = 200

class LaneDetector:
    """
    A class for detecting lane edges from segmented images.
    
    Takes in an original image and its segmented version, and returns
    the detected lane edge points.
    """
    
    def __init__(self):
        self.first_time = True
        self.previous_point1 = None
        self.previous_point2 = None
        self.previous_point3 = None
        self.previous_point4 = None
        self.previous_percent = 0.2
        self.current_percent = 1.0 - self.previous_percent

    def _crop_image(self, image: np.ndarray) -> tuple:
        """Crop the image to focus on the relevant area."""
        num_rows, num_columns, _ = image.shape

        cropped_top = int(num_rows / 2)
        cropped_image = image[cropped_top:, 0:num_columns]

        num_rows, num_columns, _ = cropped_image.shape

        left_point = (0, num_rows - 1)
        right_point = (num_columns - 1, num_rows - 1)

        # Move the points up until they reach the first black pixel
        while(any(cropped_image[int(left_point[1]), int(left_point[0])]) and 
              any(cropped_image[int(right_point[1]), int(right_point[0])]) and 
              (right_point[1] > 0 and left_point[1] > 0)):
            left_point = (left_point[0], left_point[1] - 1)
            right_point = (right_point[0], right_point[1] - 1)
        
        cropped_bottom = num_rows - int(left_point[1])
        cropped_image = cropped_image[0:int(left_point[1]), 0:num_columns]

        return cropped_image, cropped_top, cropped_bottom
    
    def _find_main_path(self, image: np.ndarray) -> tuple:
        """Find the center point of the main path."""
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
    
    def _find_edges(self, image: np.ndarray, path_center: tuple) -> tuple:
        """Find the four edge points of the lanes."""
        # Point 1 goes to the left, point 2 goes up, point 3 goes to the right, point 4 goes up
        point1 = point2 = point3 = point4 = path_center

        point1_on_path = point2_on_path = point3_on_path = point4_on_path = True

        direction = "left"

        # Move point1 to the left until it is not on the path
        while point1_on_path:
            if direction == "left":
                point1 = (point1[0] - 1, point1[1])
            elif direction == "up":
                point1 = (point1[0], point1[1] - 1)

            if point1[0] == 0 and direction == "left":
                direction = "up"
            elif not any(image[int(point1[1]), int(point1[0])]) and direction == "left":
                point1 = (point1[0] + 1, point1[1])
                direction = "up"

            # Check bounds
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
            if direction == "up":
                point2 = (point2[0], point2[1] - 1)
            elif direction == "left":
                point2 = (point2[0] - 1, point2[1])
                
            if point2[1] == 0 and direction == "up":
                direction = "left"
            elif not any(image[int(point2[1]), int(point2[0])]) and direction == "up":
                point2 = (point2[0], point2[1] + 1)
                direction = "left"

            # Check bounds
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

        while point3_on_path:
            if direction == "right":
                point3 = (point3[0] + 1, point3[1])
            elif direction == "up":
                point3 = (point3[0], point3[1] - 1)

            if point3[0] == image.shape[1] - 1 and direction == "right":
                direction = "up"
            elif not any(image[int(point3[1]), int(point3[0])]) and direction == "right":
                point3 = (point3[0] - 1, point3[1])
                direction = "up"
            
            # Check bounds
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
            if direction == "up":
                point4 = (point4[0], point4[1] - 1)
            elif direction == "right":
                point4 = (point4[0] + 1, point4[1])
                
            if point4[1] == 0 and direction == "up":
                direction = "right"
            elif not any(image[int(point4[1]), int(point4[0])]) and direction == "up":
                point4 = (point4[0], point4[1] + 1)
                direction = "right"

            # Check bounds
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
        
        return point1, point2, point3, point4

    def _filter_points(self, p1: tuple, p2: tuple, p3: tuple, p4: tuple) -> tuple:
        """Apply temporal filtering to smooth point transitions."""
        filtered_p1 = (p1[0] * self.current_percent + self.previous_point1[0] * self.previous_percent, 
                      p1[1] * self.current_percent + self.previous_point1[1] * self.previous_percent)
        filtered_p2 = (p2[0] * self.current_percent + self.previous_point2[0] * self.previous_percent, 
                      p2[1] * self.current_percent + self.previous_point2[1] * self.previous_percent)
        filtered_p3 = (p3[0] * self.current_percent + self.previous_point3[0] * self.previous_percent, 
                      p3[1] * self.current_percent + self.previous_point3[1] * self.previous_percent)
        filtered_p4 = (p4[0] * self.current_percent + self.previous_point4[0] * self.previous_percent, 
                      p4[1] * self.current_percent + self.previous_point4[1] * self.previous_percent)

        self.previous_point1 = filtered_p1
        self.previous_point2 = filtered_p2
        self.previous_point3 = filtered_p3
        self.previous_point4 = filtered_p4

        return filtered_p1, filtered_p2, filtered_p3, filtered_p4

    def _check_points(self, p1: tuple, p2: tuple, p3: tuple, p4: tuple) -> tuple:
        """Validate points and replace with previous if invalid."""
        def dist(point1: tuple, point2: tuple) -> float:
            return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

        # Check left lane points (p1, p2)
        try:
            if (dist(p1, p2) < MIN_POINT_DISTANCE):
                p1 = self.previous_point1
                p2 = self.previous_point2
            elif ((p1[1] - p2[1]) / (p1[0] - p2[0])) > 0:
                p1 = self.previous_point1
                p2 = self.previous_point2
            elif ((p1[1] - p2[1]) / (p1[0] - p2[0])) > -0.05:
                p1 = self.previous_point1
                p2 = self.previous_point2
        except ZeroDivisionError:
            pass  # Points are vertical, which is acceptable
        
        # Check right lane points (p3, p4)
        try:
            if (dist(p3, p4) < MIN_POINT_DISTANCE):
                p3 = self.previous_point3
                p4 = self.previous_point4
            elif ((p3[1] - p4[1]) / (p3[0] - p4[0])) < 0:
                p3 = self.previous_point3
                p4 = self.previous_point4
            elif ((p3[1] - p4[1]) / (p3[0] - p4[0])) < 0.05:
                p3 = self.previous_point3
                p4 = self.previous_point4
        except ZeroDivisionError:
            pass  # Points are vertical, which is acceptable

        return p1, p2, p3, p4

    def detect_lanes(self, original_image: np.ndarray, segmented_image: np.ndarray) -> dict:
        """
        Detect lane edges from images.
        
        Args:
            original_image: The original input image (used for coordinate reference)
            segmented_image: The segmented image showing the path/lane areas
            
        Returns:
            dict: Dictionary containing lane edge information with keys:
                - 'left_lane': tuple of two points (p1, p2) representing left lane edge
                - 'right_lane': tuple of two points (p3, p4) representing right lane edge
                - 'path_center': tuple representing the center point of the path
        """
        # Crop the segmented image
        cropped_image, cropped_top, cropped_bottom = self._crop_image(segmented_image)
        
        # Find the main path center
        path_center = self._find_main_path(cropped_image)
        
        # Find edges
        point1, point2, point3, point4 = self._find_edges(cropped_image, path_center)
        
        # Adjust points back to original image coordinates
        point1 = (point1[0], point1[1] + cropped_top)
        point2 = (point2[0], point2[1] + cropped_top)
        point3 = (point3[0], point3[1] + cropped_top)
        point4 = (point4[0], point4[1] + cropped_top)
        path_center = (path_center[0], path_center[1] + cropped_top)

        # Initialize previous points on first run
        if self.first_time:
            self.first_time = False
            self.previous_point1 = point1
            self.previous_point2 = point2
            self.previous_point3 = point3
            self.previous_point4 = point4
        
        # Validate and filter points
        point1, point2, point3, point4 = self._check_points(point1, point2, point3, point4)
        point1, point2, point3, point4 = self._filter_points(point1, point2, point3, point4)

        return {
            'left_lane': (point1, point2),
            'right_lane': (point3, point4),
            'path_center': path_center
        }