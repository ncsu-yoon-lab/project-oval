import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import glob
from pathlib import Path

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

    def process_single_image(self, original_image: np.ndarray, segmented_image: np.ndarray):
        """Process a single image and return the result with lane lines drawn"""
        # Crop the segmented image
        cropped_image, cropped_top, cropped_bottom = self._crop_image(segmented_image)
        
        # Find the main path center
        path_center = self._find_main_path(cropped_image)
        
        # Find edges and get image with edge points
        image_with_edges, point1, point2, point3, point4 = self._find_edges(cropped_image, path_center)
        
        # Adjust points back to original image coordinates
        point1 = (point1[0], point1[1] + cropped_top + cropped_bottom)
        point2 = (point2[0], point2[1] + cropped_top + cropped_bottom)
        point3 = (point3[0], point3[1] + cropped_top + cropped_bottom)
        point4 = (point4[0], point4[1] + cropped_top + cropped_bottom)

        # Create a copy of the original image to draw on
        result_image = original_image.copy()
        
        # Draw lane lines
        cv2.line(result_image, (int(point1[0]), int(point1[1])), (int(point2[0]), int(point2[1])), (255, 0, 0), 4)
        cv2.line(result_image, (int(point3[0]), int(point3[1])), (int(point4[0]), int(point4[1])), (255, 0, 0), 4)

        return result_image, cropped_image, image_with_edges

    def create_video_visualization(self, image_folder='test_images', output_video='lane_detection_video.mp4', fps=10):
        """Create a video visualization of lane detection"""
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend
        import matplotlib.pyplot as plt
        from io import BytesIO
        
        from path_segmentation import PathSegmentation
        path_segmenter = PathSegmentation('../sidewalk_segmentation/sidewalk_segmentation_model/model.onnx')
        
        # Get all image files
        image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff']
        image_files = []
        for ext in image_extensions:
            image_files.extend(glob.glob(os.path.join(image_folder, ext)))
            image_files.extend(glob.glob(os.path.join(image_folder, ext.upper())))
        
        image_files.sort()  # Sort for consistent ordering
        
        if not image_files:
            print(f"No images found in {image_folder}")
            return
        
        print(f"Found {len(image_files)} images")
        
        # Process first image to get dimensions
        first_image = cv2.imread(image_files[0])
        height, width = first_image.shape[:2]
        
        # Set up video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_video, fourcc, fps, (width * 2, height * 2))
        
        for i, image_path in enumerate(image_files):
            print(f"Processing image {i+1}/{len(image_files)}: {os.path.basename(image_path)}")
            
            try:
                # Load the original image
                original_image = cv2.imread(image_path)
                if original_image is None:
                    print(f"Could not read image: {image_path}")
                    continue
                
                # Segment the image to get the path
                seg_image = path_segmenter.segment_image(image_path=image_path, show_image=False)
                
                # Process the image
                result_image, cropped_image, image_with_edges = self.process_single_image(original_image, seg_image)
                
                # Create 2x2 visualization using the Agg backend
                fig, axes = plt.subplots(2, 2, figsize=(12, 8), dpi=100)
                fig.suptitle(f'Lane Detection - Frame {i+1}', fontsize=16)
                
                # Original image with lane lines
                axes[0, 0].imshow(cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
                axes[0, 0].set_title('Original + Lane Lines')
                axes[0, 0].axis('off')
                
                # Segmented image
                if len(seg_image.shape) == 3:
                    axes[0, 1].imshow(cv2.cvtColor(seg_image, cv2.COLOR_BGR2RGB))
                else:
                    axes[0, 1].imshow(seg_image, cmap='gray')
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
                
                # Convert matplotlib figure to image using buffer approach
                buf = BytesIO()
                fig.savefig(buf, format='png', bbox_inches='tight', dpi=100)
                buf.seek(0)
                
                # Read the image from buffer
                img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
                buf.close()
                
                # Decode the PNG image
                img_array = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                # Resize to match expected video dimensions
                img_array = cv2.resize(img_array, (width * 2, height * 2))
                
                # Write frame to video
                out.write(img_array)
                
                plt.close(fig)  # Close figure to free memory
                
            except Exception as e:
                print(f"Error processing {image_path}: {str(e)}")
                continue
        
        # Release video writer
        out.release()
        cv2.destroyAllWindows()
        
        print(f"Video saved as: {output_video}")
        print(f"Video specifications: {len(image_files)} frames at {fps} FPS")

if __name__ == "__main__":
    lane_detector = LaneDetector()
    
    # Create video visualization at 10 FPS
    lane_detector.create_video_visualization(
        image_folder='test_images',
        output_video='lane_detection_video.mp4',
        fps=10
    )