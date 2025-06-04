import cv2
import numpy as np
import onnxruntime as ort
from PIL import Image as PILImage
import matplotlib.pyplot as plt

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
    # results = lane_detector.calculate_distance_to_path_edge(left_line, right_line, lines_image.shape)
    # print("Distance: ", results["right_edge_distance"])
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

# Example usage:
if __name__ == "__main__":
    image_path = "small_path.jpg"
    model_path = "../sidewalk_segmentation/sidewalk_segmentation_model/model.onnx"
    
    try:
        mask, lines_img, left, right = test_onnx_with_image_debug(image_path, model_path)
        print("\n=== FINAL RESULTS ===")
        print(f"Left line: {left}")
        print(f"Right line: {right}")
        
        if left is None and right is None:
            print("\nNO LINES DETECTED!")
            print("Try adjusting lane detection parameters:")
            print("- Lower Canny thresholds (canny_low, canny_high)")
            print("- Lower Hough threshold")
            print("- Shorter minLineLength")
            print("- Check if road_classes [1, 3, 4] match your segmentation model")
        
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()