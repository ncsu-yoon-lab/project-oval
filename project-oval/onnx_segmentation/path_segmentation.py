import cv2
import numpy as np
import onnxruntime as ort
from PIL import Image as PILImage

class PathSegmentation():

    def __init__(self, model_path):
        
        self.model_path = model_path

    def _create_colored_mask(self, mask):
        """Convert segmentation mask to a colored visualization"""
        # Define colors for different classes (BGR format for OpenCV)
        color_map = {
            # 0: [0, 0, 0],      # Background (black)
            1: [255, 255, 255], # Road (white)
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

    def segment_image(self, frame: np.ndarray=None, image_path: str=None, show_image: bool=False) -> np.ndarray:
        """
        Run the onnx model on the image to segment it and output as a cv2 np array
        """

        # Initialize ONNX session
        onnx_session = ort.InferenceSession(self.model_path)
        
        if image_path:
            # Load original image
            img_original = cv2.imread(image_path)
        else:
            img_original = frame

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
        
        # Create visualizations
        original_height, original_width = img_original.shape[:2]
        
        # Resize mask to original image size for visualization
        mask_resized = cv2.resize(mask.astype(np.uint8), 
                                (original_width, original_height), 
                                interpolation=cv2.INTER_NEAREST)
        
        # Create colored segmentation mask
        segmentation_colored = self._create_colored_mask(mask_resized)

        if show_image:
            cv2.imshow('Colored Segmentation Mask', segmentation_colored)
            cv2.waitKey(0)
        
        return segmentation_colored
    