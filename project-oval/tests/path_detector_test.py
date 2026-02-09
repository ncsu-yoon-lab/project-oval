#!/usr/bin/env python
import cv2
import numpy as np
import glob
from pathlib import Path

def view_all_segmentation_results(output_dir):
    output_dir = Path(output_dir)
    original_files = sorted(glob.glob(str(output_dir / "original_numpy" / "*.npy")))
    
    for original_file in original_files:
        base_name = Path(original_file).stem.replace("_original", "")
        segmented_file = output_dir / "segmented_numpy" / f"{base_name}_segmented.npy"
        
        original_image = np.load(original_file)
        segmented_mask = np.load(segmented_file)
        
        if len(segmented_mask.shape) == 2:
            segmented_display = cv2.cvtColor(segmented_mask, cv2.COLOR_GRAY2BGR)
        else:
            segmented_display = segmented_mask
            
        if original_image.shape[:2] != segmented_display.shape[:2]:
            segmented_display = cv2.resize(segmented_display, (original_image.shape[1], original_image.shape[0]))
        
        combined_image = np.hstack([original_image, segmented_display])
        cv2.putText(combined_image, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined_image, "Segmented", (original_image.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        cv2.imshow("Segmentation Results", combined_image)
        cv2.waitKey(0)
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # CHANGE THIS TO YOUR DIRECTORY WITH IMAGES
    output_directory = "/home/wolfwagen1/ros2_ws/src/project-oval/output"
    view_all_segmentation_results(output_directory)