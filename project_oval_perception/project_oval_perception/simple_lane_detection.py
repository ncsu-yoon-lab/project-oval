import numpy as np
import cv2

class SimpleLaneDetector:
    """
    Detects lane edges by dividing image into sections and tracking the center of drivable area.
    Handles intersections by following the side with more drivable space.
    """
    
    def __init__(self, smoothing_factor=0.7, look_ahead_ratio=0.7):
        """
        Args:
            smoothing_factor: How much to weight current vs previous (0-1, higher = more responsive)
            look_ahead_ratio: How far ahead to look (0-1, where to sample in the image)
        """
        self.smoothing_factor = smoothing_factor
        self.look_ahead_ratio = look_ahead_ratio
        self.prev_left_edge = None
        self.prev_right_edge = None
        self.prev_target = None
        self.robot_position = None  # Will be set to center bottom
    
    def detect_lanes(self, segmented_image: np.ndarray) -> dict:
        """
        Detect lane edges and calculate steering target.
        
        Args:
            segmented_image: 224x224 binary image (1 = drivable, 0 = non-drivable)
            
        Returns:
            dict: {
                'left_edge': (x, y) - point on left edge
                'right_edge': (x, y) - point on right edge
                'target_point': (x, y) - where robot should steer towards
                'robot_position': (x, y) - current robot position
                'steering_error': float - how far off center (-1 to 1)
            }
        """
        # Ensure binary image
        if len(segmented_image.shape) == 3:
            gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = segmented_image.copy()
        
        # Normalize to 0 and 1
        binary = (gray > 127).astype(np.uint8)
        
        height, width = binary.shape
        
        # Set robot position at bottom center (first time only)
        if self.robot_position is None:
            self.robot_position = (width // 2, height - 10)
        
        # Define look-ahead point (where we sample the road)
        look_ahead_y = int(height * self.look_ahead_ratio)
        
        # Divide image into three sections
        left_section_end = width // 3
        right_section_start = 2 * width // 3
        
        # Sample at the look-ahead line
        sample_row = binary[look_ahead_y, :]
        
        # Find all drivable pixels in the sample row
        drivable_pixels = np.where(sample_row > 0)[0]
        
        if len(drivable_pixels) == 0:
            # No lane detected - use previous or default
            return self._use_previous_or_default(width, height)
        
        # --- Find LEFT edge (in left section) ---
        left_section_pixels = drivable_pixels[drivable_pixels < width // 2]
        if len(left_section_pixels) > 0:
            left_x = left_section_pixels[0]  # Leftmost drivable pixel
        else:
            left_x = drivable_pixels[0]  # Fallback to overall leftmost
        
        # --- Find RIGHT edge (in right section) ---
        right_section_pixels = drivable_pixels[drivable_pixels > width // 2]
        if len(right_section_pixels) > 0:
            right_x = right_section_pixels[-1]  # Rightmost drivable pixel
        else:
            right_x = drivable_pixels[-1]  # Fallback to overall rightmost
        
        left_edge = (int(left_x), look_ahead_y)
        right_edge = (int(right_x), look_ahead_y)
        
        # --- Calculate TARGET point (center of mass of drivable area) ---
        # This is smarter than just midpoint - handles intersections!
        # Weight towards the side with more drivable pixels
        center_of_mass_x = int(np.mean(drivable_pixels))
        target_point = (center_of_mass_x, look_ahead_y)
        
        # Apply smoothing
        if self.prev_left_edge is not None:
            left_edge = self._smooth_point(left_edge, self.prev_left_edge)
            right_edge = self._smooth_point(right_edge, self.prev_right_edge)
            target_point = self._smooth_point(target_point, self.prev_target)
        
        # Update previous values
        self.prev_left_edge = left_edge
        self.prev_right_edge = right_edge
        self.prev_target = target_point
        
        # Calculate steering error (-1 = left, 0 = center, 1 = right)
        lane_center = (left_edge[0] + right_edge[0]) / 2
        steering_error = (target_point[0] - self.robot_position[0]) / (width / 2)
        steering_error = np.clip(steering_error, -1.0, 1.0)
        
        return {
            'left_edge': left_edge,
            'right_edge': right_edge,
            'target_point': target_point,
            'robot_position': self.robot_position,
            'steering_error': steering_error,
            'lane_width': right_edge[0] - left_edge[0]
        }
    
    def _smooth_point(self, current: tuple, previous: tuple) -> tuple:
        """Apply exponential smoothing to reduce jitter."""
        x = int(current[0] * self.smoothing_factor + previous[0] * (1 - self.smoothing_factor))
        y = int(current[1] * self.smoothing_factor + previous[1] * (1 - self.smoothing_factor))
        return (x, y)
    
    def _use_previous_or_default(self, width: int, height: int) -> dict:
        """Return previous values or defaults if no lane detected."""
        if self.prev_target is not None:
            return {
                'left_edge': self.prev_left_edge,
                'right_edge': self.prev_right_edge,
                'target_point': self.prev_target,
                'robot_position': self.robot_position,
                'steering_error': 0.0,
                'lane_width': self.prev_right_edge[0] - self.prev_left_edge[0]
            }
        else:
            default_y = int(height * self.look_ahead_ratio)
            return {
                'left_edge': (width // 4, default_y),
                'right_edge': (3 * width // 4, default_y),
                'target_point': (width // 2, default_y),
                'robot_position': self.robot_position,
                'steering_error': 0.0,
                'lane_width': width // 2
            }
    
    def visualize(self, image: np.ndarray, detection_result: dict) -> np.ndarray:
        """
        Draw comprehensive visualization with sections, edges, and steering.
        
        Args:
            image: Original image to draw on
            detection_result: Output from detect_lanes()
            
        Returns:
            Image with visualization
        """
        vis_image = image.copy()
        if len(vis_image.shape) == 2:
            vis_image = cv2.cvtColor(vis_image, cv2.COLOR_GRAY2BGR)
        
        height, width = vis_image.shape[:2]
        
        left_edge = detection_result['left_edge']
        right_edge = detection_result['right_edge']
        target_point = detection_result['target_point']
        robot_pos = detection_result['robot_position']
        steering_error = detection_result['steering_error']
        
        # Draw section dividers (faint lines)
        cv2.line(vis_image, (width // 3, 0), (width // 3, height), (100, 100, 100), 1)
        cv2.line(vis_image, (2 * width // 3, 0), (2 * width // 3, height), (100, 100, 100), 1)
        
        # Draw lane edges with thicker lines
        cv2.circle(vis_image, left_edge, 8, (0, 0, 255), -1)  # Red
        cv2.circle(vis_image, left_edge, 10, (255, 255, 255), 2)
        
        cv2.circle(vis_image, right_edge, 8, (255, 0, 0), -1)  # Blue
        cv2.circle(vis_image, right_edge, 10, (255, 255, 255), 2)
        
        # Draw lane boundary line
        cv2.line(vis_image, left_edge, right_edge, (255, 255, 0), 3)
        
        # Draw target point (green) - where robot should aim
        cv2.circle(vis_image, target_point, 12, (0, 255, 0), -1)
        cv2.circle(vis_image, target_point, 14, (255, 255, 255), 2)
        
        # Draw robot position (red dot)
        cv2.circle(vis_image, robot_pos, 10, (0, 0, 255), -1)
        cv2.circle(vis_image, robot_pos, 12, (255, 255, 255), 2)
        
        # Draw steering line (from robot to target)
        color = (0, 255, 0) if abs(steering_error) < 0.2 else (0, 165, 255)  # Green if centered, orange if off
        cv2.line(vis_image, robot_pos, target_point, color, 3)
        cv2.arrowedLine(vis_image, robot_pos, target_point, color, 3, tipLength=0.3)
        
        # Add text info
        cv2.putText(vis_image, f"Steer: {steering_error:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, "LEFT", (left_edge[0] - 30, left_edge[1] - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(vis_image, "RIGHT", (right_edge[0] + 10, right_edge[1] - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(vis_image, "TARGET", (target_point[0] - 35, target_point[1] - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, "ROBOT", (robot_pos[0] - 30, robot_pos[1] + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return vis_image