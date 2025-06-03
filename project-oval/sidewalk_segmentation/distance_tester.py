import math

def calculate_distance(image_height, vertical_fov_degrees, camera_height, y_pixel):
    """
    Calculate the distance to a point on the ground given its y-coordinate in the image.
    
    Args:
        image_height (int): Height of the image in pixels
        vertical_fov_degrees (float): Vertical field of view in degrees
        camera_height (float): Height of the camera from the ground
        y_pixel (int): Y-coordinate of the point in the image (0 is top of image)
    
    Returns:
        float: Distance to the point on the ground
    """
    # Convert FOV from degrees to radians
    vertical_fov_rad = math.radians(vertical_fov_degrees)
    
    # Calculate the angle from the camera's optical center to the point
    # First, convert pixel position to normalized coordinates (-1 to 1)
    normalized_y = (y_pixel / image_height) * 2 - 1
    # Then calculate the angle using FOV
    angle = math.atan(math.tan(vertical_fov_rad/2) * normalized_y)
    
    # Calculate distance using trigonometry
    # Since the camera is parallel to ground, we can use:
    # tan(angle) = camera_height / ground_distance
    ground_distance = camera_height / math.tan(-angle)  # Negative because y increases downward in image
    
    return ground_distance

distance = calculate_distance(
    image_height=1080,        # 1080p image
    vertical_fov_degrees=110,  # 60° vertical FOV
    camera_height=0.7,          # Camera is 2 meters high
    y_pixel=              # Point is at y=800 in the image
)
print(f"Distance to point: {distance:.2f} meters")