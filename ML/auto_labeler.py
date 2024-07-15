from building_identifier import BuildingIdentifier as BI
import cv2

building_x = None

def get_building_x(event, x, y, flags, params):
    """Gets the x position of the center of the bounding box of the building"""

    global building_x

    if event == cv2.EVENT_LBUTTONDOWN:
        building_x = x
        print(f"Clicked coordinates: x={x}, y={y}")


def get_current_pos():
    """Gets the current position and angle of the car at the same moment the photo was taken"""

    current_latitude = 35.77013201
    current_longitude = -78.67478228
    current_heading = 214.3448479

    return current_latitude, current_longitude, current_heading
    
def main():
    """Main function for identifying a building based on the original image, the center x of the bounding box of the building, and the position of the vehicle at that moment"""

    # Initializes the building identifier
    identifier = BI(35.770849, -78.674706)

    # Gets the image
    img = cv2.imread("C:\\Users\\malin\\Downloads\\2024-07-11-09_21_01.jpg")

    # Display the image and set the mouse callback to capture the click event
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', get_building_x)
    
    # Wait until a key is pressed or a mouse click happens
    cv2.waitKey(0)

    # Gets the width of the image 
    width = img.shape[1]

    # Finds the center of the image
    center = width / 2

    # Gets the x position of the building based on the center x pixel in the image where the building was identified
    building_x_position = building_x - center

    # Gets the current latitude and longitude and heading at the same moment of the image
    current_latitude, current_longitude, current_heading = get_current_pos()

    # Field of view horizontal is 110 degrees
    FOV = 110

    # Degrees per pixel
    dpp = FOV / width

    # Gets the angle from the lense to the building based on its x position and the degrees per pixel
    building_angle = dpp * building_x_position

    # Prints the building that is identified at that angle
    print(identifier.get_building_name(current_latitude=current_latitude, current_longitude=current_longitude, current_heading=current_heading, building_angle=building_angle))

if __name__ == "__main__":
    main()
    