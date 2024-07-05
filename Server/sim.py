# Simulates receiving waypoints and every 1 second it goes to the next waypoint and communicates its current position to the GUI
# Supposed to simulate the car as it travels along the path
import requests
import time
import csv
from coords_to_cartesian import CoordsToCartesian as c2c

converter = c2c(35.7713528, -78.673756)

# Main 
def main():

    while True:

        # Gets the waypoints from the web server
        waypoints = get_waypoints()

        # Wait 2 seconds before continuing loop
        time.sleep(2)

        # Sends the next waypoint to the web server
        if waypoints[0] != 'File not found':
            send_current_pos(waypoints[1])

        # Wait 2 seconds before continuing loop
        time.sleep(2)

# Gets the waypoints from the server
def get_waypoints():
    url = 'http://3.16.149.178/download/waypoints.csv'

    # Gets the response from the server
    response = requests.get(url)

    # Decode the byte string
    csv_content = response.content.decode('utf-8')

    # Split the content by lines
    lines = csv_content.splitlines()

    # Parse the lines into a 2D array
    waypoints = [line.split(',') for line in lines]
    
    return waypoints

# Sends its current position to the server
def send_current_pos(current_pos):
    

    # Path to the CSV file
    file_path = 'C:\\Users\\malin\\Documents\\GitHub\\project-oval\\Server\\pos.csv'

    # Open the CSV file in write mode to clear its contents and write new data
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(current_pos)

    url = 'http://3.16.149.178/upload'
    files = {'file': open(file_path, 'rb')}
    response = requests.post(url, files=files)

    print(converter.latlon_to_xy(current_pos[0], current_pos[1]))

if __name__ == "__main__":
    main()