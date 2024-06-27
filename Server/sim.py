# Simulates receiving waypoints and every 1 second it goes to the next waypoint and communicates its current position to the GUI
# Supposed to simulate the car as it travels along the path
import requests
import time
import csv

# Main 
def main():

    while True:

        # Gets the waypoints from the web server
        waypoints = get_waypoints()

        # Wait 2 seconds before continuing loop
        time.sleep(2)

        # Sends the next waypoint to the web server
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
    data = [line.split(',') for line in lines]

    return waypoints

# Sends its current position to the server
def send_current_pos(current_pos):

    # Path to the CSV file
    file_path = 'C:\\Users\\malin\\Documents\\GitHub\\Server\\test.csv'

    # Open the CSV file in write mode to clear its contents and write new data
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(current_pos)

    url = 'http://3.16.149.178/upload'
    files = {'file': open('C:\\Users\\malin\\Documents\\GitHub\\Server\\test.csv', 'rb')}
    response = requests.post(url, files=files)

if __name__ == "__main__":
    main()