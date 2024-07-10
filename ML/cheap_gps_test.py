import numpy as np
import joblib
import pandas as pd
import csv
import os

FILE = "C:\\Users\\malin\\Downloads\\Project_Oval_Test5\\combined_data_test5.csv"
FIELD_NAMES = ['timestamp', 'swift_latitude', 'swift_longitude', 'zed_x', 'zed_y', 'gps_latitude', 'gps_longitude', 'ml_latitude', 'ml_longitude']

# Convert the degrees minutes to decimal degrees
def convert_degrees(point):
    degrees = np.floor(point / 100)
    minutes = point - degrees * 100
    return degrees + minutes / 60

def main():

    # Load the dataset
    gps_data = pd.read_csv("C:\\Users\\malin\\Downloads\\Project_Oval_Test5\\data_gps_logger_test5.csv")
    zed_data = pd.read_csv("C:\\Users\\malin\\Downloads\\Project_Oval_Test5\\data_zed_logger_test5.csv")


    # Receives the gps lat and longs and converts them to degrees in the same csv
    gps_data['gps_lat'] = gps_data['gps_lat'].apply(convert_degrees)
    gps_data['gps_lon'] = gps_data['gps_lon'].apply(lambda x: -convert_degrees(x))

    start_time = time_to_sec(str(gps_data['timestamp'][0]))
    end_time = time_to_sec(str(gps_data['timestamp'][len(gps_data['timestamp']) - 1]))

    if start_time > time_to_sec(str(zed_data['timestamp'][0])):
        start_time = time_to_sec(str(zed_data['timestamp'][0]))
    
    if end_time < time_to_sec(str(zed_data['timestamp'][len(zed_data['timestamp']) - 1])):
        end_time = time_to_sec(str(zed_data['timestamp'][len(zed_data['timestamp']) - 1]))

    time = start_time

    # Gets the saved model
    model = joblib.load('gps_adjuster.pkl')

    previous_gps_lat = previous_gps_lon = previous_swift_lat = previous_swift_lon = previous_zed_x = previous_zed_y = 0.0

    # Goes through every time stamp 
    while time != end_time:

        str_time = sec_to_time(time)

        # Checks if the timestamp is in the gps data frame
        if str_time in gps_data["timestamp"].values:
            idx = gps_data.index[gps_data["timestamp"] == str_time].tolist()[0]
            gps_latitude = gps_data["gps_lat"][idx]
            gps_longitude = gps_data["gps_lon"][idx]

            previous_gps_lat = gps_latitude
            previous_gps_lon = gps_longitude
        else:
            gps_latitude = previous_gps_lat
            gps_longitude = previous_gps_lon
        
        # Checks if the timestamp is in the zed data frame
        if str_time in zed_data["timestamp"].values:
            idx = zed_data.index[zed_data["timestamp"] == str_time].tolist()[0]
            
            swift_latitude = zed_data["latitude"][idx]
            swift_longitude = zed_data["longitude"][idx]

            previous_swift_lat = swift_latitude
            previous_swift_lon = swift_longitude

            zed_x = zed_data["zed_x"][idx]
            zed_y = zed_data["zed_y"][idx]

            previous_zed_x = zed_x
            previous_zed_y = zed_y
        else:
            zed_x = previous_zed_x
            zed_y = previous_zed_y

            swift_latitude = previous_swift_lat
            swift_longitude = previous_swift_lon

        # Gets the correct lat lon and prints it
        ml_lat, ml_lon = adjust_lat_lon(gps_latitude, gps_longitude, model)
        
        data = [str_time, swift_latitude, swift_longitude, zed_x, zed_y, gps_latitude, gps_longitude, ml_lat, ml_lon]

        log_data(data)

        time += 1

def time_to_sec(time):
    hours, minutes, seconds = map(int, time.split(':'))

    return int(hours) * 3600 + int(minutes) * 60 + int(seconds)

def sec_to_time(sec):
    hours = sec // 3600
    min = (sec // 60) % 60
    sec = (sec % 60) % 60

    if sec % 10 == sec:
        sec = "0" + str(sec)

    return str(hours) + ":" + str(min) + ":" + str(sec)

def log_data(data):
    saved_data = {
        FIELD_NAMES[0]: data[0],
        FIELD_NAMES[1]: data[1],
        FIELD_NAMES[2]: data[2],
        FIELD_NAMES[3]: data[3],
        FIELD_NAMES[4]: data[4],
        FIELD_NAMES[5]: data[5],
        FIELD_NAMES[6]: data[6],
        FIELD_NAMES[7]: data[7],
        FIELD_NAMES[8]: data[8]
    }

    file_exists = os.path.isfile(FILE)

    with open(FILE, 'a', newline = '') as csvfile:

        writer = csv.DictWriter(csvfile, fieldnames = FIELD_NAMES)

        if not file_exists:
            writer.writeheader()
        
        writer.writerow(saved_data)

# Function that adjusts the latitude and longitude and returns the point
def adjust_lat_lon(off_lat, off_lon, model):
    off_point = np.array([[off_lat, off_lon]])
    corrected_point = model.predict(off_point)
    return corrected_point[0][0], corrected_point[0][1]

if __name__ == "__main__":
    main()