import numpy as np
import joblib

# Gets the saved model
model = joblib.load('gps_adjuster.pkl')

# Function that adjusts the latitude and longitude and returns the point
def adjust_lat_lon(off_lat, off_lon):
    off_point = np.array([[off_lat, off_lon]])
    corrected_point = model.predict(off_point)
    return corrected_point[0]

# Example input
off_lat, off_lon = 35.770804, -78.675410

# Gets the correct lat lon and prints it
corrected_lat_lon = adjust_lat_lon(off_lat, off_lon)
print(f'Corrected Latitude and Longitude: {corrected_lat_lon}')