import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestRegressor
from sklearn.metrics import mean_squared_error
import joblib

# Load the dataset
data = pd.read_csv('C:\\Users\\malin\\Documents\\GitHub\\project-oval\\ML\\position_data_trainer_combined.csv')

# Prepare the features (input data) and labels (correct points)
X = data[['gps_latitude_filtered', 'gps_longitude_filtered']].values
y = data[['swift_latitude_filtered', 'swift_longitude_filtered']].values

# Split the dataset into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Train a Random Forest Regressor
model = RandomForestRegressor(n_estimators=100, random_state=42)
model.fit(X_train, y_train)

# Predict on the test set
y_pred = model.predict(X_test)

# Evaluate the model
mse = mean_squared_error(y_test, y_pred)
print(f'Mean Squared Error: {mse}')

joblib.dump(model, 'C:\\Users\\malin\\Documents\\GitHub\\project-oval\\ML\\gps_adjuster_combined.pkl')