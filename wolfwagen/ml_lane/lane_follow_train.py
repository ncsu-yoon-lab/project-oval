import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import pandas as pd
import sys
keras = tf.keras

tf.config.run_functions_eagerly(True)

if len(sys.argv) != 2:
    print("wrong number of cmdline args")
    exit(1)
epochs = int(sys.argv[1])

buf_size = 3000
batch_size = 32
img_width = 128
img_height = 128
channels = 3
input_shape = (img_height, img_width, channels)
data_path = "lanefollow/data/"

# Prepare training data

labels = pd.read_csv(data_path = "meta.csv")["steering"].tolist()

raw_train: tf.data.Dataset = keras.utils.image_dataset_from_directory(
    data_path + "Train",
    labels=labels,
    label_mode="int",
    batch_size=None,
    image_size=(img_height, img_width),
    shuffle=True
).batch(batch_size)

model = keras.Sequential([

# Input
    keras.Input(shape=(img_height, img_width, channels)),

# Normalize 0-255 to 0-1
    keras.layers.Rescaling(scale=1./255),

# First block -- low-level feature maps
    keras.layers.Conv2D(filters=8, kernel_size=3, data_format="channels_last"),
    keras.layers.Activation(activation=keras.activations.swish),

    keras.layers.Conv2D(filters=8, kernel_size=3, data_format="channels_last"),
    keras.layers.Activation(activation=keras.activations.swish),
    
    keras.layers.MaxPool2D(pool_size=(2, 2)),

# Second block -- mid-level features
    keras.layers.Conv2D(filters=16, kernel_size=3, data_format="channels_last"),
    keras.layers.Activation(activation=keras.activations.swish),

    keras.layers.Conv2D(filters=16, kernel_size=3, data_format="channels_last"),
    keras.layers.Activation(activation=keras.activations.swish),
    
    keras.layers.MaxPool2D(pool_size=(2, 2)),

# Third block -- high-level features
    keras.layers.Conv2D(filters=32, kernel_size=3, data_format="channels_last"),
    keras.layers.Activation(activation=keras.activations.swish),

    keras.layers.Conv2D(filters=32, kernel_size=3, data_format="channels_last"),
    keras.layers.Activation(activation=keras.activations.swish),
    
    keras.layers.MaxPool2D(pool_size=(2, 2)),

# Steering generator
    keras.layers.Flatten(),
    keras.layers.Dense(units=256, activation=keras.activations.swish),
    keras.layers.Dense(units=128, activation=keras.activations.swish),
    keras.layers.Dense(units=1, activation=keras.activations.linear)

])

model.summary()

model.compile(
    loss=keras.losses.MeanSquaredError(),
    optimizer=keras.optimizers.RMSprop(),
    metrics=["accuracy"]
)

model.fit(
    x=raw_train,
    epochs=epochs,
    verbose=2
)

model.save("lane_follow_model")