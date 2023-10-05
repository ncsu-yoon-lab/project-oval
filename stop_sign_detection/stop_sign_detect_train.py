import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import pandas as pd
import sys
keras = tf.keras # idk why I have to do this but it breaks if I dont

tf.config.run_functions_eagerly(True)

NUM_CLASSES = 43

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
data_path = "data/"

# Prepare training data

class_names = [ "0",  "1",  "2",  "3",  "4",  "5",  "6",  "7",  "8",  "9",
               "10", "11", "12", "13", "14", "15", "16", "17", "18", "19",
               "20", "21", "22", "23", "24", "25", "26", "27", "28", "29",
               "30", "31", "32", "33", "34", "35", "36", "37", "38", "39",
               "40", "41", "42"]

raw_train: tf.data.Dataset = keras.utils.image_dataset_from_directory(
    data_path + "Train",
    labels="inferred",
    label_mode="int",
    class_names=class_names,
    batch_size=None,
    image_size=(img_height, img_width),
    shuffle=True
).batch(batch_size)

# pd_train_classifications: pd.Series = pd.read_csv(data_path + "Train.csv")["ClassId"].sort_values(0, ascending=True)
# raw_train_half: tf.data.Dataset = keras.utils.image_dataset_from_directory(
#     data_path + "Train",
#     label_mode=None,
#     batch_size=None,
#     image_size=(img_height, img_width),
#     shuffle=False
# )

# train_classifications: tf.data.Dataset = tf.data.Dataset.from_tensor_slices(pd_train_classifications)
# raw_train = tf.data.Dataset.zip((raw_train_half, train_classifications)).shuffle(buf_size).batch(batch_size)

# inputs, expecteds = iter(raw_train).get_next()

# cols = 8
# rows = batch_size // cols
# fig, axes = plt.subplots(rows, cols)

# for e_i in range(batch_size):
#     axes[e_i // cols][e_i % cols].imshow(inputs[e_i] / 255)
# print(expecteds)
# plt.show()
# exit()

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

# Classifier
    keras.layers.Flatten(),
    keras.layers.Dense(units=256, activation=keras.activations.swish),
    keras.layers.Dense(units=128, activation=keras.activations.swish),
    keras.layers.Dense(units=NUM_CLASSES, activation=keras.activations.softmax)

])

model.summary()

model.compile(
    loss=keras.losses.SparseCategoricalCrossentropy(),
    optimizer=keras.optimizers.RMSprop(),
    metrics=["accuracy"]
)

model.fit(
    x=raw_train,
    epochs=epochs,
    verbose=2
)

model.save("stop_sign_model")