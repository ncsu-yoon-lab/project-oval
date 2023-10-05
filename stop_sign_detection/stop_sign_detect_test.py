import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import pandas as pd
keras = tf.keras # idk why I have to do this but it breaks if I dont

tf.config.run_functions_eagerly(True)

buf_size = 3000
batch_size = 10
img_width = 128
img_height = 128
channels = 3
input_shape = (img_height, img_width, channels)
data_path = "data/"

# Prepare test data

pd_test_classifications: pd.Series = pd.read_csv(data_path + "Test.csv")["ClassId"]
raw_test_half: tf.data.Dataset = keras.utils.image_dataset_from_directory(
    data_path + "Test",
    label_mode=None,
    batch_size=None,
    image_size=(img_height, img_width),
    shuffle=False
)

test_classifications: tf.data.Dataset = tf.data.Dataset.from_tensor_slices(pd_test_classifications)
raw_test = tf.data.Dataset.zip((raw_test_half, test_classifications)).shuffle(buf_size).batch(batch_size)

# Load existing model
model: keras.Model = keras.models.load_model("stop_sign_model")

print(model.evaluate(raw_test))

# inputs, expecteds = iter(raw_test).get_next()
# expecteds = expecteds.numpy()

# fig, axes = plt.subplots(batch_size)

# e_i = 0
# for prediction in model.predict(inputs):
#     axes[e_i].imshow(inputs[e_i] / 255)
#     print("expected: [", expecteds[e_i], "] =", prediction[expecteds[e_i]])
#     print("predicted: [", prediction.argmax(), "] =", prediction.max())
#     print()
#     e_i += 1
# plt.show()