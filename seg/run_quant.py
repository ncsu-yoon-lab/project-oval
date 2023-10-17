import cv2
import tensorflow as tf
import numpy as np


#creating numpy array
picture = cv2.resize(cv2.imread("test.jpg"), (512, 512))
picture = np.asarray([(picture / 255).astype(np.float32)])
picture = np.transpose(picture, [0, 3, 1, 2])

# Load the TFLite model and allocate tensors.
interpreter = tf.lite.Interpreter(model_path="model.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Test the model on random input data.
input_shape = input_details[0]['shape']
print(input_details)
interpreter.set_tensor(input_details[0]['index'], picture)

interpreter.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
output_data = interpreter.get_tensor(output_details[0]['index'])
print(output_data.shape)
i = 0
for output in output_data:
    for mask in output:
        cv2.imwrite(f"mask{i}.png", cv2.resize(mask * 255, (1024, 1024)))
        i += 1