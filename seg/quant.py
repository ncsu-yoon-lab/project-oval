import tensorflow as tf
converter = tf.lite.TFLiteConverter.from_saved_model("segformer")
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_quant_model = converter.convert()

with open('model.tflite', 'wb') as f:
    f.write(tflite_quant_model)