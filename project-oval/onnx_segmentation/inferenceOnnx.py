#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time
import numpy as np
from std_msgs.msg import Int64, Float32
import onnxruntime as ort
from PIL import Image as PILImage

IMAGE_TOPIC = "/zed/zed_node/stereo/image_rect_color"

# ONNX model path - update this to your model
MODEL_PATH = "../sidewalk_segmentation/sidewalk_segmentation_model/model.onnx"

frame_count = 0
start_time = None
inference_times = []

br = CvBridge()
image = None
onnx_session = None

def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]  

def preprocess_image(img):
    """
    Preprocess image for ONNX model inference
    Adjust this based on your model's requirements
    """
    img_resized = cv2.resize(img, (512, 512))
    
    img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
    
    pil_img = PILImage.fromarray(img_rgb)
    img_array = np.array(pil_img).astype(np.float32)
    
    img_array = img_array / 255.0
    
    img_array = img_array.transpose(2, 0, 1)
    img_array = np.expand_dims(img_array, axis=0) 
    return img_array

def run_inference(img):
    """
    Run ONNX model inference and measure time
    """
    global frame_count, start_time, inference_times, onnx_session
    
    if onnx_session is None:
        print("ONNX session not initialized!")
        return None
    
    inference_start = time.time()
    
    processed_img = preprocess_image(img)
    
    try:
        outputs = onnx_session.run(None, {'pixel_values': processed_img})
        
        if len(outputs) > 0:
            result = outputs[0]
            if len(result.shape) > 2:
                # For segmentation models
                mask = np.argmax(result.squeeze(0), axis=0)
            else:
                mask = result
        else:
            mask = None
            
    except Exception as e:
        print(f"Inference error: {e}")
        return None
    
    inference_end = time.time()
    inference_time = inference_end - inference_start
    inference_times.append(inference_time)
    
    frame_count += 1
    
    if start_time is None:
        start_time = time.time()
    
    if frame_count % 30 == 0:
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time
        avg_inference_time = np.mean(inference_times[-30:]) * 1000  # Convert to ms
        
        print(f"Frames processed: {frame_count}")
        print(f"Average FPS: {fps:.2f}")
        print(f"Average inference time: {avg_inference_time:.2f} ms")
        print(f"Frames per minute: {fps * 60:.0f}")
        print("-" * 40)
    
    return mask

def visualize_result(original_img, mask):
    """
    Visualize the inference result
    """
    if mask is not None:
        mask_resized = cv2.resize(mask.astype(np.uint8), 
                                (original_img.shape[1], original_img.shape[0]), 
                                interpolation=cv2.INTER_NEAREST)
        
        colored_mask = cv2.applyColorMap(mask_resized * 20, cv2.COLORMAP_JET)
        
        overlay = cv2.addWeighted(original_img, 0.7, colored_mask, 0.3, 0)
        
        cv2.imshow('ONNX Inference Result', overlay)
        cv2.imshow('Original', original_img)
        cv2.waitKey(1)

def main():
    global onnx_session
    
    rclpy.init()
    node = rclpy.create_node('onnx_inference_node')
    
    try:
        
        available_providers = ort.get_available_providers()
        providers = []
        if 'CUDAExecutionProvider' in available_providers:
            providers.append(('CUDAExecutionProvider', {
                'device_id': 0,
                'arena_extend_strategy': 'kNextPowerOfTwo',
                'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # 2GB limit
                'cudnn_conv_algo_search': 'EXHAUSTIVE',
                'do_copy_in_default_stream': True,
            }))
            print("CUDA provider configured")
        else:
            print("WARNING: CUDA provider not available!")
        
        providers.append('CPUExecutionProvider')
        
        onnx_session = ort.InferenceSession(MODEL_PATH, providers=providers)
        
        session_providers = onnx_session.get_providers()
    
        
        input_names = [input.name for input in onnx_session.get_inputs()]
        output_names = [output.name for output in onnx_session.get_outputs()]
        print(f"Model inputs: {input_names}")
        print(f"Model outputs: {output_names}")
        
    except Exception as e:
        print(f"Failed to load ONNX model: {e}")
        return
    
    # Create subscription to camera topic
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
    # Publishers for performance metrics
    fps_pub = node.create_publisher(Float32, "onnx_fps", 10)
    inference_time_pub = node.create_publisher(Float32, "onnx_inference_time", 10)
    frame_count_pub = node.create_publisher(Int64, "onnx_frame_count", 10)
    
    # Start ROS2 spinning in separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    FREQ = 30 
    rate = node.create_rate(FREQ, node.get_clock())
    
    while rclpy.ok() and image is None:
        print("Waiting for image topic...")
        rate.sleep()
    
    print("Starting ONNX inference performance test...")
    print("Press Ctrl+C to stop and see final results")
    
    try:
        while rclpy.ok():
            if image is not None:
                result = run_inference(image)
                
                visualize_result(image, result)
                
                if frame_count > 0 and start_time is not None:
                    elapsed_time = time.time() - start_time
                    current_fps = frame_count / elapsed_time
                    
                    fps_msg = Float32()
                    fps_msg.data = current_fps
                    fps_pub.publish(fps_msg)
                    
                    if len(inference_times) > 0:
                        avg_inference_time = np.mean(inference_times[-10:]) * 1000
                        inference_msg = Float32()
                        inference_msg.data = avg_inference_time
                        inference_time_pub.publish(inference_msg)
                    
                    frame_msg = Int64()
                    frame_msg.data = frame_count
                    frame_count_pub.publish(frame_msg)
            
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\nStopping inference test...")
    
    if start_time is not None and frame_count > 0:
        total_time = time.time() - start_time
        final_fps = frame_count / total_time
        frames_per_minute = final_fps * 60
        avg_inference_time = np.mean(inference_times) * 1000
        
        print("\n" + "="*50)
        print("FINAL PERFORMANCE REPORT")
        print("="*50)
        print(f"Total frames processed: {frame_count}")
        print(f"Total time: {total_time:.2f} seconds")
        print(f"Average FPS: {final_fps:.2f}")
        print(f"Frames per minute: {frames_per_minute:.0f}")
        print(f"Average inference time: {avg_inference_time:.2f} ms")
        print(f"Min inference time: {np.min(inference_times)*1000:.2f} ms")
        print(f"Max inference time: {np.max(inference_times)*1000:.2f} ms")
        print("="*50)
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()