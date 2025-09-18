#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time
import onnxruntime as ort
import numpy as np
from std_msgs.msg import Int64

class ONNXSegmentationModel:
    def __init__(self, model_path, device="cuda", use_tensorrt=False):
        self.model_path = model_path 
        self.device = device 
        
        # Configure session options
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 4 
        session_options.inter_op_num_threads = 1
        session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        # Configure providers
        available_providers = ort.get_available_providers()
        print("Available providers:", available_providers)
        
        if device == "cuda":
            providers = []  
            
            # Only use TensorRT if explicitly requested (avoids hanging)
            if use_tensorrt and 'TensorrtExecutionProvider' in available_providers:
                print("Using TensorRT provider - first run may take several minutes")
                trt_options = {
                    'device_id': 0,
                    'trt_max_workspace_size': 1 << 30,  # 1GB
                    'trt_fp16_enable': True,
                    'trt_engine_cache_enable': True,
                    'trt_engine_cache_path': './trt_cache',
                }
                providers.append(('TensorrtExecutionProvider', trt_options))
            
            # CUDA provider
            if 'CUDAExecutionProvider' in available_providers:
                cuda_options = {
                    'device_id': 0,
                    'arena_extend_strategy': 'kSameAsRequested',
                    'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # 2GB limit
                }
                providers.append(('CUDAExecutionProvider', cuda_options))
            
            providers.append('CPUExecutionProvider')
        else:
            providers = ['CPUExecutionProvider']
        
        print(f"Using providers: {[p[0] if isinstance(p, tuple) else p for p in providers]}")
        
        # Create session
        try:
            self.session = ort.InferenceSession(
                model_path, 
                sess_options=session_options,
                providers=providers
            )
            print(f"Model loaded successfully: {model_path}")
            print(f"Active providers: {self.session.get_providers()}")
        except Exception as e:
            print(f"Failed to load with optimized providers: {e}")
            # Fallback to basic CUDA
            self.session = ort.InferenceSession(
                model_path,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )
            print("Using fallback CUDA provider")
        
        # Get model input/output info
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        print(f"Input name: {self.input_name}")
        print(f"Output name: {self.output_name}")
        
    def preprocess_image(self, image):
        """Convert image to (1, 3, 672, 672) format for ONNX model"""
        resized = cv2.resize(image, (672, 672))
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb_image.astype(np.float32) / 255.0
        # Convert to CHW format and add batch dimension
        input_tensor = np.transpose(normalized, (2, 0, 1))  # HWC to CHW
        input_tensor = np.expand_dims(input_tensor, axis=0)  # Add batch dimension
        return input_tensor
    
    def postprocess_output(self, output):
        """Postprocess model output to create segmentation mask"""
        output = np.squeeze(output, axis=0)  # [num_classes, 672, 672]
        exp_output = np.exp(output - np.max(output, axis=0, keepdims=True))
        probabilities = exp_output / np.sum(exp_output, axis=0, keepdims=True)
        segmentation_mask = np.argmax(probabilities, axis=0)  # [672, 672]
        mask_viz = (segmentation_mask * 255).astype(np.uint8)
        return mask_viz, probabilities
    
    def segment_image(self, image):
        """Perform segmentation on input image"""
        input_tensor = self.preprocess_image(image)
        output = self.session.run([self.output_name], {self.input_name: input_tensor})[0]
        mask, probabilities = self.postprocess_output(output)
        return mask, probabilities

class SegmentationNode(Node):
    def __init__(self, model_path, image_topic, device="cuda", use_tensorrt=False, 
                 frequency=10, show_display=True, publish_segmentation=True):
        super().__init__("onnx_segmentation_node")
        
        self.model_path = model_path 
        self.image_topic = image_topic 
        self.device = device 
        self.frequency = frequency
        self.publish_segmentation = publish_segmentation
        self.show_display = show_display
        
        # Model Initialization 
        try:
            self.seg_model = ONNXSegmentationModel(
                self.model_path, 
                self.device,
                use_tensorrt=use_tensorrt
            )
            self.get_logger().info(f"Model loaded successfully from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {str(e)}")
            return
        
        self.br = CvBridge()
        self.current_image = None 
        self.image_lock = threading.Lock()
        
        # Creating subscriber 
        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.camera_callback,
            10
        )
        
        # Create segmentation publisher
        if self.publish_segmentation:
            self.segmentation_publisher = self.create_publisher(
                Image,
                '/segmentation_mask',
                10
            )
        
        self.timer = self.create_timer(1.0 / self.frequency, self.process_timer_callback)
        
        self.get_logger().info(f"Segmentation node initialized")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        if self.publish_segmentation:
            self.get_logger().info(f"Publishing to: /segmentation_mask")
        
    def camera_callback(self, msg):
        try:
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {str(e)}")
    
    def process_timer_callback(self):
        with self.image_lock:
            if self.current_image is None:
                self.get_logger().warn("No image received yet", throttle_duration_sec=5.0)
                return
            image_to_process = self.current_image.copy()
        
        try:
            start_time = time.time()
            segmentation_mask, probabilities = self.seg_model.segment_image(image_to_process)
            processing_time = time.time() - start_time
            
            self.get_logger().info(f"Segmentation completed in {processing_time:.3f}s")
            
            if self.show_display:
                self.display_results(image_to_process, segmentation_mask, probabilities)
            
            if self.publish_segmentation:
                self.publish_segmentation_mask(segmentation_mask)
                
        except Exception as e:
            self.get_logger().error(f"Error in processing: {str(e)}")
    
    def display_results(self, original_image, segmentation_mask, probabilities):
        """FIXED: Display results when called (removed wrong early return)"""
        try:
            original_resized = cv2.resize(original_image, (672, 672))
            colored_mask = cv2.applyColorMap(segmentation_mask, cv2.COLORMAP_JET)
            alpha = 0.6
            overlay = cv2.addWeighted(original_resized, 1-alpha, colored_mask, alpha, 0)

            display_image = np.hstack([original_resized, colored_mask, overlay])
            
            cv2.putText(display_image, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(display_image, "Segmentation", (682, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(display_image, "Overlay", (1354, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            cv2.imshow("Segmentation Results", display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in display: {str(e)}")
    
    def publish_segmentation_mask(self, segmentation_mask):
        try:
            mask_msg = self.br.cv2_to_imgmsg(segmentation_mask, "mono8")
            mask_msg.header.stamp = self.get_clock().now().to_msg()
            mask_msg.header.frame_id = "camera_frame"
            self.segmentation_publisher.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing segmentation mask: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SegmentationNode(
            model_path="/home/wolfwagen1/ros2_ws/src/project-oval/project-oval/models/dino_segmentation.onnx",  
            image_topic="/zed/zed_node/left/image_rect_color",
            device="cuda",
            use_tensorrt=False,
            frequency=10.0,
            show_display=True,
            publish_segmentation=True
        )
        
        # Check if model was loaded successfully
        if hasattr(node, 'seg_model'):
            node.get_logger().info("Starting segmentation node...")
            rclpy.spin(node)
        else:
            node.get_logger().error("Failed to initialize segmentation node")
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()