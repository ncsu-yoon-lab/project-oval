#%%
import onnxruntime as ort
model_path="/home/wolfwagen1/ros2_ws/src/project-oval/project-oval/models/dino_segmentation_2.onnx" 
device = "cuda"
session_options = ort.SessionOptions()
session_options.intra_op_num_threads = 4 
session_options.inter_op_num_threads = 1
session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
# import torch
# torch.ops.load_library(torch.ops.__file__)

#%% Load the ONNX model for segmentation
available_providers = ort.get_available_providers()
print("Available providers:", available_providers)
use_tensorrt=False
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

#%% Load the object detection model 
object_detection_model_path = "/home/wolfwagen1/ros2_ws/src/project-oval/project-oval/models/qloc_object_detection_model_2.onnx"
odm_onnx = ort.InferenceSession(
                object_detection_model_path,
                sess_options=session_options,
                providers=providers
)

#%%
model_onnx = ort.InferenceSession(
                model_path,
                sess_options=session_options,
                providers=providers
            )

# %%
import torch
import numpy as np
import os

# Load the folder 
folder_path = '/media/wolfwagen1/c9c2a9fe-c435-4115-9237-57bc783cf9641/qlocImage'

images_file = []
files = os.listdir(folder_path)
for file in files:
    if file.endswith('.png') or file.endswith('.jpg'):
        images_file.append(os.path.join(folder_path, file))

print(f"Found {len(images_file)} images in {folder_path}")

#%% Load and preprocess images
import numpy as np
import cv2

image_res = 224
test_image_path = images_file[0]
image = cv2.imread(test_image_path)  # Load image from file
resized = cv2.resize(image, (image_res, image_res))
rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
normalized = rgb_image.astype(np.float32) / 255.0
input_tensor = np.transpose(normalized, (2, 0, 1))  # HWC to CHW
input_tensor = np.expand_dims(input_tensor, axis=0)  # Add batch dimension

output_name = model_onnx.get_outputs()[0].name
input_name = model_onnx.get_inputs()[0].name

output = model_onnx.run([output_name], {input_name: input_tensor})[0]
output = np.squeeze(output, axis=0)  # [num_classes, image_res, image_res]
exp_output = np.exp(output - np.max(output, axis=0, keepdims=True))
probabilities = exp_output / np.sum(exp_output, axis=0, keepdims=True)
segmentation_mask = np.argmax(probabilities, axis=0)  # [image_res, image)Res]
mask_viz = (segmentation_mask * 255).astype(np.uint8)

def apply_convex_hull_to_mask(mask):
    # Convert to binary mask (assuming non-zero values are the object)
    binary_mask = (mask > 0).astype(np.uint8)
    
    # Find contours
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return mask  # Return original if no contours found
    
    # Get largest contour (main object)
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Get convex hull
    hull = cv2.convexHull(largest_contour)
    
    # Create new mask with convex hull
    convex_hull_mask = np.zeros_like(mask)
    cv2.fillPoly(convex_hull_mask, [hull], 255)  # Fill with 255 (white)
    
    return convex_hull_mask

convex_hull_mask = apply_convex_hull_to_mask(mask_viz)

def display_results(original_image, segmentation_mask, probabilities):

    original_resized = cv2.resize(original_image, (image_res, image_res))
    colored_mask = cv2.applyColorMap(segmentation_mask, cv2.COLORMAP_JET)
    alpha = 0.6
    overlay = cv2.addWeighted(original_resized, 1-alpha, colored_mask, alpha, 0)

    convex_hull_mask = segmentation_mask
    colored_hull_mask = cv2.applyColorMap(convex_hull_mask, cv2.COLORMAP_JET)
    overlay_hull = cv2.addWeighted(original_resized, 1-alpha, colored_hull_mask, alpha, 0)

    display_image = np.hstack([original_resized, colored_mask, overlay, overlay_hull])
    
    cv2.putText(display_image, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(display_image, "Segmentation", (682, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(display_image, "Overlay", (1354, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(display_image, "Overlay_Hull", (2036, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Resize the display image to have width of 672 pixels
    target_width = 672
    height, width = display_image.shape[:2]
    aspect_ratio = height / width
    target_height = int(target_width * aspect_ratio)
    
    display_image_resized = cv2.resize(display_image, (target_width, target_height))
    
    # Adjust text positions for the resized image
    scale_factor = target_width / width
    cv2.putText(display_image_resized, "Original", 
            (int(10 * scale_factor), int(30 * scale_factor)), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.4 * scale_factor, (255, 255, 255), 1)
    cv2.putText(display_image_resized, "Segmentation", 
            (int(682 * scale_factor), int(30 * scale_factor)), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.4 * scale_factor, (255, 255, 255), 1)
    cv2.putText(display_image_resized, "Overlay", 
            (int(1354 * scale_factor), int(30 * scale_factor)), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.4 * scale_factor, (255, 255, 255), 1)
    cv2.putText(display_image_resized, "Overlay_Hull", 
            (int(2036 * scale_factor), int(30 * scale_factor)), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.4 * scale_factor, (255, 255, 255), 1)
    
    cv2.imwrite("segmentation_results.png", display_image_resized)

display_results(image, mask_viz, probabilities)

#%%
# Create random input
random_input = torch.randn(1, 3, 224, 224)

# Convert PyTorch tensor to numpy (ONNX Runtime expects numpy arrays)
input_numpy = random_input.numpy()

input_info = model_onnx.get_inputs()[0]
print(f"Input info: {input_info}")

output = model_onnx.run(None, {input_name: input_numpy})

input_info = model_onnx.get_inputs()[0] 
input_name = input_info.name 
print(f"Input info: {input_info}")
print(f"Input name: {input_name}")

output = model_onnx.run(None, {input_name: input_numpy})

# Output is a list, get the first (and likely only) output
output_array = output[0]
print(f"Output shape: {output_array.shape}")
print(f"Output type: {type(output_array)}")

# If you want to convert back to PyTorch tensor
output_tensor = torch.from_numpy(output_array)
print(f"Output tensor shape: {output_tensor.shape}")
# %%
