#%%
import onnxruntime as ort
model_path="/home/wolfwagen1/ros2_ws/src/project-oval/project-oval/models/dino_segmentation_2.onnx" 
device = "cuda"
session_options = ort.SessionOptions()
session_options.intra_op_num_threads = 4 
session_options.inter_op_num_threads = 1
session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
#%%
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
#%%    
model_onnx = ort.InferenceSession(
                model_path,
                sess_options=session_options,
                providers=providers
            )


# %%
import torch
import numpy as np

# Create random input
random_input = torch.randn(1, 3, 224, 224)

# Convert PyTorch tensor to numpy (ONNX Runtime expects numpy arrays)
input_numpy = random_input.numpy()

input_info = model_onnx.get_inputs()[0]
print(f"Input info: {input_info}")


output = model_onnx.run(None, {input_name: input_numpy})

input_info = model_onnx.get_inputs()[0]
input_name = input_info.name  # ✅ Add this line
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
