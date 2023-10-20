import torch
from transformers import SegformerForSemanticSegmentation

# setattr(torch.distributed, "is_initialized", lambda : False)
print(torch.backends.quantized.supported_engines)
# torch.backends.quantized.engine = 'qnnpack'

model = SegformerForSemanticSegmentation.from_pretrained("nvidia/segformer-b0-finetuned-cityscapes-1024-1024")

model_quant = torch.ao.quantization.quantize_dynamic(model)

torch.save(model_quant.state_dict(), "pt_b0_quant.pt")
