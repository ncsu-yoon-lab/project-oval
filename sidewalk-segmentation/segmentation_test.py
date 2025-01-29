import torch
import safetensors
from safetensors.torch import load_file
from PIL import Image
import torchvision.transforms as transforms
import numpy as np
import matplotlib.pyplot as plt
from transformers import AutoModelForSemanticSegmentation
from suchir_test import process_image
import cv2

class SegmentationTester:
    def __init__(self, model_path):
        # Load the model from safetensors file
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model_state = load_file(model_path)
        
        # Initialize your model architecture here
        # This is a placeholder - replace with your actual model architecture
        self.model = AutoModelForSemanticSegmentation.from_pretrained("./sidewalk_segmentation_model")  
        self.model.load_state_dict(self.model_state)
        self.model.to(self.device)
        self.model.eval()
        
        # Define image transforms
        self.transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                              std=[0.229, 0.224, 0.225])
        ])

    def preprocess_image(self, image_path):
        """Load and preprocess the input image."""
        image = Image.open(image_path).convert('RGB')
        input_tensor = self.transform(image)
        input_batch = input_tensor.unsqueeze(0).to(self.device)
        return input_batch, image

    def visualize_results(self, original_image, segmentation_mask):
        """Visualize the segmentation results."""
        plt.figure(figsize=(12, 4))
        
        plt.subplot(1, 2, 1)
        plt.imshow(original_image)
        plt.title('Original Image')
        plt.axis('off')
        
        plt.subplot(1, 2, 2)
        plt.imshow(segmentation_mask, cmap='viridis')
        plt.title('Segmentation Mask')
        plt.axis('off')
        
        plt.show()

    @torch.no_grad()
    def test_image(self, image_path):
        """Run segmentation on a test image and visualize results."""
        # Preprocess image
        input_batch, original_image = self.preprocess_image(image_path)

        # Check if input_batch is a dictionary (Hugging Face standard)
        if isinstance(input_batch, dict):
            pixel_values = input_batch["pixel_values"]
        else:
            pixel_values = input_batch  # If it's already a tensor

        # Perform inference
        output = self.model(pixel_values=pixel_values)

        # Access logits (raw model output)
        logits = output.logits  # Shape: [batch_size, num_classes, height, width]

        # Remove the batch dimension
        segmentation_mask = logits.squeeze(0).cpu().numpy()  # Shape: [num_classes, height, width]

        # If multi-class segmentation, get the most likely class per pixel
        if len(segmentation_mask.shape) > 2:
            segmentation_mask = np.argmax(segmentation_mask, axis=0)  # Shape: [height, width]

        # Visualize results
        self.visualize_results(original_image, segmentation_mask)

        np.savetxt('output.txt', segmentation_mask, fmt='%d')

        return segmentation_mask

def main():
    # Example usage
    model_path = "./sidewalk_segmentation_model/model.safetensors"
    test_image_path = "test_image.jpg"
    
    # Initialize tester
    tester = SegmentationTester(model_path)
    
    # Run test
    segmentation_mask = tester.test_image(test_image_path)
    
    print("Segmentation complete!")
    print(f"Mask shape: {segmentation_mask.shape}")
    print(f"Unique classes found: {np.unique(segmentation_mask)}")

    output, edges = process_image(segmentation_mask)
    tester.visualize_results(output, edges)
    # print(output)

if __name__ == "__main__":
    main()