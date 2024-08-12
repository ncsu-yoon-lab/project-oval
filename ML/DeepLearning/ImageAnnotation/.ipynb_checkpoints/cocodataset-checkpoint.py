import os
import torch
from torchvision.datasets import CocoDetection
import torchvision.transforms as T
from PIL import Image


class Compose(object):
    def __init__(self, transforms):
        self.transforms = transforms

    def __call__(self, image, target):
        for t in self.transforms:
            image, target = t(image, target)
        return image, target

class ToTensor(object):
    def __call__(self, image, target):
        if isinstance(image, Image.Image):
            image = T.ToTensor()(image)
        return image, target


class CocoDataset(CocoDetection):
    def __init__(self, img_folder, ann_file, transforms=None):
        super(CocoDataset, self).__init__(img_folder, ann_file)
        self.transforms = transforms

    def _load_image(self, id):
        path = self.coco.loadImgs(id)[0]['file_name']
        full_path = os.path.join(self.root, path)
        try:
            img = Image.open(full_path).convert("RGB")
            img.verify()  # Verify the image file
            img = Image.open(full_path).convert("RGB")  # Reopen the image to use it
        except (FileNotFoundError, IOError, SyntaxError) as e:
            print(f"Error opening or verifying image {full_path}: {e}")
            return None
        return img

    def __getitem__(self, idx):
        id = self.ids[idx]
        img = self._load_image(id)

        if img is None:
            return self.__getitem__((idx + 1) % len(self))  # Try the next index

        target = self.coco.loadAnns(self.coco.getAnnIds(imgIds=id))
        
        boxes = []
        labels = []
        for obj in target:
            bbox = obj['bbox']
            # Convert bbox from (x, y, width, height) to (x1, y1, x2, y2)
            x1, y1, width, height = bbox
            x2 = x1 + width
            y2 = y1 + height
            boxes.append([x1, y1, x2, y2])
            labels.append(obj['category_id'])

        # If there are no bounding boxes, skip this sample
        if len(boxes) == 0:
            print(f"Warning: No bounding boxes found for idx {idx}. Skipping...")
            return self.__getitem__((idx + 1) % len(self))  # Try the next index
        
        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = torch.as_tensor(labels, dtype=torch.int64)
        image_id = torch.tensor([idx])

        target = {"boxes": boxes, "labels": labels, "image_id": image_id}
        
        if self.transforms is not None:
            img, target = self.transforms(img, target)
        
        return img, target