from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import ByteMultiArray
import numpy as np
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image as PIL_Image
import time
from dataclasses import dataclass

@dataclass
class VisualFeatureExtractorNode:
    class AlexNetConv3(nn.Module):
        def __init__(self):
            super().__init__()
            original_model = models.alexnet(weights=models.AlexNet_Weights.DEFAULT)
            self.features = nn.Sequential(
                *list(original_model.features.children())[:7]
            )
        def forward(self, x):
            x = self.features(x)
            return x

    def __init__(self, frame_stride=1, crop_image=False, image_crop_x_min=40, image_crop_x_max=260, 
                 image_crop_y_min=0, image_crop_y_max=224):
        #Parameters
        self.frame_stride = frame_stride
        self.crop_image = 1 if crop_image < 1 else crop_image
        self.crop_width_start = image_crop_x_min
        self.crop_width_end = image_crop_x_max
        self.crop_height_start = image_crop_y_min
        self.crop_height_end = image_crop_y_max
        self.image_filter = 'none'
        
        self.counter = 0
        self.bridge = CvBridge()
        self.model = self.AlexNetConv3()
        self.model.eval()
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

        # Random matrix loading moved to C++ wrapper
        print("Visual Feature Extractor Node started")

    
    def image_callback_wrapper(self, img_bytes, width, height, encoding):
        if (self.counter % self.frame_stride != 0):
            self.counter += 1
            return
        
        img_np = np.frombuffer(img_bytes, dtype=np.uint8)

        # Verify if array length to determine the format
        expected_grayscale_size = height * width
        expected_rgb_size = height * width * 3

        if img_np.size == expected_grayscale_size:
            # GrayScale
            img_np = img_np.reshape((height, width))
            # print(f"✓ Imagem em grayscale: {img_np.shape}")
            
        elif img_np.size == expected_rgb_size:
            
            img_rgb = img_np.reshape((height, width, 3))
            # Converte RGB to grayscale
            img_np = np.dot(img_rgb[..., :3], [0.2989, 0.5870, 0.1140]).astype(np.uint8)
            # print(f"✓ Imagem RGB convertida para grayscale: {img_np.shape}")
            
        else:
            raise ValueError(
                f"Tamanho de imagem não esperado: {img_np.size}. "
                f"Esperado: {expected_grayscale_size} (grayscale) ou "
                f"{expected_rgb_size} (RGB). Shape alvo: ({height}, {width})"
            )
        img_msg = Image()
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = 'mono8'
        img_msg.data = img_np.tobytes()
        img_msg.step = width
        features_vector = self.image_callback(img_msg)
        return features_vector
    
    def image_callback_wrapper_compressed(self, img_bytes, format):
        """
        Wrapper for compressed images (JPEG, PNG, etc.)
        Decompresses the image and processes it through the vision pipeline.
        
        Args:
            img_bytes: Compressed image data as bytes
            format: Image format (e.g., 'jpeg', 'png')
        
        Returns:
            Feature vector as numpy array (64896 features)
        """
        if (self.counter % self.frame_stride != 0):
            self.counter += 1
            return
        
        # Decode compressed image using OpenCV
        img_np = np.frombuffer(img_bytes, dtype=np.uint8)
        img_origin = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
        
        if img_origin is None:
            raise ValueError(f"Failed to decode compressed image with format: {format}")
        
        # print(f"✓ Decoded compressed image ({format}): {img_origin.shape}")
        
        # Create a standard Image message to reuse existing processing pipeline
        # Convert to grayscale
        img_gray = cv2.cvtColor(img_origin, cv2.COLOR_BGR2GRAY)
        
        img_msg = Image()
        img_msg.height = img_gray.shape[0]
        img_msg.width = img_gray.shape[1]
        img_msg.encoding = 'mono8'
        img_msg.data = img_gray.tobytes()
        img_msg.step = img_msg.width
        
        features_vector = self.image_callback(img_msg)
        return features_vector
        
    def image_callback(self, img_msg: Image) -> ByteMultiArray:
        start_total = time.time()
        
        img_origin = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
        if self.crop_image:
            img_origin = img_origin[self.crop_width_start:self.crop_width_end,
                                    self.crop_height_start:self.crop_height_end]
        
        if self.image_filter == 'gauss':
            imga = cv2.cvtColor(img_origin, cv2.COLOR_BGR2RGB)
            imga = cv2.GaussianBlur(imga, (5,5), 0)
        elif self.image_filter == 'clahe':
            hsv_img = cv2.cvtColor(img_origin, cv2.COLOR_BGR2HSV)
            h, s, v = hsv_img[:,:,0], hsv_img[:,:,1], hsv_img[:,:,2]
            v = self.clahe.apply(v)
            hsv_img = np.dstack((h,s,v))
            imga = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)
        else:
            imga = cv2.cvtColor(img_origin, cv2.COLOR_BGR2RGB)
        
        preprocess = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        img = PIL_Image.fromarray(imga)
        input_tensor = preprocess(img)
        input_batch = input_tensor.unsqueeze(0)
        
        if torch.cuda.is_available():
            input_batch = input_batch.to('cuda')
            self.model.to('cuda')
        
        # Timing: Neural network inference
        t_inference_start = time.time()
        with torch.no_grad():
            output_ = self.model(input_batch)
        output = output_.cpu()
        
        output_flatten = output[0].numpy().flatten('C')
        features_vector = output_flatten  # Return as 1D array (64896 features)
        
        # LSBH computation moved to C++ for better performance
        
        end_total = time.time()
        time_exec = end_total - start_total

        # print(f"[VisualFeatureExtractor] Python processing time for image {self.counter}: {time_exec*1000:.2f} ms ({time_exec:.4f} seconds)")
        # print(f"[VisualFeatureExtractor] Returning feature vector of size: {len(features_vector)}")
        
        return features_vector