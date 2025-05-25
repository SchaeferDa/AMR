#! /usr/bin/env python

import torch
from ultralytics import YOLO

# Model loading
model = YOLO('./runs/detect/train/weights/best.pt')  # Can be 'yolov5n' - 'yolov5x6', or 'custom'

# Inference on images
img = "Screenshot1.jpg"  # Can be a file, Path, PIL, OpenCV, numpy, or list of images

# Run inference
results = model(img)

# Display results
results[0].show()  # Other options: .show(), .save(), .crop(), .pandas(), etc. Explore these in the Predict mode documentation.