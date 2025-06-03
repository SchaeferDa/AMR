#! /usr/bin/env python

import torch, os, time, cv2, numpy
from ultralytics import YOLO
from torchvision.transforms import functional as F

torch.backends.cudnn.enabled = False

# Model loading
model = YOLO('./models/model-v2.pt')  # Can be 'yolov5n' - 'yolov5x6', or 'custom'

# Inference on images
folder_path = "./datasets/labeld1/not_used/images"  # Can be a file, Path, PIL, OpenCV, numpy, or list of images
files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

while True:
    try:
        for file_name in files:
            file = os.path.join(folder_path, file_name)
            results = model(file)
            img = results[0].plot()
            cv2.imshow("Live view", img)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        break

# Display results
#results[0].show()  # Other options: .show(), .save(), .crop(), .pandas(), etc. Explore these in the Predict mode documentation.

cv2.destroyAllWindows()