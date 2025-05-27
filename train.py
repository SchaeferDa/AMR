#! /usr/bin/env python

import torch
from ultralytics import YOLO

def main():
    # Model loading
    model = YOLO('yolo11n.pt')

    results = model.train(data="coco-train.yaml", epochs=1000, imgsz=640)

if __name__ == '__main__':
    main()