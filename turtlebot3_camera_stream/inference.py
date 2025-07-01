import torch, os, time, cv2, numpy
from ultralytics import YOLO
from torchvision.transforms import functional as F



class Inference:
    def init(self, model):
        torch.backends.cudnn.enabled = False
        self.cuda = torch.cuda.is_available()

        # Model loading
        self.model = YOLO(model)

    def inference(self, x):
        results = self.model.predict(x, conf=0.7, imgsz=(640,480), verbose = False)
        if cuda:
                return results[0].cpu()
        else:
                return results[0]
