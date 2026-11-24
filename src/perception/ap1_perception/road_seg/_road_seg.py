"""Road segmentation (YOLO26n-sem) on ONNX Runtime.

Runs an Ultralytics-exported semantic segmentation model without depending on
ultralytics or torch. The caller passes the model path (the node's default is
road_seg_node.default_road_seg_model). The export bakes ArgMax into the graph,
so the model outputs a (1, H, W) uint8 class map: one class ID per pixel.

Class IDs and names come from the model's metadata. The pretrained model uses
the 19 Cityscapes classes: 0 = road, 1 = sidewalk, ...

Preprocessing mirrors ultralytics' predict defaults (letterbox with grey 114,
RGB, /255), so class maps match `ultralytics.YOLO(model).predict(img)`.
"""
import ast

import cv2
import numpy as np

from ..runtime import create_session

# Letterbox fill value ultralytics uses during training and prediction
_PAD_VALUE = 114


class RoadSeg:
    def __init__(self, model_path, device="auto", threads=4):
        self.session = create_session(model_path, device, threads)

        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        # Fixed-size export: (1, 3, 448, 768) for the 1920x1080 camera
        self.input_h, self.input_w = model_input.shape[2:]

        # ultralytics stores the class names in the ONNX metadata as a dict literal
        meta = self.session.get_modelmeta().custom_metadata_map
        self.names = ast.literal_eval(meta['names']) if 'names' in meta else {}

    @property
    def provider(self) -> str:
        """The execution provider inference actually runs on."""
        return self.session.get_providers()[0]

    def __call__(self, rgb):
        """Segment an RGB uint8 image.

        Returns a uint8 class map at the model's resolution with the padding removed
        (432x768 for a 1920x1080 image).
        """
        blob, (top, left, h, w) = self._preprocess(rgb)
        class_map = self.session.run(None, {self.input_name: blob})[0][0]
        return np.ascontiguousarray(class_map[top:top + h, left:left + w])

    def _preprocess(self, rgb):
        """Letterbox to the model input size; returns the NCHW blob and the (top, left, h, w) of the image in it."""
        h, w = rgb.shape[:2]
        r = min(self.input_h / h, self.input_w / w)
        new_w, new_h = max(round(w * r), 1), max(round(h * r), 1)
        top = (self.input_h - new_h) // 2
        left = (self.input_w - new_w) // 2

        if (w, h) != (new_w, new_h):
            rgb = cv2.resize(rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        img = np.full((self.input_h, self.input_w, 3), _PAD_VALUE, dtype=np.uint8)
        img[top:top + new_h, left:left + new_w] = rgb

        blob = np.ascontiguousarray(img.transpose(2, 0, 1)[np.newaxis], dtype=np.float32) / 255.0
        return blob, (top, left, new_h, new_w)
