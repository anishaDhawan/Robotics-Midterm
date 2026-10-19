"""Load the three YOLO models and read their boxes, body joints, and masks.
The README links to the documentation. ONNX is the saved model format.
"""
from pathlib import Path
import cv2
import numpy as np
import onnxruntime as ort

# These numbers identify the shoulders, elbows, wrists, hips, knees, and ankles.
BODY_POINTS = {5: 'left_shoulder', 6: 'right_shoulder', 7: 'left_elbow', 8: 'right_elbow',
               9: 'left_wrist', 10: 'right_wrist', 11: 'left_hip', 12: 'right_hip',
               13: 'left_knee', 14: 'right_knee', 15: 'left_ankle', 16: 'right_ankle'}

class PoseDepth:
    """Find body joints and a box around each detected person."""

    def __init__(self, model_path=None):
        # Load the model on the CPU and limit how many workers it uses.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        path = model_path or Path.home() / 'human_model/yolov8n-pose.onnx'
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        self.size = int(self.input.shape[-1])
        cv2.setNumThreads(2)

    def predict(self, image, details=False):
        (h, w) = image.shape[:2]
        scale = min(self.size / w, self.size / h)
        (nw, nh) = (round(w * scale), round(h * scale))
        (left, top) = ((self.size - nw) // 2, (self.size - nh) // 2)
        # Resize the image and add a border so the person is not stretched.
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(image, (nw, nh))
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        # Arrange the colours for the model and change pixel values from 0-255 to 0-1.
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        # Run YOLO on the prepared image.
        output = self.session.run(None, {self.input.name: tensor})[0]
        if output.shape[1] != 56:
            raise ValueError(f'Unexpected pose output: {output.shape}')
        rows = output[0].T
        rows = rows[np.isfinite(rows).all(axis=1) & (rows[:, 4] >= 0.5)]
        (boxes, poses, scores) = ([], [], [])
        for row in rows:
            (cx, cy, bw, bh) = row[:4]
            boxes.append([float((cx - bw / 2 - left) / scale), float((cy - bh / 2 - top) / scale), float(bw / scale), float(bh / scale)])
            # Each of the 17 body points has an image position and a confidence score.
            points = row[5:].reshape(17, 3).copy()
            points[:, 0] = (points[:, 0] - left) / scale
            points[:, 1] = (points[:, 1] - top) / scale
            poses.append(points)
            scores.append(float(row[4]))
        # Keep the stronger box when overlapping boxes appear to describe the same person.
        keep = np.asarray(cv2.dnn.NMSBoxes(boxes, scores, 0.5, 0.45)).reshape(-1)
        if details:
            return [{'points': poses[int(i)], 'box': boxes[int(i)], 'score': scores[int(i)]} for i in keep]
        return [poses[int(i)] for i in keep]

class SegmentationDepth:
    """Find which image pixels YOLO predicts belong to each person (the mask)."""

    def __init__(self, model_path=None):
        # Load the model on the CPU and limit how many workers it uses.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        path = model_path or Path.home() / 'human_model/yolov8n-seg.onnx'
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        self.size = int(self.input.shape[-1])
        cv2.setNumThreads(2)

    def predict(self, image):
        (h, w) = image.shape[:2]
        scale = min(self.size / w, self.size / h)
        (nw, nh) = (round(w * scale), round(h * scale))
        (left, top) = ((self.size - nw) // 2, (self.size - nh) // 2)
        # Resize the image and add a border so the person is not stretched.
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(image, (nw, nh))
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        # Arrange the colours for the model and change pixel values from 0-255 to 0-1.
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        # Run YOLO to get person predictions and the pieces used to build masks.
        outputs = self.session.run(None, {self.input.name: tensor})
        predictions = next((o for o in outputs if o.ndim == 3))
        prototypes = next((o for o in outputs if o.ndim == 4))[0]
        (channels, ph, pw) = prototypes.shape
        if predictions.shape[1] != 4 + 80 + channels:
            raise ValueError('Unexpected segmentation output shape')
        rows = predictions[0].T
        rows = rows[np.isfinite(rows).all(axis=1)]
        rows = rows[(rows[:, 4:84].argmax(axis=1) == 0) & (rows[:, 4] >= 0.5)]
        boxes = []
        scores = []
        coefficients = []
        for row in rows:
            (cx, cy, bw, bh) = row[:4]
            boxes.append([float((cx - bw / 2 - left) / scale), float((cy - bh / 2 - top) / scale), float(bw / scale), float(bh / scale)])
            scores.append(float(row[4]))
            coefficients.append(row[84:])
        # Keep the stronger box when overlapping boxes appear to describe the same person.
        keep = np.asarray(cv2.dnn.NMSBoxes(boxes, scores, 0.5, 0.45)).reshape(-1)
        results = []
        for index in keep:
            index = int(index)
            # Combine YOLO's mask pieces to make this person's mask, then remove the added border.
            logits = (coefficients[index] @ prototypes.reshape(channels, -1)).reshape(ph, pw)
            square = cv2.resize(logits, (self.size, self.size), interpolation=cv2.INTER_LINEAR)
            unpadded = square[top:top + nh, left:left + nw]
            mask = cv2.resize(unpadded, (w, h), interpolation=cv2.INTER_LINEAR) > 0
            (x, y, bw, bh) = boxes[index]
            columns = np.arange(w)[None, :]
            rows_y = np.arange(h)[:, None]
            # Keep mask pixels only inside this person's box.
            mask &= (columns >= x) & (columns < x + bw) & (rows_y >= y) & (rows_y < y + bh)
            results.append({'mask': mask, 'score': scores[index], 'box': boxes[index]})
        return results

class YoloDetector:
    """Find a person box when body joints and masks cannot give us a usable point."""

    def __init__(self, path, confidence=0.4, class_id=0):
        # Load the model on the CPU and limit how many workers it uses.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]
        self.size = int(self.input.shape[-1])
        self.confidence = confidence
        self.class_id = class_id

    def detect(self, frame):
        (height, width) = frame.shape[:2]
        scale = min(self.size / width, self.size / height)
        (resized_width, resized_height) = (round(width * scale), round(height * scale))
        left = (self.size - resized_width) // 2
        top = (self.size - resized_height) // 2
        # Resize the image and add a border so the person is not stretched.
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[top:top + resized_height, left:left + resized_width] = cv2.resize(frame, (resized_width, resized_height))
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        # Arrange the colours for the model and change pixel values from 0-255 to 0-1.
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        # Run YOLO to get possible object boxes and their scores.
        predictions = self.session.run(None, {self.input.name: tensor})[0]
        if predictions.ndim != 3 or predictions.shape[1] != 84:
            raise ValueError(f'Expected raw COCO YOLOv8 output [1,84,N], got {predictions.shape}')
        (boxes, scores) = ([], [])
        for row in predictions[0].T:
            # Keep the requested class (person by default) when its score is high enough.
            classes = row[4:]
            if int(np.argmax(classes)) != self.class_id or float(classes[self.class_id]) < self.confidence:
                continue
            (cx, cy, bw, bh) = row[:4]
            x1 = max(0.0, min(width, float((cx - bw / 2 - left) / scale)))
            y1 = max(0.0, min(height, float((cy - bh / 2 - top) / scale)))
            x2 = max(0.0, min(width, float((cx + bw / 2 - left) / scale)))
            y2 = max(0.0, min(height, float((cy + bh / 2 - top) / scale)))
            if x2 > x1 and y2 > y1:
                boxes.append([x1, y1, x2 - x1, y2 - y1])
                scores.append(float(classes[self.class_id]))
        return (boxes, scores)
