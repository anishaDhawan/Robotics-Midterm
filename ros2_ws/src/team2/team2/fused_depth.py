"""Choose one point on the person using body joints, a mask, or a person box.
The limits used help check measurements.
"""
from pathlib import Path
import cv2
import numpy as np
from team2.models import PoseDepth, SegmentationDepth, YoloDetector, BODY_POINTS
from team2.geometry import choose_supported_point, restore_box

def overlap(a, b):
    """Measure box overlap: 0 means no overlap and 1 means identical boxes."""
    (x, y, w, h) = a
    (X, Y, W, H) = b
    area = max(0, min(x + w, X + W) - max(x, X)) * max(0, min(y + h, Y + H) - max(y, Y))
    return area / max(w * h + W * H - area, 1e-09)

def original_pixel(u, v, angle, w, h):
    if angle == 90:
        return (w - 1 - v, u)
    if angle == -90:
        return (v, h - 1 - u)
    return (u, v)

def point(u, v, z, p, name, score):
    """Use a pixel and its depth to find its position in metres relative to the camera."""
    return {'pixel': [int(u), int(v)], 'xyz': [(u - p[2]) * z / p[0], (v - p[6]) * z / p[5], z], 'body_part': name, 'confidence': float(score)}

def surface_points(depth, region, p, score, name):
    """Choose pixels across the area and keep each pixel with its own depth measurement."""
    # Keep usable depth readings between 0.1 and 8 metres.
    valid = region & np.isfinite(depth) & (depth > 0.1) & (depth < 8.0)
    (ys, xs) = np.nonzero(valid)
    if len(xs) < 30:
        return []
    # Check depth at pixels across the area so we don't rely on one pixel.
    # Use at most 256 to keep processing fast.
    indices = np.linspace(0, len(xs) - 1, min(256, len(xs)), dtype=int)
    return [point(int(xs[i]), int(ys[i]), float(depth[ys[i], xs[i]]), p, name, score) for i in indices]

class FusedDepth:

    def __init__(self, model_dir=None):
        self.model_dir = Path(model_dir).expanduser() if model_dir else Path.home() / 'human_model'
        self.pose = PoseDepth(self.model_dir / 'yolov8n-pose.onnx')
        self.seg = SegmentationDepth(self.model_dir / 'yolov8n-seg.onnx')
        self.box = None

    def estimate(self, rgb, depth, info):
        # Check that colour, depth, and camera settings describe matching images.
        (h, w) = rgb.shape[:2]
        p = np.asarray(info.p)
        if depth.shape != (h, w) or (info.width, info.height) != (w, h):
            raise ValueError('RGB/depth must be aligned with matching calibration dimensions')
        if not np.isfinite(p).all() or p[0] <= 0 or p[5] <= 0 or (p[3] != 0) or (p[7] != 0):
            raise ValueError('Expected valid rectified camera calibration with zero offsets')
        choices = []
        diagnostics = []
        # Try the image upright and turned both ways to help find lying people.
        views = [(0, rgb), (90, cv2.rotate(rgb, cv2.ROTATE_90_COUNTERCLOCKWISE)), (-90, cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE))]
        for (angle, view) in views:
            poses = self.pose.predict(view, details=True)
            masks = self.seg.predict(view)
            diagnostics.append({'rotation': angle, 'poses': len(poses), 'masks': len(masks)})
            # Skip this view if either model reports more than one person.
            if len(poses) > 1 or len(masks) > 1:
                continue
            pose = poses[0] if poses else None
            seg = masks[0] if masks else None
            # Check that the body joints and mask appear to describe the same person.
            if pose is not None and seg is not None and (overlap(pose['box'], seg['box']) < 0.3):
                continue
            interior = None
            if seg is not None:
                mask = seg['mask'].astype(np.uint8)
                if angle == 90:
                    mask = cv2.rotate(mask, cv2.ROTATE_90_CLOCKWISE)
                elif angle == -90:
                    mask = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
                # Remove the outer edge of the mask, where pixels may include the background.
                interior = cv2.erode(mask, np.ones((3, 3), np.uint8)).astype(bool)
            candidates = []
            if pose is not None:
                (x, y, bw, bh) = pose['box']
                # Check the body points one by one, keeping those with a high enough score.
                for (index, name) in BODY_POINTS.items():
                    (u, v, confidence) = pose['points'][index]
                    if confidence < 0.7 or not np.isfinite([u, v, confidence]).all():
                        continue
                    if not (x <= u < x + bw and y <= v < y + bh):
                        continue
                    (u, v) = original_pixel(u, v, angle, w, h)
                    (u, v) = (round(u), round(v))
                    if not (0 <= u < w and 0 <= v < h):
                        continue
                    # If there is a mask, both the body point and chosen depth pixel must be inside it.
                    if interior is not None and (not interior[v, u]):
                        continue
                    # Make an empty selection area; the next lines choose which pixels to check.
                    region = np.zeros((h, w), bool)
                    region[max(0, v - 1):min(h, v + 2), max(0, u - 1):min(w, u + 2)] = True
                    if interior is not None:
                        region &= interior
                    # Keep usable depth readings between 0.1 and 8 metres.
                    valid = region & np.isfinite(depth) & (depth > 0.1) & (depth < 8.0)
                    (ys, xs) = np.nonzero(valid)
                    if len(xs) < 5:
                        continue
                    values = depth[ys, xs]
                    median = np.median(values)
                    # Skip patches with very different depths: they may mix the person and an obstacle.
                    if np.max(values) - np.min(values) > 0.3:
                        continue
                    # Choose the actual pixel whose depth is nearest the middle depth value.
                    i = int(np.argmin(abs(values - median)))
                    candidates.append(point(int(xs[i]), int(ys[i]), float(values[i]), p, name, confidence))
            # Group nearby 3D points and choose the actual point closest overall to the others.
            choice = choose_supported_point(candidates)
            if choice is not None:
                choice.update(source='mask_pose' if interior is not None else 'pose_only', rotation=angle)
                choices.append(choice)
            # If no joint group works, try pixels inside the person mask.
            elif interior is not None:
                samples = surface_points(depth, interior, p, seg['score'], 'mask_pixel')
                choice = choose_supported_point(samples, minimum_points=20)
                if choice is not None:
                    choice.update(source='mask_only', rotation=angle)
                    choices.append(choice)
        if choices:
            # Prefer joints inside a mask, then mask pixels, then joints without a mask.
            rank = {'mask_pose': 3, 'mask_only': 2, 'pose_only': 1}
            return (max(choices, key=lambda c: (rank[c['source']], len(c['support']))), diagnostics)
        # If none of those methods works, try the model that finds person boxes.
        if self.box is None:
            self.box = YoloDetector(self.model_dir / 'yolov8n.onnx', 0.5)
        for (angle, view) in views:
            (boxes, scores) = self.box.detect(view)
            keep = np.asarray(cv2.dnn.NMSBoxes(boxes, scores, 0.5, 0.45)).reshape(-1)
            if len(keep) != 1:
                continue
            i = int(keep[0])
            (x, y, bw, bh) = boxes[i]
            (x1, y1, x2, y2) = restore_box((x, y, x + bw, y + bh), angle, w, h)
            # Avoid box edges to reduce background pixels. Pixels inside can still be an obstacle.
            dx = 0.1 * (x2 - x1)
            dy = 0.1 * (y2 - y1)
            # Make an empty selection area; the next lines choose which pixels to check.
            region = np.zeros((h, w), bool)
            region[max(0, int(y1 + dy)):min(h, int(y2 - dy)), max(0, int(x1 + dx)):min(w, int(x2 - dx))] = True
            samples = surface_points(depth, region, p, scores[i], 'box_pixel')
            choice = choose_supported_point(samples, minimum_points=30)
            if choice is None:
                continue
            # Require points spread across part of the box, not just one tiny patch.
            pixels = np.array([q['pixel'] for q in choice['support']])
            if np.ptp(pixels[:, 0]) < 0.2 * (x2 - x1) or np.ptp(pixels[:, 1]) < 0.2 * (y2 - y1):
                continue
            choice.update(source='box_only', rotation=angle)
            choices.append(choice)
        return (max(choices, key=lambda c: len(c['support'])) if choices else None, diagnostics)

class Confirmation:
    """Check whether several images give similar human locations on the map.
    All detection methods need three agreeing observations.
    """

    def __init__(self):
        self.history = []

    def reset(self):
        self.history = []

    def add(self, xyz, stamp, source):
        # Do not count the same image twice or use an older observation.
        if self.history and stamp <= self.history[-1][1]:
            return False
        # Start again if this image places the human more than 30 cm from
        # the previous estimate, or if more than 5 seconds passed between images.
        if self.history and (stamp - self.history[-1][1] > 5 or np.linalg.norm(np.asarray(xyz) - self.history[-1][0]) > 0.3):
            self.reset()
        self.history.append((np.asarray(xyz), stamp, source))
        self.history = self.history[-3:]
        # Wait for three observations, whichever detection method was used.
        if len(self.history) < 3:
            return False
        # Check that all stored map locations are within 30 cm of each other.
        points = np.array([q[0] for q in self.history])
        if np.max(np.linalg.norm(points[:, None] - points[None, :], axis=2)) > 0.3:
            return False
        return True
