"""Move boxes back to the original image and choose a point near the other measurements."""
import numpy as np

def restore_box(box, rotation, width, height):
    """Move a box found in a rotated image back to its original image position.

    Coordinates describe box edges, so the far edge can equal the image size.
    The angle is 0, 90 degrees left, or 90 degrees right (-90).
    """
    (x1, y1, x2, y2) = box
    if rotation == 90:
        (x1, y1, x2, y2) = (width - y2, x1, width - y1, x2)
    elif rotation == -90:
        (x1, y1, x2, y2) = (y1, height - x2, y2, height - x1)
    elif rotation != 0:
        raise ValueError('Unsupported rotation')
    (x1, x2) = [max(0, min(width, int(round(v)))) for v in (x1, x2)]
    (y1, y2) = [max(0, min(height, int(round(v)))) for v in (y1, y2)]
    return (x1, y1, x2, y2)

def choose_supported_point(candidates, radius_m=0.65, minimum_points=3):
    # Stop if there are too few points to compare.
    if len(candidates) < minimum_points:
        return None
    xyz = np.array([c['xyz'] for c in candidates])
    # Measure the distance between every pair of 3D points.
    distances = np.linalg.norm(xyz[:, None, :] - xyz[None, :, :], axis=2)
    # Find the point with the most other points nearby.
    anchor = int(np.argmax(np.sum(distances <= radius_m, axis=1)))
    group = np.flatnonzero(distances[anchor] <= radius_m)
    if len(group) < minimum_points:
        return None
    # Choose the actual point closest overall to the others. This is the medoid.
    totals = distances[np.ix_(group, group)].sum(axis=1)
    chosen = int(group[np.argmin(totals)])
    return {'selected': candidates[chosen], 'support': [candidates[int(i)] for i in group]}
