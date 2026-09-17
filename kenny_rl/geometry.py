"""Analytical rays against axis-aligned 3D boxes; conservative circular footprint."""
import numpy as np


def ray_boxes(origin, directions, boxes, max_range):
    """Return nearest ray distance and box index (-1 for no surface).

    Boxes are [xmin, ymin, zmin, xmax, ymax, zmax]. Directions are unit vectors.
    Parallel axes are handled explicitly, including origins on box boundaries.
    """
    n = len(directions)
    if len(boxes) == 0:
        return np.full(n, max_range), np.full(n, -1, dtype=int)
    origin, directions, boxes = map(np.asarray, (origin, directions, boxes))
    parallel = np.abs(directions[:, None, :]) < 1e-10
    safe = np.where(parallel, 1.0, directions[:, None, :])
    a = (boxes[None, :, :3] - origin) / safe
    b = (boxes[None, :, 3:] - origin) / safe
    outside = (origin < boxes[None, :, :3]) | (origin > boxes[None, :, 3:])
    lo = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=2)
    hi = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=2)
    valid = (hi >= np.maximum(lo, 0)) & ~np.any(parallel & outside, axis=2)
    distance = np.where(valid, np.maximum(lo, 0), np.inf)
    indices = distance.argmin(axis=1)
    closest = distance[np.arange(n), indices]
    hit = closest < max_range
    return np.where(hit, closest, max_range), np.where(hit, indices, -1)


def circle_boxes(position, radius, boxes, height):
    if len(boxes) == 0:
        return False
    boxes = np.asarray(boxes)
    closest = np.maximum(boxes[:, :2], np.minimum(position, boxes[:, 3:5]))
    d2 = np.sum((closest - position) ** 2, axis=1)
    return bool(np.any((d2 <= radius ** 2) & (boxes[:, 2] < height) & (boxes[:, 5] > 0)))


def circle_rects(position, radius, rectangles):
    if len(rectangles) == 0:
        return False
    rect = np.asarray(rectangles)
    closest = np.maximum(rect[:, :2], np.minimum(position, rect[:, 2:]))
    return bool(np.any(np.sum((closest - position) ** 2, axis=1) <= radius ** 2))


def inside_rects(points, rectangles):
    points = np.asarray(points)
    if len(rectangles) == 0:
        return np.zeros(points.shape[:-1], dtype=bool)
    rectangles = np.asarray(rectangles)
    return np.any(np.all((points[..., None, :] >= rectangles[:, :2]) &
                         (points[..., None, :] <= rectangles[:, 2:]), axis=-1), axis=-1)


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
