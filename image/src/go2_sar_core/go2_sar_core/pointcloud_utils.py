"""Small numpy helpers for sensor_msgs/PointCloud2 and planar geometry."""
import math

import numpy as np

_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
           5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud_to_xyz(msg):
    """Return the finite x, y, z points of a PointCloud2 as an (N, 3) float32 array."""
    fields = {f.name: f for f in msg.fields}
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [_DTYPES[fields[n].datatype] for n in ('x', 'y', 'z')],
        'offsets': [fields[n].offset for n in ('x', 'y', 'z')],
        'itemsize': msg.point_step,
    })
    raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def rotation_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def voxel_downsample_2d(points, cell):
    """Keep one point per cell x cell square."""
    if len(points) == 0:
        return points
    keys = np.floor(points / cell).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def icp_translation(source, reference, max_iterations=10, max_correspondence=0.3,
                    min_inliers=30):
    """Translation-only point-to-point ICP in 2D (rotation is taken from the IMU).

    source, reference: (N, 2) and (M, 2) points in the same frame.
    Returns (dx, dy, inlier_ratio) or None if there are too few correspondences.
    """
    if len(source) < min_inliers or len(reference) < min_inliers:
        return None
    src = source.copy()
    total = np.zeros(2)
    ratio = 0.0
    for i in range(max_iterations):
        diff = src[:, None, :] - reference[None, :, :]
        d2 = np.einsum('nmk,nmk->nm', diff, diff)
        nearest = d2.argmin(axis=1)
        dist = np.sqrt(d2[np.arange(len(src)), nearest])
        # shrink the correspondence gate as the alignment improves
        gate = max(max_correspondence * (0.7 ** i), 0.08)
        inliers = dist < gate
        ratio = inliers.mean()
        if inliers.sum() < min_inliers:
            return None
        step = (reference[nearest[inliers]] - src[inliers]).mean(axis=0)
        src += step
        total += step
        if np.hypot(*step) < 1e-3:
            break
    return float(total[0]), float(total[1]), float(ratio)


def corridor_clearance(points, headings, half_width, max_range):
    """Free distance along each heading for a robot of width 2*half_width.

    points:   (N, 2) obstacle points in the robot frame.
    headings: (K,) candidate directions [rad].
    Returns (K,) distances; max_range if the corridor is empty.
    """
    if len(points) == 0:
        return np.full(len(headings), max_range)
    c, s = np.cos(headings)[:, None], np.sin(headings)[:, None]
    along = c * points[:, 0] + s * points[:, 1]       # (K, N)
    lateral = -s * points[:, 0] + c * points[:, 1]    # (K, N)
    blocked = (np.abs(lateral) < half_width) & (along > 0.0)
    dist = np.where(blocked, along, max_range)
    return np.minimum(dist.min(axis=1), max_range)
