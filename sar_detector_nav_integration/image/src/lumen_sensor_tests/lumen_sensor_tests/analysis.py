"""Numerical routines independent of ROS executors and robot hardware."""

from collections import deque
import math

import numpy as np


def cloud_xyz(msg):
    """Read XYZ with field offsets, endian, point padding and row padding intact.

    Deliberately inspect nonfinite values even when the sender sets is_dense.
    Supports scalar FLOAT32/FLOAT64 XYZ fields; other fields are ignored.
    """
    count = int(msg.width) * int(msg.height)
    if count == 0:
        return np.empty((0, 3), dtype=float)
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid point_step or row_step')
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated PointCloud2 data')
    endian = '>' if msg.is_bigendian else '<'
    columns = []
    for name in ('x', 'y', 'z'):
        fields = [f for f in msg.fields if f.name == name]
        if len(fields) != 1 or fields[0].count != 1:
            raise ValueError(f'Missing, duplicate or nonscalar {name} field')
        field = fields[0]
        if field.datatype not in (7, 8):
            raise ValueError(f'{name} must be FLOAT32 or FLOAT64')
        dtype = np.dtype(endian + ('f4' if field.datatype == 7 else 'f8'))
        if field.offset < 0 or field.offset + dtype.itemsize > msg.point_step:
            raise ValueError(f'{name} field exceeds point_step')
        column = np.ndarray(
            (msg.height, msg.width), dtype=dtype, buffer=memoryview(msg.data),
            offset=field.offset, strides=(msg.row_step, msg.point_step))
        columns.append(column.reshape(-1))
    return np.column_stack(columns).astype(float, copy=False)


def transform_xyz(xyz, translation, quaternion):
    """Apply target<-source transform; quaternion is x,y,z,w."""
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or not np.isfinite(t).all() or norm < 1e-12:
        raise ValueError('Invalid TF translation or quaternion')
    if abs(norm - 1.0) > 0.01:
        raise ValueError('TF quaternion is not normalized')
    x, y, z, w = q / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return xyz @ rotation.T + t


def cloud_summary(xyz):
    finite = np.isfinite(xyz).all(axis=1)
    valid = xyz[finite]
    return {
        'point_count': len(xyz), 'finite_count': len(valid),
        'invalid_count': int((~finite).sum()),
        'nan_count': int(np.isnan(xyz).any(axis=1).sum()),
        'xyz_min': valid.min(axis=0).tolist() if len(valid) else None,
        'xyz_max': valid.max(axis=0).tolist() if len(valid) else None,
    }


def filter_xyz(xyz, min_range, max_range, min_height, max_height, exclusion):
    """Filter in base frame using horizontal distance and a body exclusion box."""
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    distance = np.hypot(xyz[:, 0], xyz[:, 1])
    keep = ((distance >= min_range) & (distance <= max_range)
            & (xyz[:, 2] >= min_height) & (xyz[:, 2] <= max_height))
    if exclusion:
        x0, x1, y0, y1, z0, z1 = exclusion
        inside = ((xyz[:, 0] >= x0) & (xyz[:, 0] <= x1)
                  & (xyz[:, 1] >= y0) & (xyz[:, 1] <= y1)
                  & (xyz[:, 2] >= z0) & (xyz[:, 2] <= z1))
        keep &= ~inside
    return xyz[keep]


def orientation_candidate(xyz, min_points=3, range_band=0.10):
    """Describe the nearest range band, not an identified physical object."""
    if not len(xyz):
        return {'direction': 'UNKNOWN', 'reason': 'No eligible observations'}
    ranges = np.hypot(xyz[:, 0], xyz[:, 1])
    near = xyz[ranges <= ranges.min() + range_band]
    if len(near) < min_points:
        return {'direction': 'UNKNOWN', 'reason': 'Sparse nearest returns'}
    angles = np.arctan2(near[:, 1], near[:, 0])
    mean_vector = np.array([np.cos(angles).mean(), np.sin(angles).mean()])
    if np.linalg.norm(mean_vector) < 0.9:
        return {'direction': 'UNKNOWN', 'reason': 'Ambiguous nearest returns'}
    angle = math.atan2(mean_vector[1], mean_vector[0])
    names = ('front', 'front-left', 'left', 'rear-left',
             'rear', 'rear-right', 'right', 'front-right')
    index = int(math.floor((angle + math.pi / 8) / (math.pi / 4))) % 8
    return {'direction': names[index], 'bearing_deg': math.degrees(angle),
            'range_m': float(np.median(np.hypot(near[:, 0], near[:, 1]))),
            'point_count': len(near)}


class ArrivalWindow:
    """Arrival-rate estimate using monotonic wall time, bounded memory."""

    def __init__(self):
        self.times = deque(maxlen=100)

    def add(self, when):
        self.times.append(when)

    def summary(self, now):
        interval = None
        if len(self.times) > 1:
            interval = (self.times[-1] - self.times[0]) / (len(self.times) - 1)
        return {'arrival_age_sec': now - self.times[-1] if self.times else None,
                'rate_hz': 1.0 / interval if interval and interval > 0 else None,
                'interval_sec': interval}


def timestamp_health(now_sec, stamp_sec, arrival_age, stale_timeout, future_tolerance):
    age = now_sec - stamp_sec
    reasons = []
    if arrival_age is None or arrival_age > stale_timeout:
        reasons.append('No recent arrivals')
    if stamp_sec == 0:
        reasons.append('Zero timestamp: acquisition age unknown')
    elif age > stale_timeout:
        reasons.append('Stale measurement timestamp')
    elif age < -future_tolerance:
        reasons.append('Future timestamp: check clock synchronization')
    return age, reasons


def imu_summary(msg, stationary, gyro_tolerance, gravity, gravity_tolerance):
    q = np.array([msg.orientation.x, msg.orientation.y,
                  msg.orientation.z, msg.orientation.w])
    gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                     msg.angular_velocity.z])
    accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z])
    result, issues = {}, []
    for name, values, covariance in (
            ('orientation', q, msg.orientation_covariance),
            ('angular_velocity', gyro, msg.angular_velocity_covariance),
            ('linear_acceleration', accel, msg.linear_acceleration_covariance)):
        available = bool(covariance[0] != -1.0)
        finite = np.isfinite(values).all()
        result[name] = values.tolist() if finite else 'NONFINITE'
        result[name + '_available'] = available
        cov = np.asarray(covariance)
        result[name + '_covariance'] = (
            'unavailable' if not available else
            'invalid' if not np.isfinite(cov).all() else
            'unknown' if np.all(cov == 0) else 'provided')
        if available and (not finite or not np.isfinite(cov).all()):
            issues.append(f'Invalid {name} values or covariance')
    norm = math.hypot(*q) if np.isfinite(q).all() else None
    if norm is not None and not math.isfinite(norm):
        norm = None
    result['quaternion_norm'] = norm
    if result['orientation_available']:
        if norm is None or abs(norm - 1.0) > 0.01:
            issues.append('Orientation quaternion is not normalized')
    else:
        result['orientation_note'] = 'Not supplied; no absolute yaw assumed'
    if stationary:
        for name, values, target, tolerance in (
                ('angular_velocity', gyro, 0.0, gyro_tolerance),
                ('linear_acceleration', accel, gravity, gravity_tolerance)):
            if not result[name + '_available'] or not np.isfinite(values).all():
                result[name + '_stationary_check'] = 'UNAVAILABLE'
                issues.append(f'Stationary check unavailable: {name}')
                continue
            magnitude = math.hypot(*values)
            if not math.isfinite(magnitude):
                result[name + '_stationary_check'] = 'UNAVAILABLE'
                issues.append(f'Invalid {name} magnitude')
                continue
            result[name + '_magnitude'] = magnitude
            passed = abs(magnitude - target) <= tolerance
            result[name + '_stationary_check'] = 'PASS' if passed else 'WARN'
            if not passed:
                issues.append(f'Stationary {name} outside tolerance')
    return result, issues
