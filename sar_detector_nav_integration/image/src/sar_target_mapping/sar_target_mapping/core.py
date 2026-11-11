"""DETECTOR_NAV_INTEGRATION: ROS-independent geometry and association."""
import math
import statistics
import struct


def stamp_ns(header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


def depth_point(image, info, bbox, min_depth=0.2, max_depth=8.0):
    """Central-box median depth; requires rectified RGB-aligned depth and K."""
    if image.width != info.width or image.height != info.height:
        raise ValueError('Depth and CameraInfo dimensions differ')
    if image.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Depth encoding must be 16UC1 (mm) or 32FC1 (m)')
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if not all(math.isfinite(x) for x in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError('Invalid intrinsics')
    u, v, w, h = bbox
    if not all(math.isfinite(x) for x in bbox) or w <= 0 or h <= 0:
        raise ValueError('Invalid bounding box')
    if not (0 <= u < image.width and 0 <= v < image.height):
        raise ValueError('Box center outside image')
    size, fmt, scale = (2, 'H', 0.001) if image.encoding == '16UC1' else (4, 'f', 1.)
    if image.step < image.width * size or len(image.data) < image.step * image.height:
        raise ValueError('Malformed depth buffer')
    # Inner 40% of box; at most about 21 x 21 samples, respecting row padding.
    x0, x1 = max(0, int(u-w*.2)), min(image.width-1, int(u+w*.2))
    y0, y1 = max(0, int(v-h*.2)), min(image.height-1, int(v+h*.2))
    samples = []
    for y in range(y0, y1+1, max(1, (y1-y0)//20)):
        for x in range(x0, x1+1, max(1, (x1-x0)//20)):
            z = struct.unpack_from(('>' if image.is_bigendian else '<')+fmt,
                                   image.data, y*image.step+x*size)[0]*scale
            if math.isfinite(z) and min_depth <= z <= max_depth:
                samples.append(z)
    if len(samples) < 5:
        raise ValueError('Insufficient valid depth samples')
    z = statistics.median(samples)
    mad = statistics.median(abs(d-z) for d in samples)
    if mad > max(.10, .15*z):
        raise ValueError('Depth region inconsistent')
    distortion = list(getattr(info, 'd', []))
    if any(abs(value) > 1e-12 for value in distortion):
        # RGB-aligned depth uses RGB pixel geometry; account for lens distortion.
        import cv2
        import numpy as np
        matrix = np.array(info.k, dtype=float).reshape(3, 3)
        pixel = np.array([[[u, v]]], dtype=float)
        model = getattr(info, 'distortion_model', '')
        if model == 'equidistant' and len(distortion) == 4:
            ray = cv2.fisheye.undistortPoints(pixel, matrix, np.array(distortion))
        elif model in ('plumb_bob', 'rational_polynomial'):
            ray = cv2.undistortPoints(pixel, matrix, np.array(distortion))
        else:
            raise ValueError('Unsupported distortion model')
        return (float(ray[0, 0, 0])*z, float(ray[0, 0, 1])*z, z)
    return ((u-cx)*z/fx, (v-cy)*z/fy, z)


def transform_point(point, translation, quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(sum(v*v for v in quaternion))
    if not all(math.isfinite(v) for v in (*point, *translation, *quaternion)) or abs(norm-1) > .01:
        raise ValueError('Invalid TF')
    x, y, z, w = (v/norm for v in quaternion)
    a, b, c = point
    return (translation[0]+(1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
            translation[1]+2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
            translation[2]+2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c)


def associate(records, label, point, radius):
    matches = [(math.hypot(r['position'][0]-point[0], r['position'][1]-point[1]), i)
               for i, r in enumerate(records) if r['class_id'] == label]
    distance, index = min(matches, default=(float('inf'), None))
    return index if distance <= radius else None
