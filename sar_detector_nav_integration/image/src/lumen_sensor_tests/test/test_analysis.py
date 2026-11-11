import math
import struct
from types import SimpleNamespace as Obj

import numpy as np
import pytest

from lumen_sensor_tests.analysis import (
    ArrivalWindow, cloud_summary, cloud_xyz, filter_xyz, imu_summary,
    orientation_candidate, timestamp_health, transform_xyz,
)


def make_cloud(points, big=False, double=False, width=None):
    size = 8 if double else 4
    width = len(points) if width is None else width
    height = len(points) // width if width else 0
    step = 4 * size
    row_step = width * step + 8
    data = bytearray(row_step * height)
    for i, point in enumerate(points):
        # Deliberately shuffled field offsets, extra point and row padding.
        struct.pack_into(('>' if big else '<') + ('ddd' if double else 'fff'),
                         data, (i // width) * row_step + (i % width) * step,
                         point[2], point[0], point[1])
    return Obj(width=width, height=height, point_step=step, row_step=row_step,
               is_bigendian=big, is_dense=True, data=data,
               fields=[Obj(name=n, offset=o*size, datatype=8 if double else 7, count=1)
                       for n, o in [('x', 1), ('y', 2), ('z', 0)]])


@pytest.mark.parametrize('big', [False, True])
@pytest.mark.parametrize('double', [False, True])
def test_organized_padded_cloud(big, double):
    points = [(1, 2, 3), (4, 5, 6), (-1, -2, -3), (7, 8, 9)]
    np.testing.assert_allclose(cloud_xyz(make_cloud(points, big, double, 2)), points)


def test_invalid_returns_not_hidden_by_dense_flag():
    xyz = cloud_xyz(make_cloud([(1, 2, 3), (float('nan'), 0, 0), (0, float('inf'), 0)]))
    result = cloud_summary(xyz)
    assert result['invalid_count'] == 2
    assert result['nan_count'] == 1
    assert result['xyz_min'] == [1, 2, 3]


@pytest.mark.parametrize('mutation', ['truncated', 'missing', 'row', 'field', 'type'])
def test_malformed_cloud(mutation):
    msg = make_cloud([(1, 2, 3)])
    if mutation == 'truncated':
        msg.data = msg.data[:2]
    elif mutation == 'missing':
        msg.fields.pop()
    elif mutation == 'row':
        msg.row_step = 1
    elif mutation == 'field':
        msg.fields[0].offset = 100
    else:
        msg.fields[0].datatype = 2
    with pytest.raises(ValueError):
        cloud_xyz(msg)


def test_empty_cloud_unknown():
    xyz = cloud_xyz(make_cloud([]))
    assert cloud_summary(xyz)['xyz_min'] is None
    assert orientation_candidate(xyz)['direction'] == 'UNKNOWN'


@pytest.mark.parametrize('angle,expected', [
    (0, 'front'), (45, 'front-left'), (90, 'left'), (135, 'rear-left'),
    (180, 'rear'), (225, 'rear-right'), (270, 'right'), (315, 'front-right'),
])
def test_orientation_after_rotated_sensor_transform(angle, expected):
    # Sensor faces 90 degrees left; coordinates alone would report the wrong direction.
    a = math.radians(angle - 90)
    source = np.tile([math.cos(a), math.sin(a), 0], (4, 1))
    target = transform_xyz(source, [0, 0, 0], [0, 0, math.sqrt(.5), math.sqrt(.5)])
    assert orientation_candidate(target)['direction'] == expected


def test_translation_and_bad_quaternion():
    np.testing.assert_allclose(transform_xyz(np.array([[1., 0, 0]]), [2, 3, 4], [0, 0, 0, 1]),
                               [[3, 3, 4]])
    with pytest.raises(ValueError):
        transform_xyz(np.zeros((1, 3)), [0, 0, 0], [0, 0, 0, 0])


def test_filter_ground_body_range_and_invalid():
    points = np.array([[1, 0, 0], [.1, 0, 0], [2, 0, -.5], [20, 0, 0],
                       [float('nan'), 0, 0], [0, 1, 0]])
    kept = filter_xyz(points, .05, 5, -.1, 1, [-.3, .3, -.2, .2, -.5, .3])
    np.testing.assert_allclose(kept, [[1, 0, 0], [0, 1, 0]])


def test_sparse_and_ambiguous_orientation():
    assert orientation_candidate(np.array([[1., 0, 0]]))['direction'] == 'UNKNOWN'
    assert orientation_candidate(np.array([[1., 0, 0], [-1, 0, 0]]), 2)['direction'] == 'UNKNOWN'


def test_arrival_rate_and_missing_data():
    arrivals = ArrivalWindow()
    assert arrivals.summary(1)['arrival_age_sec'] is None
    for when in [1., 1.1, 1.2]:
        arrivals.add(when)
    assert arrivals.summary(2)['rate_hz'] == pytest.approx(10)
    assert arrivals.summary(2)['arrival_age_sec'] == pytest.approx(.8)


@pytest.mark.parametrize('stamp,arrival,fragment', [
    (0., .1, 'Zero timestamp'), (8., .1, 'Stale measurement'),
    (11., .1, 'Future timestamp'), (10., 2., 'No recent arrivals'),
])
def test_timestamp_faults(stamp, arrival, fragment):
    _, problems = timestamp_health(10., stamp, arrival, 1., .1)
    assert any(fragment in p for p in problems)


def imu():
    return Obj(orientation=Obj(x=0., y=0., z=0., w=1.),
               angular_velocity=Obj(x=0., y=0., z=0.),
               linear_acceleration=Obj(x=0., y=0., z=9.80665),
               orientation_covariance=[0.] * 9, angular_velocity_covariance=[0.] * 9,
               linear_acceleration_covariance=[0.] * 9)


def test_stationary_imu_and_unknown_covariance():
    values, issues = imu_summary(imu(), True, .1, 9.80665, 1.5)
    assert not issues
    assert values['orientation_covariance'] == 'unknown'
    assert values['linear_acceleration_stationary_check'] == 'PASS'


def test_imu_missing_orientation_does_not_require_quaternion():
    msg = imu()
    msg.orientation_covariance[0] = -1.
    msg.orientation.w = 0.
    values, issues = imu_summary(msg, True, .1, 9.80665, 1.5)
    assert not issues
    assert not values['orientation_available']


def test_imu_invalid_and_nonstationary():
    msg = imu()
    msg.orientation.w = 0.
    msg.angular_velocity.z = 1.
    msg.linear_acceleration.x = float('nan')
    values, issues = imu_summary(msg, True, .1, 9.80665, 1.5)
    assert any('not normalized' in p for p in issues)
    assert values['angular_velocity_stationary_check'] == 'WARN'
    assert values['linear_acceleration_stationary_check'] == 'UNAVAILABLE'
