"""Synthetic DDS integration: diagnostic nodes only, no robot interfaces.

Run with ROS_LOCALHOST_ONLY=1 and a separate ROS_DOMAIN_ID as documented.
"""

import json
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from tf2_ros import StaticTransformBroadcaster

from lumen_sensor_tests.imu_diagnostic import ImuDiagnostic
from lumen_sensor_tests.lidar_diagnostic import LidarDiagnostic
from lumen_sensor_tests.lidar_orientation_check import LidarOrientationCheck
from lumen_sensor_tests.receive_time_adapter import ReceiveTimeAdapter, restamp


@pytest.fixture
def harness(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '187')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    rclpy.init(args=['--ros-args', '-p', 'cloud_topic:=/lumen_test/cloud',
                    '-p', 'imu_topic:=/lumen_test/imu', '-p', 'report_rate_hz:=10.0',
                    '-p', 'stale_timeout_sec:=0.4', '-p', 'stationary_check:=true',
                    '-p', 'expected_direction:=left'])
    executor = SingleThreadedExecutor()
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    probe = add(Node('synthetic_sensor_probe'))
    reports = []
    probe.create_subscription(DiagnosticArray, '/lumen/diagnostics', reports.append, 10)

    def wait(predicate, publish=None, timeout=6.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                publish()
            executor.spin_once(timeout_sec=.03)
            if predicate():
                return
        raise AssertionError('Timed out waiting for synthetic diagnostic result')

    yield add, probe, reports, wait
    for node in reversed(nodes):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


def latest(reports):
    status = reports[-1].status[0]
    return status, {entry.key: json.loads(entry.value) for entry in status.values}


def cloud(probe, frame='base_link'):
    msg = PointCloud2()
    msg.header.stamp = probe.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.height, msg.width = 1, 3
    msg.point_step, msg.row_step = 12, 36
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.data = struct.pack('<fffffffff', 1., 0., 0., 1., .01, 0., 1., -.01, 0.)
    return msg


def test_no_data_reports_and_no_motion_publishers(harness):
    add, probe, reports, wait = harness
    node = add(LidarDiagnostic())
    wait(lambda: bool(reports))
    status, _ = latest(reports)
    assert status.level == DiagnosticStatus.ERROR
    assert 'No messages' in status.message
    publishers = node.get_publisher_names_and_types_by_node(node.get_name(), '/')
    assert {name for name, _ in publishers} <= {'/lumen/diagnostics', '/rosout', '/parameter_events'}


def test_cloud_reception_and_stale_detection(harness):
    add, probe, reports, wait = harness
    add(LidarDiagnostic())
    publisher = probe.create_publisher(PointCloud2, '/lumen_test/cloud', qos_profile_sensor_data)
    wait(lambda: bool(reports) and latest(reports)[1].get('filtered_point_count') == 3,
         lambda: publisher.publish(cloud(probe)))
    assert latest(reports)[0].level == DiagnosticStatus.OK
    wait(lambda: bool(reports) and not latest(reports)[1].get('fresh', True))
    assert latest(reports)[0].level == DiagnosticStatus.WARN


def test_missing_tf_does_not_report_direction(harness):
    add, probe, reports, wait = harness
    add(LidarOrientationCheck())
    publisher = probe.create_publisher(PointCloud2, '/lumen_test/cloud', qos_profile_sensor_data)
    wait(lambda: bool(reports) and latest(reports)[1].get('analysis_available') is False,
         lambda: publisher.publish(cloud(probe, 'unconnected_lidar')))
    status, values = latest(reports)
    assert status.level == DiagnosticStatus.ERROR
    assert values['candidate']['direction'] == 'UNKNOWN'


def test_tf_rotates_orientation_into_base_frame(harness):
    add, probe, reports, wait = harness
    add(LidarOrientationCheck())
    broadcaster = StaticTransformBroadcaster(probe)
    tf = TransformStamped()
    tf.header.stamp = probe.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'rotated_lidar'
    tf.transform.rotation.z = 2 ** -.5
    tf.transform.rotation.w = 2 ** -.5
    broadcaster.sendTransform(tf)
    publisher = probe.create_publisher(PointCloud2, '/lumen_test/cloud', qos_profile_sensor_data)
    wait(lambda: bool(reports) and latest(reports)[1].get('comparison') == 'MATCH',
         lambda: publisher.publish(cloud(probe, 'rotated_lidar')))
    assert latest(reports)[1]['candidate']['direction'] == 'left'


def test_standard_imu_stationary(harness):
    add, probe, reports, wait = harness
    add(ImuDiagnostic())
    publisher = probe.create_publisher(Imu, '/lumen_test/imu', qos_profile_sensor_data)

    def publish():
        msg = Imu()
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.80665
        publisher.publish(msg)

    wait(lambda: bool(reports) and latest(reports)[1].get('linear_acceleration_stationary_check') == 'PASS',
         publish)
    assert latest(reports)[0].level == DiagnosticStatus.OK


@pytest.mark.parametrize('kind', ['lidar', 'imu'])
def test_installed_launch_receives_synthetic_data(harness, tmp_path, kind):
    """Exercise installed entry points, launch arguments and YAML in a process."""
    _, probe, reports, wait = harness
    config = tmp_path / 'test.yaml'
    config.write_text(
        '/**:\n  ros__parameters:\n    report_rate_hz: 10.0\n'
        'lidar_diagnostic:\n  ros__parameters:\n    base_frame: wrong_yaml_frame\n'
        'imu_diagnostic:\n  ros__parameters:\n    stationary_check: false\n')
    overrides = []
    if kind == 'lidar':
        overrides = ['base_frame:=synthetic_sensor_frame']
        topic_arg, message_type = 'cloud_topic:=/lumen_test/cloud', PointCloud2
        topic = '/lumen_test/cloud'

        def make():
            return cloud(probe, 'synthetic_sensor_frame')

        def complete():
            return bool(reports) and latest(reports)[1].get('filtered_point_count') == 3
    else:
        overrides = ['stationary_check:=true']
        topic_arg, message_type = 'imu_topic:=/lumen_test/imu', Imu
        topic = '/lumen_test/imu'

        def make():
            msg = Imu()
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.orientation.w = 1.0
            msg.linear_acceleration.z = 9.80665
            return msg

        def complete():
            return (bool(reports) and
                    latest(reports)[1].get('linear_acceleration_stationary_check') == 'PASS')

    publisher = probe.create_publisher(message_type, topic, qos_profile_sensor_data)
    log_path = tmp_path / 'launch.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'lumen_sensor_tests', f'{kind}_test.launch.py',
             topic_arg, f'params_file:={config}', *overrides], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            wait(complete, lambda: publisher.publish(make()), timeout=12.)
            assert process.poll() is None, log_path.read_text()
            assert latest(reports)[0].level == DiagnosticStatus.OK
        except Exception as exc:
            raise AssertionError(log_path.read_text()[-5000:]) from exc
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    import os
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)


@pytest.mark.parametrize('offset,assessment,code', [
    (0, 'TIMING_PLAUSIBLE', 0), (2507, 'PERSISTENT_OFFSET', 2),
    (None, 'NO_DATA', 2),
])
def test_timestamp_command_reports(harness, tmp_path, offset, assessment, code):
    _, probe, _, wait = harness
    publisher = probe.create_publisher(PointCloud2, '/lumen_test/cloud', qos_profile_sensor_data)
    output = tmp_path / 'reports'

    def publish():
        if offset is not None:
            msg = cloud(probe)
            msg.header.stamp.sec -= offset
            publisher.publish(msg)

    with (tmp_path / 'timestamp.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'run', 'lumen_sensor_tests', 'timestamp_check',
             '--cloud-topic', '/lumen_test/cloud', '--duration', '3',
             '--output-dir', str(output)], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            wait(lambda: process.poll() is not None, publish, timeout=12.)
            assert process.returncode == code, (tmp_path / 'timestamp.log').read_text()
            report = json.loads(next(output.glob('*.json')).read_text())
            assert report['streams']['/lumen_test/cloud']['assessment'] == assessment
            assert 'cannot be' in next(output.glob('*.md')).read_text()
        finally:
            if process.poll() is None:
                import os
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)


def test_retime_disabled_has_no_sensor_endpoints(harness):
    add, _, reports, wait = harness
    node = add(ReceiveTimeAdapter(parameter_overrides=[
        Parameter('cloud_input', value='/lumen_test/cloud')]))
    wait(lambda: bool(reports))
    assert 'DISABLED' in reports[-1].status[0].message
    assert not node.streams
    publishers = node.get_publisher_names_and_types_by_node(node.get_name(), '/')
    assert not any(name.startswith('/lumen/retimed') for name, _ in publishers)


@pytest.mark.parametrize('kind,typ', [('cloud', PointCloud2), ('imu', Imu), ('odom', Odometry)])
def test_retime_preserves_payload_and_stops_with_input(harness, kind, typ):
    add, probe, reports, wait = harness
    node = add(ReceiveTimeAdapter(parameter_overrides=[
        Parameter('enabled', value=True),
        Parameter(kind+'_input', value='/lumen_test/raw_'+kind),
        Parameter(kind+'_output', value='/lumen_test/retimed_'+kind),
    ]))
    source = probe.create_publisher(typ, '/lumen_test/raw_'+kind, qos_profile_sensor_data)
    received = []
    probe.create_subscription(typ, '/lumen_test/retimed_'+kind, received.append, 10)
    msg = cloud(probe) if kind == 'cloud' else typ()
    msg.header.frame_id = 'sensor_frame'
    msg.header.stamp = probe.get_clock().now().to_msg()
    msg.header.stamp.sec -= 2507
    if kind == 'imu':
        msg.orientation.w = 1.0
        msg.angular_velocity.z = 0.123
        msg.linear_acceleration.z = 9.8
        msg.orientation_covariance[0] = -1.
    if kind == 'odom':
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = 12.5
        msg.pose.covariance[0] = .7
        msg.twist.twist.linear.x = .2
    original_bytes = serialize_message(msg)
    wait(lambda: bool(received), lambda: source.publish(msg))
    output = received[-1]
    assert output.header.stamp.sec > msg.header.stamp.sec + 2500
    assert serialize_message(msg) == original_bytes
    assert serialize_message(restamp(output, msg.header.stamp)) == original_bytes
    wait(lambda: bool(reports) and reports[-1].status[0].level == DiagnosticStatus.ERROR)
    count = len(received)
    deadline = time.monotonic()+.25
    wait(lambda: time.monotonic() >= deadline)
    assert len(received) == count
    values = {entry.key: json.loads(entry.value) for entry in reports[-1].status[0].values}
    assert values['timestamp_source'] == 'receive_time'
    assert values['acquisition_freshness'] == 'UNVERIFIED'
    assert values['original_offset_sec'] > 2500
    names = {name for name, _ in node.get_publisher_names_and_types_by_node(node.get_name(), '/')}
    assert '/tf' not in names and '/tf_static' not in names and '/cmd_vel' not in names


def test_retime_rejects_topic_feedback(harness):
    with pytest.raises(ValueError, match='feedback loops'):
        ReceiveTimeAdapter(parameter_overrides=[
            Parameter('enabled', value=True), Parameter('cloud_input', value='/lumen_test/cloud'),
            Parameter('cloud_output', value='/lumen_test/cloud')])


def test_retime_rejects_remapped_feedback(harness):
    with pytest.raises(ValueError, match='feedback loops'):
        ReceiveTimeAdapter(
            cli_args=['--ros-args', '-r', '/lumen_test/output:=/lumen_test/cloud'],
            parameter_overrides=[
                Parameter('enabled', value=True), Parameter('cloud_input', value='/lumen_test/cloud'),
                Parameter('cloud_output', value='/lumen_test/output')])


def test_retime_does_not_publish_on_zero_or_backward_clock(harness, monkeypatch):
    from types import SimpleNamespace
    from rclpy.time import Time
    add, probe, _, _ = harness
    node = add(ReceiveTimeAdapter(parameter_overrides=[
        Parameter('enabled', value=True), Parameter('cloud_input', value='/lumen_test/cloud')]))
    msg = cloud(probe)
    for seconds, expected in [(0, 0), (10, 1), (9, 1), (10, 1), (11, 2)]:
        monkeypatch.setattr(node, 'get_clock', lambda s=seconds: SimpleNamespace(
            now=lambda: Time(seconds=s)))
        node.receive('cloud', msg)
        assert node.streams['cloud']['published'] == expected
    assert node.streams['cloud']['receiver_clock_resets'] == 3


@pytest.mark.parametrize('enabled', [False, True])
def test_retime_launch_requires_explicit_enable(harness, tmp_path, enabled):
    _, probe, reports, wait = harness
    config = tmp_path / 'retime.yaml'
    # File alone must not enable the launch; explicit launch argument wins.
    config.write_text('receive_time_adapter:\n  ros__parameters:\n'
                      '    enabled: true\n    cloud_input: /lumen_test/cloud\n'
                      '    cloud_output: /lumen_test/retimed_cloud\n    report_rate_hz: 10.0\n')
    received = []
    probe.create_subscription(PointCloud2, '/lumen_test/retimed_cloud', received.append, 10)
    source = probe.create_publisher(PointCloud2, '/lumen_test/cloud', qos_profile_sensor_data)
    command = ['ros2', 'launch', 'lumen_sensor_tests', 'retime.launch.py', f'params_file:={config}']
    if enabled:
        command.append('enabled:=true')
    with (tmp_path/'retime.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if enabled:
                wait(lambda: bool(received), lambda: source.publish(cloud(probe)), timeout=10.)
            else:
                wait(lambda: bool(reports) and 'DISABLED' in reports[-1].status[0].message,
                     lambda: source.publish(cloud(probe)), timeout=10.)
                assert not received
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    import os
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
