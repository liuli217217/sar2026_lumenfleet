"""Offline-capable, passive time audit. No clock changes or sensor restamping."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from .timing import TimingWindow, compare_streams


GUIDANCE = '''
## How to act on the findings

- DDS_STARTUP_ERROR: ROS could not initialize communication. Check the selected
  interface against `ip -br address`, DDS configuration and middleware libraries.
  This is not evidence of a sensor timestamp problem. On the local setup,
  set UNITREE_NETWORK_INTERFACE to the verified robot-facing interface before
  sourcing setup.sh. Do not guess a replacement or overwrite competition DDS settings.
- TIMING_PLAUSIBLE: keep original timestamps; no retiming indicated. This is
  not proof of acquisition accuracy or permission to enable navigation.
- NO_DATA: check `ros2 topic info --verbose TOPIC`. A discovered publisher may
  be idle. Verify the topic type, DDS domain/interface and QoS in this terminal.
- ARRIVALS_STALE / ARRIVAL_GAPS: inspect sensor health, transport and receiver
  load. Do not restamp missing, buffered or frozen data to make it look healthy.
- SOURCE_TIMESTAMP_ANOMALY: inspect resets, repeated samples, packet reordering
  and bag playback. Zero timestamps cannot be used to estimate clock offset.
- RECEIVER_CLOCK_UNSTABLE: check clock steps, paused/scaled simulated time and
  `use_sim_time`. Repeat with consistent clocks; do not infer source clock drift.
- PERSISTENT_OFFSET: compare UTC clocks on the receiver and the actual sensor
  publisher. A constant clock offset and constant buffering delay cannot be
  distinguished from these observations alone.
- OFFSET_CHANGING / OFFSET_VARIABLE: check both clocks, receiver load, sensor
  queues and transport; a growing age is not automatically clock drift.
- INSUFFICIENT_DATA: increase observation duration or check a low-rate source.

## Read-only troubleshooting commands (no internet or Codex required)

On the receiver and, via an existing authorized shell, the publisher computer:

```bash
hostname
date -u '+%Y-%m-%d %H:%M:%S UTC'
timedatectl status
ip -br address
```

On the receiver, using the same ROS environment as this check:

```bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
ros2 topic info --verbose TOPIC
ros2 topic echo TOPIC --field header --once --qos-reliability best_effort
```

Replace TOPIC with a topic listed in the report. DDS publisher GUID prefixes
are included when discovery provides them; they are identifiers, not IP
addresses. If the source timestamp disagrees with the computer reached by SSH,
identify the actual publisher/process before changing anything. A process may
use a sensor clock or a cached boot-time offset rather than its host wall clock.

## Correction and verification checklist

1. Keep the robot stationary and stop time-sensitive mapping/recording before
   any clock adjustment. Preserve this original report.
2. Establish a trusted reference and identify the computer/process assigning
   the sensor timestamp. NTP enabled does not mean the clock is synchronized.
3. If host clocks disagree, arrange approved local time synchronization to the
   trusted reference (for example a configured Jetson NTP server). Robot
   internet is not required for local NTP. Do not assume a server is installed
   or modify unrelated DDS configuration. Manual setting is approximate and
   does not provide ongoing synchronization.
4. Rerun this exact check after correcting the source. If the offset remains,
   investigate sensor time conversion/cached offsets. Restart only an identified
   service after confirming its effects and obtaining operator approval.
5. Only when timing is plausible, verify timestamped TF and cross-sensor
   alignment separately. Similar median offsets do not establish synchronization.

A receive-time adapter is a separate, explicit experimental workaround. It is
not enabled by this command. See docs/COMPETITION_TIMESTAMP_GUIDE.md and
`ros2 launch lumen_sensor_tests retime.launch.py --show-args` for configuration.
It cannot recover acquisition time
or prove freshness and should not silently feed LiDAR/IMU fusion. Preserve raw
topics and avoid duplicate TF authorities. The existing retime_utlidar.py also
retimes odometry and broadcasts TF; do not launch it merely to clear a warning.
'''


def markdown(report):
    lines = ['# Sensor timestamp check', '',
             f"UTC report time: {report['created_utc']}",
             f"Observation: {report['elapsed_sec']:.2f} seconds; interrupted: {report['interrupted']}",
             '', '**Apparent timestamp age is not a measurement of transmission latency.**',
             'Acquisition freshness remains unverified from subscriber observations alone.', '']
    for topic, result in report['streams'].items():
        lines.extend([f'## {topic}: {result["assessment"]}', '',
                      '```json', json.dumps(result, indent=2, allow_nan=False), '```', ''])
    lines.extend(report['cross_stream_observations'])
    lines.append(GUIDANCE)
    return '\n'.join(lines)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cloud-topic', action='append', default=[])
    parser.add_argument('--imu-topic', action='append', default=[])
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--stale-timeout', type=float, default=1.)
    parser.add_argument('--offset-limit', type=float, default=.5)
    parser.add_argument('--variation-limit', type=float, default=.05)
    parser.add_argument('--drift-limit', type=float, default=.005)
    parser.add_argument('--output-dir', default='reports')
    options, ros_args = parser.parse_known_args(args)
    for key in ('duration', 'stale_timeout', 'offset_limit', 'variation_limit', 'drift_limit'):
        value = getattr(options, key)
        if not math.isfinite(value) or value <= 0:
            parser.error(f'{key} must be finite and positive')
    if options.duration > 600:
        parser.error('duration is limited to 600 seconds')
    topics = [(t, PointCloud2) for t in options.cloud_topic] + [(t, Imu) for t in options.imu_topic]
    if not topics or len(topics) > 8 or len({t for t, _ in topics}) != len(topics):
        parser.error('Supply 1–8 distinct --cloud-topic / --imu-topic arguments')
    if any(not t.startswith('/') for t, _ in topics):
        parser.error('Use absolute topic names beginning with /')
    output = Path(options.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    suffix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    prefix = output / ('timestamp_' + suffix)
    # Check output permissions before waiting for a sampling period.
    json_path = prefix.with_suffix('.json')
    with json_path.open('x') as stream:
        stream.write('{"status": "sampling_in_progress"}\n')
    node = None
    try:
        rclpy.init(args=ros_args)
        node = Node('lumen_timestamp_check')
        windows = {topic: TimingWindow() for topic, _ in topics}
        for topic, message_type in topics:
            def receive(msg, topic=topic):
                windows[topic].add(time.monotonic(), node.get_clock().now().nanoseconds*1e-9,
                                   msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                                   msg.header.frame_id)
            node.create_subscription(message_type, topic, receive, qos_profile_sensor_data)
    except Exception as exc:
        error_report = {
            'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
            'elapsed_sec': 0., 'interrupted': False, 'thresholds': vars(options),
            'streams': {topic: {'assessment': 'DDS_STARTUP_ERROR', 'received_count': 0,
                                'error': str(exc)} for topic, _ in topics},
            'cross_stream_observations': [],
        }
        json_path.write_text(json.dumps(error_report, indent=2)+'\n')
        prefix.with_suffix('.md').write_text(markdown(error_report))
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'DDS_STARTUP_ERROR: {exc}\nReports: {json_path} and {prefix.with_suffix(".md")}', flush=True)
        return 2
    print(f'Observing {len(topics)} topic(s) for {options.duration:g}s; no control commands.', flush=True)
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic()-started < options.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        end = time.monotonic()
        streams = {}
        for topic, window in windows.items():
            result = window.report(end, options.stale_timeout, options.offset_limit,
                                   options.variation_limit, options.drift_limit)
            try:
                result['publishers'] = [
                    {'node': info.node_name, 'namespace': info.node_namespace,
                     'type': info.topic_type,
                     'gid': '.'.join(f'{int(v):02x}' for v in info.endpoint_gid),
                     'reliability': str(info.qos_profile.reliability)}
                    for info in node.get_publishers_info_by_topic(topic)]
            except Exception as exc:
                result['discovery_error'] = str(exc)
            streams[topic] = result
        report = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
                  'elapsed_sec': end-started, 'interrupted': interrupted,
                  'use_sim_time': node.get_parameter('use_sim_time').value,
                  'thresholds': vars(options),
                  'environment': {k: os.environ.get(k) for k in
                                  ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION')},
                  'streams': streams, 'cross_stream_observations': compare_streams(streams)}
        try:
            json_path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            prefix.with_suffix('.md').write_text(markdown(report))
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    for topic, result in streams.items():
        print(f'{topic}: {result["assessment"]}; apparent offset={result["apparent_offset_sec"]} s')
    print(f'Reports: {json_path} and {prefix.with_suffix(".md")}')
    # Distinguish usable observations from inconclusive/anomalous outcomes for shell scripts.
    return 0 if not interrupted and all(
        r['assessment'] == 'TIMING_PLAUSIBLE' for r in streams.values()) else 2
