import json

from lumen_sensor_tests import timestamp_check


def test_startup_failure_still_writes_actionable_report(monkeypatch, tmp_path):
    monkeypatch.setattr(timestamp_check.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(timestamp_check.rclpy, 'ok', lambda: False)

    def fail(name):
        raise RuntimeError('Configured network interface is unavailable')

    monkeypatch.setattr(timestamp_check, 'Node', fail)
    assert timestamp_check.main([
        '--cloud-topic', '/test/cloud', '--output-dir', str(tmp_path)]) == 2
    report = json.loads(next(tmp_path.glob('*.json')).read_text())
    assert report['streams']['/test/cloud']['assessment'] == 'DDS_STARTUP_ERROR'
    assert 'ip -br address' in next(tmp_path.glob('*.md')).read_text()
