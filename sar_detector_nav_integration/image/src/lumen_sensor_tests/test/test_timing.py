import pytest

from lumen_sensor_tests.timing import TimingWindow, compare_streams


def sample(offset=.02, drift=0., jitter=0.):
    window = TimingWindow()
    for i in range(301):
        t = i*.1
        window.add(t, 1000+t, 1000+t-offset-drift*t-jitter*(i % 2), 'lidar')
    return window


def test_healthy():
    result = sample().report(30.01)
    assert result['assessment'] == 'TIMING_PLAUSIBLE'
    assert result['arrival_rate_hz'] == pytest.approx(10)
    assert result['acquisition_freshness'] == 'UNVERIFIED'


@pytest.mark.parametrize('offset', [2507., -2507.])
def test_persistent_offset(offset):
    # Use a positive epoch so large offsets don't create invalid negative stamps.
    window = TimingWindow()
    for i in range(301):
        t = i*.1
        window.add(t, 10000+t, 10000+t-offset, 'lidar')
    result = window.report(30.01)
    assert result['assessment'] == 'PERSISTENT_OFFSET'
    assert result['apparent_offset_sec'] == pytest.approx(offset)


def test_drift():
    result = sample(drift=.02).report(30.01)
    assert result['assessment'] == 'OFFSET_CHANGING'
    assert result['offset_trend_sec_per_sec'] == pytest.approx(.02)


def test_jitter():
    # Vary age without moving source time backwards between 0.1-second samples.
    assert sample(jitter=.08).report(30.01)['assessment'] == 'OFFSET_VARIABLE'


def test_stopped_arrivals():
    assert sample().report(32)['assessment'] == 'ARRIVALS_STALE'


def test_frozen_source_clock():
    window = TimingWindow()
    for i in range(30):
        window.add(i, 1000+i, 1000, 'lidar')
    result = window.report(29.1)
    assert result['assessment'] == 'SOURCE_TIMESTAMP_ANOMALY'
    assert result['repeated_timestamps'] == 29


def test_clock_step_not_misreported_as_drift():
    window = sample()
    window.add(30.1, 1035.1, 1030.1, 'lidar')
    assert window.report(30.2)['assessment'] == 'RECEIVER_CLOCK_UNSTABLE'


def test_no_data_and_insufficient_data():
    window = TimingWindow()
    assert window.report(3)['assessment'] == 'NO_DATA'
    window.add(3, 1003, 1003, 'imu')
    assert window.report(3.1)['assessment'] == 'INSUFFICIENT_DATA'


def test_zero_and_backward():
    window = TimingWindow()
    for m, stamp in enumerate([1000, 1001, 999, 0]):
        window.add(m, 1000+m, stamp, 'imu')
    result = window.report(3.1)
    assert result['invalid_timestamps'] == 1
    assert result['backward_timestamps'] == 1
    assert result['assessment'] == 'SOURCE_TIMESTAMP_ANOMALY'


def test_bounded_memory_and_cross_stream_clue():
    window = TimingWindow(max_samples=30)
    for i in range(300):
        window.add(i*.1, 10000+i*.1, 7493+i*.1, 'lidar')
    result = window.report(29.91)
    assert result['retained_samples'] == 30
    assert result['received_count'] == 300
    assert len(compare_streams({'cloud': result, 'imu': result})) == 1


def test_arrival_gap_detected():
    window = sample()
    window.add(32., 1032., 1031.98, 'lidar')
    assert window.report(32.01)['assessment'] == 'ARRIVAL_GAPS'
