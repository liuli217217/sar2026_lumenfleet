"""Standard Imu diagnostics; no assumption of an absolute heading reference."""

import math

from sensor_msgs.msg import Imu

from .analysis import imu_summary
from .common import SensorDiagnostic, run


class ImuDiagnostic(SensorDiagnostic):
    def __init__(self):
        super().__init__('imu_diagnostic', 'imu_topic', Imu)
        self.stationary = self.parameter('stationary_check', False)
        self.gyro_tolerance = self.parameter('stationary_gyro_tolerance', 0.1)
        self.gravity = self.parameter('gravity_m_s2', 9.80665)
        self.gravity_tolerance = self.parameter('gravity_tolerance', 1.5)
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.gyro_tolerance, self.gravity, self.gravity_tolerance)):
            raise ValueError('Stationary thresholds must be finite and positive')

    def report(self):
        values, issues = self.health()
        if self.latest is not None:
            readings, problems = imu_summary(self.latest, self.stationary,
                                             self.gyro_tolerance, self.gravity,
                                             self.gravity_tolerance)
            values.update(readings)
            issues.extend(problems)
        self.publish_report(values, issues, error=self.latest is None)


def main(args=None):
    run(ImuDiagnostic, args)
