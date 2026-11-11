"""Stationary operator-assisted direction check; never publishes motion."""

from .analysis import orientation_candidate
from .common import run
from .lidar_diagnostic import LidarDiagnostic


class LidarOrientationCheck(LidarDiagnostic):
    def __init__(self):
        super().__init__('lidar_orientation_check')
        self.expected = self.parameter('expected_direction', 'front')
        if self.expected not in ('front', 'left', 'right', 'rear'):
            raise ValueError('expected_direction must be front, left, right or rear')
        self.min_points = self.parameter('orientation_min_points', 3)
        self.range_band = self.parameter('orientation_range_band', 0.1)
        if self.min_points < 1 or not 0 < self.range_band <= 1.0:
            raise ValueError('Invalid orientation point count or range band')

    def report(self):
        values, issues, points, error = self.inspect()
        values['expected_direction'] = self.expected
        values['operator_confirmation_required'] = True
        if points is None or not values.get('fresh', False):
            values['candidate'] = {'direction': 'UNKNOWN', 'reason': 'Unavailable or stale data'}
        else:
            values['candidate'] = orientation_candidate(points, self.min_points, self.range_band)
        measured = values['candidate']['direction']
        values['comparison'] = ('UNKNOWN' if measured == 'UNKNOWN' else
                                'MATCH' if measured == self.expected else 'MISMATCH')
        if values['comparison'] != 'MATCH':
            issues.append('Orientation candidate ' + values['comparison'])
        values['orientation_table_row'] = f'| {self.expected} | {measured} | operator review |'
        self.publish_report(values, issues, error)


def main(args=None):
    run(LidarOrientationCheck, args)
