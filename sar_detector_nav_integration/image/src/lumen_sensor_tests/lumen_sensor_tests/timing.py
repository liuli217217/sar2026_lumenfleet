"""Bounded timestamp observations; apparent age is not network latency."""

from collections import deque
import numpy as np


class TimingWindow:
    def __init__(self, max_samples=100000):
        self.samples = deque(maxlen=max_samples)
        self.count = 0
        self.frames = set()
        self.zero_count = self.repeated_count = self.backward_count = 0
        self.clock_steps = 0
        self.previous = None

    def add(self, monotonic, receiver, stamp, frame):
        if self.previous is not None:
            pm, pr, ps = self.previous
            if stamp > 0 and ps > 0:
                self.repeated_count += int(stamp == ps)
                self.backward_count += int(stamp < ps)
            self.clock_steps += int(abs((receiver-pr) - (monotonic-pm)) > 0.25)
        self.zero_count += int(stamp <= 0)
        self.previous = (monotonic, receiver, stamp)
        self.samples.append(self.previous)
        self.count += 1
        if len(self.frames) < 16:
            self.frames.add(frame)

    def report(self, end, stale=1., offset_limit=.5, variation_limit=.05,
               drift_limit=.005):
        out = {'received_count': self.count, 'retained_samples': len(self.samples),
               'frames': sorted(self.frames), 'invalid_timestamps': self.zero_count,
               'repeated_timestamps': self.repeated_count,
               'backward_timestamps': self.backward_count,
               'receiver_clock_steps': self.clock_steps,
               'apparent_offset_sec': None, 'offset_p95_p05_sec': None,
               'offset_trend_sec_per_sec': None, 'arrival_rate_hz': None,
               'acquisition_freshness': 'UNVERIFIED', 'findings': []}
        findings = out['findings']
        if not self.samples:
            out['assessment'] = 'NO_DATA'
            findings.append('No messages received; inspect publisher, type, QoS and DDS settings.')
            return out
        samples = np.array(self.samples)
        mono, receiver, stamps = samples.T
        age = max(0., end-mono[-1])
        out['last_arrival_age_sec'] = age
        span = mono[-1]-mono[0]
        out['sample_span_sec'] = float(span)
        if span > 0:
            out['arrival_rate_hz'] = float((len(mono)-1)/span)
        gaps = np.diff(mono)
        out['max_arrival_gap_sec'] = float(gaps.max()) if len(gaps) else None
        if age > stale:
            findings.append('Arrivals stopped before the observation ended.')
        if len(gaps) and gaps.max() > stale:
            findings.append('Arrival gaps exceed the configured threshold.')
        if self.zero_count:
            findings.append('Zero/negative source timestamps; acquisition time unavailable.')
        if self.repeated_count:
            findings.append('Repeated source timestamps; possible duplicate samples or frozen clock.')
        if self.backward_count:
            findings.append('Source timestamps moved backwards; possible reset, reordering or replay.')
        if self.clock_steps:
            findings.append('Receiver ROS clock diverged from steady time; offset inference unreliable.')
        valid = stamps > 0
        if valid.any():
            offsets = receiver[valid]-stamps[valid]
            out['apparent_offset_sec'] = float(np.median(offsets))
            out['offset_p95_p05_sec'] = float(np.percentile(offsets, 95)-np.percentile(offsets, 5))
            out['first_source_stamp_sec'] = float(stamps[valid][0])
            out['last_source_stamp_sec'] = float(stamps[valid][-1])
            x = mono[valid]-mono[valid][0]
            if len(x) >= 3 and x[-1] >= 2:
                x = x-x.mean()
                out['offset_trend_sec_per_sec'] = float(
                    np.dot(x, offsets-offsets.mean())/np.dot(x, x))
        if self.clock_steps:
            assessment = 'RECEIVER_CLOCK_UNSTABLE'
        elif age > stale:
            assessment = 'ARRIVALS_STALE'
        elif self.zero_count or self.backward_count or self.repeated_count:
            assessment = 'SOURCE_TIMESTAMP_ANOMALY'
        elif len(mono) < 10 or span < 2:
            assessment = 'INSUFFICIENT_DATA'
        elif len(gaps) and gaps.max() > stale:
            assessment = 'ARRIVAL_GAPS'
        elif abs(out['offset_trend_sec_per_sec'] or 0) > drift_limit:
            assessment = 'OFFSET_CHANGING'
        elif out['offset_p95_p05_sec'] > variation_limit:
            assessment = 'OFFSET_VARIABLE'
        elif abs(out['apparent_offset_sec']) > offset_limit:
            assessment = 'PERSISTENT_OFFSET'
        else:
            assessment = 'TIMING_PLAUSIBLE'
        out['assessment'] = assessment
        if assessment == 'PERSISTENT_OFFSET':
            findings.append('Clock offset suspected; constant buffering delay cannot be excluded.')
        elif assessment in ('OFFSET_CHANGING', 'OFFSET_VARIABLE'):
            findings.append('Possible clock drift, changing buffering, scheduling delays or clock adjustment.')
        elif assessment == 'TIMING_PLAUSIBLE':
            findings.append('No timing anomaly detected within configured thresholds; no retiming indicated.')
        if out['apparent_offset_sec'] is not None and out['apparent_offset_sec'] < -offset_limit:
            findings.append('Source timestamps are ahead of receiver time.')
        if self.count > len(self.samples):
            findings.append('Statistics use the most recent retained samples; anomaly counters cover the full run.')
        return out


def compare_streams(streams, tolerance=.1):
    """Similarity is a clue, never proof of common physical clock ownership."""
    result = []
    names = list(streams)
    for i, left in enumerate(names):
        for right in names[i+1:]:
            a, b = streams[left], streams[right]
            if (a['assessment'] == b['assessment'] == 'PERSISTENT_OFFSET'
                    and abs(a['apparent_offset_sec']-b['apparent_offset_sec']) <= tolerance):
                result.append(f'{left} and {right} have similar apparent offsets; '
                              'a shared clock/conversion is possible, not proven.')
    return result
