"""DETECTOR_NAV_INTEGRATION: persistent registry and approach geometry, no ROS."""
import json
import math
import os
from pathlib import Path


class Registry:
    STATES = {'CANDIDATE', 'CONFIRMED', 'IN_PROGRESS', 'COMPLETED', 'DEFERRED'}

    def __init__(self, path, mission_id, min_hits=3, min_span=.4, merge_radius=.6):
        self.path = Path(path)
        self.mission_id = mission_id
        self.min_hits, self.min_span, self.radius = min_hits, min_span, merge_radius
        self.records = []
        if self.path.exists():
            data = json.loads(self.path.read_text())
            if data.get('schema_version') != 1 or data.get('mission_id') != mission_id:
                raise ValueError('Ledger belongs to a different mission/map or schema')
            self.records = data['targets']
            if len({r['id'] for r in self.records}) != len(self.records):
                raise ValueError('Duplicate ledger IDs')
            for r in self.records:
                if r['state'] not in self.STATES or not all(math.isfinite(x) for x in r['position']):
                    raise ValueError('Invalid ledger record')
                if r['state'] == 'IN_PROGRESS':
                    r.update(state='DEFERRED', reason='process restarted during rescue', next_retry_ns=0)
        self.save()  # fail early if the ledger cannot be persisted

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix+'.tmp')
        with temporary.open('w') as stream:
            json.dump(dict(schema_version=1, mission_id=self.mission_id, targets=self.records),
                      stream, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.path)

    def observe(self, observations, now_ns, max_age=2.):
        changed = False
        for observation in observations:
            if observation.get('class_id') != 'mannequin':
                continue
            point = observation['position']
            confidence = float(observation.get('confidence', 0.))
            if not math.isfinite(confidence) or not 0 <= confidence <= 1:
                continue
            stamp = int(observation['last_stamp_ns'])
            if len(point) != 3 or not all(math.isfinite(x) for x in point):
                continue
            if stamp <= 0 or not 0 <= (now_ns-stamp)/1e9 <= max_age:
                continue
            closest = min(self.records, key=lambda r: math.dist(r['position'][:2], point[:2]), default=None)
            if closest is None or math.dist(closest['position'][:2], point[:2]) > self.radius:
                closest = dict(id=max((r['id'] for r in self.records), default=-1)+1,
                               class_id='mannequin', position=list(point), state='CANDIDATE',
                               hits=0, first_stamp_ns=stamp, last_stamp_ns=0,
                               attempts=0, next_retry_ns=0, reason='')
                self.records.append(closest)
            r = closest
            if stamp <= r['last_stamp_ns']:
                continue  # mapper publishes cumulative snapshots, not new evidence per array
            r['last_stamp_ns'] = stamp
            r['hits'] += 1
            # Keep active/completed position fixed: observations cannot move an in-flight goal.
            if r['state'] not in ('COMPLETED', 'IN_PROGRESS'):
                r['position'] = list(point)
            r['confidence'] = confidence
            if (r['state'] == 'CANDIDATE' and r['hits'] >= self.min_hits and
                    (stamp-r['first_stamp_ns'])/1e9 >= self.min_span):
                r['state'] = 'CONFIRMED'
            changed = True
        if changed:
            self.save()

    def choose(self, robot_xy, now_ns, max_attempts=2, fresh_sec=3.):
        available = [r for r in self.records if r['state'] in ('CONFIRMED', 'DEFERRED')
                     and r['attempts'] < max_attempts and now_ns >= r['next_retry_ns']
                     and 0 <= (now_ns-r['last_stamp_ns'])/1e9 <= fresh_sec]
        return min(available, key=lambda r: math.dist(robot_xy, r['position'][:2]), default=None)

    def start(self, record):
        record.update(state='IN_PROGRESS', attempts=record['attempts']+1, reason='')
        self.save()

    def defer(self, record, reason, now_ns, cooldown=30.):
        record.update(state='DEFERRED', reason=reason, next_retry_ns=now_ns+int(cooldown*1e9))
        self.save()

    def complete(self, record, now_ns):
        previous = dict(record)
        record.update(state='COMPLETED', completed_stamp_ns=now_ns, reason='simulated rescue completed',
                      completion_mode='mock')
        try:
            self.save()
        except (OSError, ValueError):
            record.clear()
            record.update(previous)
            raise


def cell(grid, wx, wy):
    """OccupancyGrid lookup including origin yaw and negative-coordinate floor."""
    info = grid.info
    if info.resolution <= 0 or len(grid.data) != info.width*info.height:
        return None
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = wx-info.origin.position.x, wy-info.origin.position.y
    x = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/info.resolution)
    y = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/info.resolution)
    if not (0 <= x < info.width and 0 <= y < info.height):
        return None
    return grid.data[y*info.width+x]


def footprint_clear(grid, x, y, radius, threshold=50):
    """Conservative circumscribed footprint against already-inflated global costmap."""
    step = grid.info.resolution
    if step <= 0:
        return False
    count = math.ceil(radius/step)
    # Include cells touching the disk (half-diagonal padding).
    for ix in range(-count-1, count+2):
        for iy in range(-count-1, count+2):
            if math.hypot(ix*step, iy*step) > radius+step*math.sqrt(.5):
                continue
            value = cell(grid, x+ix*step, y+iy*step)
            if value is None or value < 0 or value >= threshold:
                return False
    return True


def approach_candidates(grid, victim, robot, standoff=1., radius=.35):
    vx, vy = victim[:2]
    initial = math.atan2(robot[1]-vy, robot[0]-vx)
    candidates = []
    for i in range(12):
        angle = initial+i*math.tau/12
        x, y = vx+standoff*math.cos(angle), vy+standoff*math.sin(angle)
        if footprint_clear(grid, x, y, radius):
            candidates.append((x, y, math.atan2(vy-y, vx-x)))
    return sorted(candidates, key=lambda p: math.dist(p[:2], robot[:2]))


def apply_costmap_update(grid, update):
    """Apply Nav2 incremental costmap data after validating the full update."""
    if (update.header.frame_id != grid.header.frame_id or update.x < 0 or update.y < 0
            or update.width <= 0 or update.height <= 0
            or update.x+update.width > grid.info.width
            or update.y+update.height > grid.info.height
            or len(update.data) != update.width*update.height):
        raise ValueError('Invalid incremental costmap update')
    for row in range(update.height):
        start = (update.y+row)*grid.info.width+update.x
        for col in range(update.width):
            grid.data[start+col] = update.data[row*update.width+col]
