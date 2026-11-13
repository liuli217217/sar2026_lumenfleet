#!/usr/bin/env python3
"""Generate the 4 m x 4 m SAR maze mission.

Outputs (all in the *world* frame, whose origin is the Go2 start pose, x forward):
  worlds/disaster_maze_4x4.sdf               Gazebo world (walls, obstacles, victims)
  missions/maze_4x4/map.png + map.yaml       prior map for the robot: walls only (ROS map_server format)
  missions/maze_4x4/victims_estimated.yaml   prior victim positions for the robot (with errors)
  missions/maze_4x4/victims_ground_truth.yaml  true victims: for the mission monitor only

Edit the tables below and re-run:  python3 make_maze_world.py

The layout is written in a "field" frame (x right, y up, field corner at 0,0):

 y=4 +----------+---------------+
     |V1  A     |   B   |  C  V3|
     |V1        |     O2|     V3|
     |        O1|       |     V4|      W1: x = 1.35, y 0 .. 2.6
     |          |       |     V4|      W2: x = 2.65, y 1.4 .. 4
     |   ^      |V2     |       |
     |  Go2     |V2       O3    |
 y=0 +----------+---------------+
    x=0       1.35  2.65       x=4

The Go2 starts at field (0.65, 0.55) facing +y (north, into the maze); that
pose becomes the world origin, so world x = field y - 0.55, world y = 0.65 - field x.
O1..O3 are obstacles that are NOT in the prior map. Rules: at most one
obstacle per corridor, against a wall, keeping every passage >= 0.8 m wide.
check_passable() verifies with the planner's robot radius that the whole
route is reachable, and refuses to write the files otherwise.

Victims lie face down with the head towards the direction the robot comes
from: in a YOLO viewpoint study this pose, seen from the head side, gave by
far the best COCO "person" scores; supine or side-lying figures were hardly
detected.
"""
import itertools
import math
import os
import random
import struct
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD_OUT = os.path.join(HERE, '..', 'worlds', 'disaster_maze_4x4.sdf')
MISSION_DIR = os.path.join(HERE, '..', 'missions', 'maze_4x4')

FIELD = 4.0
WALL_THICKNESS = 0.1
WALL_HEIGHT = 0.8
START_FIELD = (0.65, 0.55, 90.0)          # Go2 start in the field frame (x, y, yaw deg)

# (name, x1, y1, x2, y2) wall centre lines, field frame
WALLS = [
    ('wall_south', 0.0, 0.0, FIELD, 0.0),
    ('wall_north', 0.0, FIELD, FIELD, FIELD),
    ('wall_west', 0.0, 0.0, 0.0, FIELD),
    ('wall_east', FIELD, 0.0, FIELD, FIELD),
    ('maze_w1', 1.35, 0.0, 1.35, 2.6),
    ('maze_w2', 2.65, 1.4, 2.65, FIELD),
]

# Obstacles unknown to the robot: (name, shape, x, y, size...) field frame.
# box: sx, sy, sz ; cylinder: radius, height
OBSTACLES = [
    ('obstacle_1', 'box', 1.15, 2.00, 0.25, 0.25, 0.40),     # corridor A, against W1
    ('obstacle_2', 'cylinder', 2.45, 2.60, 0.12, 0.50),      # corridor B, against W2
    ('obstacle_3', 'box', 3.30, 0.35, 0.25, 0.25, 0.30),     # bottom passage B->C, south wall
]

# Passability check (same robot radius as sar_navigator's A* inflation).
PLANNER_ROBOT_RADIUS = 0.30
MIN_FREE_WIDTH = 0.8            # narrowest passage allowed on the routes [m] (Go2 is 0.31 m wide)

# victim_humanoid_small (scale 0.6): body from 0.13 m to 1.07 m along its model z
# axis, half width 0.22 m, 0.31 m high when lying face down.
VICTIM_CENTRE_OFFSET = 0.6
VICTIM_Z = 0.243
VICTIM_LENGTH = 0.94
VICTIM_HALF_WIDTH = 0.22
VICTIMS = [
    # (name, body centre x, body centre y, head direction [deg]) field frame
    ('victim_1', 0.30, 3.47, -90.0),   # corridor A, north end, west wall
    ('victim_2', 1.62, 0.53, 90.0),    # corridor B, south end, west side
    ('victim_3', 3.70, 3.47, -90.0),   # corridor C, north dead end, east wall
    ('victim_4', 3.70, 1.97, -90.0),   # corridor C, middle, east wall
]

# Prior knowledge given to the robot.
VICTIM_POSITION_ERROR = 0.3     # max error of the estimated victim positions [m]
RANDOM_SEED = 7
MAP_RESOLUTION = 0.05           # [m / pixel]
MAP_MARGIN = 0.3                # free border around the field in the map [m]


def to_world(x, y):
    """Field frame -> world frame (origin = Go2 start, x = start heading)."""
    sx, sy, syaw = START_FIELD
    a = -math.radians(syaw)
    dx, dy = x - sx, y - sy
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def yaw_to_world(deg):
    return deg - START_FIELD[2]


# ------------------------------------------------------------------ world SDF
def material(r, g, b):
    return f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'


def wall_model(name, x1, y1, x2, y2):
    (ax, ay), (bx, by) = to_world(x1, y1), to_world(x2, y2)
    length = math.hypot(bx - ax, by - ay) + WALL_THICKNESS
    size = f'{length:.3f} {WALL_THICKNESS} {WALL_HEIGHT}'
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{(ax + bx) / 2:.3f} {(ay + by) / 2:.3f} {WALL_HEIGHT / 2:.3f} 0 0 {math.atan2(by - ay, bx - ax):.4f}</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>{size}</size></box></geometry>{material(0.55, 0.55, 0.6)}</visual>
      </link>
    </model>"""


def obstacle_model(name, shape, x, y, *dims):
    wx, wy = to_world(x, y)
    if shape == 'box':
        geom, z = f'<box><size>{dims[0]} {dims[1]} {dims[2]}</size></box>', dims[2] / 2
    else:
        geom, z = f'<cylinder><radius>{dims[0]}</radius><length>{dims[1]}</length></cylinder>', dims[1] / 2
    # A visual is required: the GPU lidar and the camera only see rendered geometry.
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {z:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry>{geom}</geometry></collision>
        <visual name='visual'><geometry>{geom}</geometry>{material(0.25, 0.3, 0.35)}</visual>
      </link>
    </model>"""


def victim_include(name, cx, cy, head_deg):
    wx, wy = to_world(cx, cy)
    head = yaw_to_world(head_deg)
    ux, uy = math.cos(math.radians(head)), math.sin(math.radians(head))
    ox, oy = wx - VICTIM_CENTRE_OFFSET * ux, wy - VICTIM_CENTRE_OFFSET * uy
    return f"""
    <include>
      <uri>model://victim_humanoid_small</uri>
      <name>{name}</name>
      <pose>{ox:.3f} {oy:.3f} {VICTIM_Z} 0 1.5708 {math.radians(head):.4f}</pose>
    </include>"""


def world_sdf():
    body = ''.join(wall_model(*w) for w in WALLS)
    body += ''.join(obstacle_model(*o) for o in OBSTACLES)
    body += ''.join(victim_include(*v) for v in VICTIMS)
    return f"""<?xml version="1.0" ?>
<!-- Generated by disaster_models/scripts/make_maze_world.py - edit the script, not this file.
     World origin = Go2 start pose. -->
<sdf version="1.10">
  <world name="disaster_maze">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          {material(0.8, 0.8, 0.8)}
        </visual>
      </link>
    </model>{body}
  </world>
</sdf>
"""


# ------------------------------------------------------------------ prior map
def write_png_gray(path, img):
    """Minimal 8-bit grayscale PNG writer (no OpenCV / PIL needed)."""
    h, w = img.shape
    raw = b''.join(b'\x00' + img[r].tobytes() for r in range(h))

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(raw, 9)))
        f.write(chunk(b'IEND', b''))


def write_map():
    corners = [to_world(x, y) for x in (0.0, FIELD) for y in (0.0, FIELD)]
    x0 = min(c[0] for c in corners) - MAP_MARGIN
    y0 = min(c[1] for c in corners) - MAP_MARGIN
    x1 = max(c[0] for c in corners) + MAP_MARGIN
    y1 = max(c[1] for c in corners) + MAP_MARGIN
    w = int(math.ceil((x1 - x0) / MAP_RESOLUTION))
    h = int(math.ceil((y1 - y0) / MAP_RESOLUTION))
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    px = x0 + (cols + 0.5) * MAP_RESOLUTION
    py = y0 + (h - 1 - rows + 0.5) * MAP_RESOLUTION      # image row 0 is the top (max y)
    occupied = np.zeros((h, w), bool)
    for _, a, b, c, d in WALLS:
        (ax, ay), (bx, by) = to_world(a, b), to_world(c, d)
        vx, vy = bx - ax, by - ay
        t = np.clip(((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy), 0.0, 1.0)
        dist = np.hypot(px - (ax + t * vx), py - (ay + t * vy))
        occupied |= dist <= WALL_THICKNESS / 2 + MAP_RESOLUTION / 2
    img = np.where(occupied, 0, 255).astype(np.uint8)
    os.makedirs(MISSION_DIR, exist_ok=True)
    write_png_gray(os.path.join(MISSION_DIR, 'map.png'), img)
    with open(os.path.join(MISSION_DIR, 'map.yaml'), 'w') as f:
        f.write(f"""# Prior map of the maze (walls only; obstacles and victims are not in it).
# ROS map_server format. World frame = Go2 start pose.
image: map.png
resolution: {MAP_RESOLUTION}
origin: [{x0:.3f}, {y0:.3f}, 0.0]   # world (x, y, yaw) of the lower-left pixel
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    return w, h


def write_victims():
    rng = random.Random(RANDOM_SEED)
    est, truth = [], []
    for name, cx, cy, head in VICTIMS:
        wx, wy = to_world(cx, cy)
        r = VICTIM_POSITION_ERROR * math.sqrt(rng.random())
        a = rng.uniform(-math.pi, math.pi)
        est.append((name, wx + r * math.cos(a), wy + r * math.sin(a)))
        truth.append((name, wx, wy, yaw_to_world(head)))
    with open(os.path.join(MISSION_DIR, 'victims_estimated.yaml'), 'w') as f:
        f.write(f'# Rough victim positions given to the robot (world frame, error <= {VICTIM_POSITION_ERROR} m).\n')
        f.write('victims:\n')
        for name, x, y in est:
            f.write(f'  - {{name: {name}, x: {x:.2f}, y: {y:.2f}}}\n')
    with open(os.path.join(MISSION_DIR, 'victims_ground_truth.yaml'), 'w') as f:
        f.write('# Ground truth - for the mission monitor only, never for the robot.\n')
        f.write(f'# Body: centre (x, y), head direction [deg], length {VICTIM_LENGTH} m, '
                f'half width {VICTIM_HALF_WIDTH} m.\n')
        f.write(f'length: {VICTIM_LENGTH}\nhalf_width: {VICTIM_HALF_WIDTH}\nvictims:\n')
        for name, x, y, head in truth:
            f.write(f'  - {{name: {name}, x: {x:.3f}, y: {y:.3f}, head_deg: {head:.1f}}}\n')
    return est, truth


# ------------------------------------------------------------- passability
def occupancy_grid(include_obstacles=True, include_victims=True, res=0.05):
    """Ground-truth occupancy (world frame) of walls, obstacles and victims."""
    corners = [to_world(x, y) for x in (0.0, FIELD) for y in (0.0, FIELD)]
    x0, y0 = min(c[0] for c in corners) - 0.2, min(c[1] for c in corners) - 0.2
    x1, y1 = max(c[0] for c in corners) + 0.2, max(c[1] for c in corners) + 0.2
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    px, py = x0 + (cols + 0.5) * res, y0 + (rows + 0.5) * res
    occ = np.zeros((h, w), bool)

    def segment(ax, ay, bx, by, half):
        vx, vy = bx - ax, by - ay
        t = np.clip(((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy), 0.0, 1.0)
        return np.hypot(px - (ax + t * vx), py - (ay + t * vy)) <= half

    for _, a, b, c, d in WALLS:
        occ |= segment(*to_world(a, b), *to_world(c, d), WALL_THICKNESS / 2)
    if include_obstacles:
        for name, shape, x, y, *dims in OBSTACLES:
            wx, wy = to_world(x, y)
            if shape == 'box':
                occ |= (np.abs(px - wx) <= dims[0] / 2) & (np.abs(py - wy) <= dims[1] / 2)
            else:
                occ |= np.hypot(px - wx, py - wy) <= dims[0]
    if include_victims:
        for _, cx, cy, head in VICTIMS:
            wx, wy = to_world(cx, cy)
            a = math.radians(yaw_to_world(head))
            ux, uy = math.cos(a), math.sin(a)
            occ |= segment(wx - VICTIM_LENGTH / 2 * ux, wy - VICTIM_LENGTH / 2 * uy,
                           wx + VICTIM_LENGTH / 2 * ux, wy + VICTIM_LENGTH / 2 * uy, VICTIM_HALF_WIDTH)
    return occ, (x0, y0, res)


def distance_field(occ, res):
    h, w = occ.shape
    orows, ocols = np.nonzero(occ)
    cells = np.column_stack([a.ravel() for a in np.meshgrid(np.arange(h), np.arange(w), indexing='ij')])
    out = np.empty(len(cells))
    for i in range(0, len(cells), 2000):
        c = cells[i:i + 2000]
        out[i:i + 2000] = np.sqrt(((c[:, None, 0] - orows) ** 2 + (c[:, None, 1] - ocols) ** 2).min(axis=1))
    return out.reshape(h, w) * res


def astar(free, start, goal):
    import heapq
    h, w = free.shape
    steps = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    best, parent, heap = {start: 0.0}, {}, [(0.0, start)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            path = [cur]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1]
        for dr, dc, cost in steps:
            n = (cur[0] + dr, cur[1] + dc)
            if 0 <= n[0] < h and 0 <= n[1] < w and free[n]:
                g = best[cur] + cost
                if g < best.get(n, math.inf):
                    best[n], parent[n] = g, cur
                    heapq.heappush(heap, (g + math.hypot(n[0] - goal[0], n[1] - goal[1]), n))
    return None


def widest_path(dist, free, a, b, exclude):
    """Max over paths a->b of the min clearance (cells within `exclude` cells of
    a or b do not count). Modified Dijkstra."""
    import heapq
    h, w = free.shape
    near_end = lambda c: min(math.hypot(c[0] - a[0], c[1] - a[1]), math.hypot(c[0] - b[0], c[1] - b[1])) <= exclude
    best = {a: math.inf}
    heap = [(-math.inf, a)]
    while heap:
        neg, cur = heapq.heappop(heap)
        if cur == b:
            return -neg
        if -neg < best.get(cur, -1):
            continue
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                n = (cur[0] + dr, cur[1] + dc)
                if (dr or dc) and 0 <= n[0] < h and 0 <= n[1] < w and free[n]:
                    val = min(-neg, math.inf if near_end(n) else float(dist[n]))
                    if val > best.get(n, -1):
                        best[n] = val
                        heapq.heappush(heap, (-val, n))
    return 0.0


def check_passable():
    """Every victim must be reachable from the start (and from each other) with the
    planner's inflation, and every obstacle must leave MIN_FREE_WIDTH of space."""
    occ, (x0, y0, res) = occupancy_grid()
    dist = distance_field(occ, res)
    free = dist > PLANNER_ROBOT_RADIUS
    to_cell = lambda x, y: (int((y - y0) / res), int((x - x0) / res))

    def approach_cell(wx, wy):
        # free cell closest to the victim body = where the robot would stop
        rows, cols = np.nonzero(free)
        d = np.hypot(x0 + (cols + 0.5) * res - wx, y0 + (rows + 0.5) * res - wy)
        k = int(d.argmin())
        return (int(rows[k]), int(cols[k])), float(d[k])

    ok = True
    start = to_cell(0.0, 0.0)
    points = [('start', start)]
    for vname, cx, cy, _ in VICTIMS:
        cell, d = approach_cell(*to_world(cx, cy))
        points.append((vname, cell))
        print(f'  {vname}: closest reachable stop {d:.2f} m from the body centre line')
    for (an, a), (bn, b) in itertools.combinations(points, 2):
        if astar(free, a, b) is None:
            print(f'  NO PATH {an} -> {bn} with the planner radius {PLANNER_ROBOT_RADIUS} m')
            ok = False
    if not ok:
        return False
    # Narrowest passage on every leg: the path whose tightest point is as wide as
    # possible (maximin). The last 0.6 m at each end (start / next to a victim) is
    # not counted - the robot stops close to the victim on purpose.
    narrowest = math.inf
    for (an, a), (bn, b) in itertools.combinations(points, 2):
        bottleneck = widest_path(dist, free, a, b, exclude=0.6 / res)
        narrowest = min(narrowest, bottleneck)
        print(f'  {an} -> {bn}: narrowest passage {2 * bottleneck:.2f} m')
    status = 'ok' if 2 * narrowest >= MIN_FREE_WIDTH else 'TOO NARROW'
    print(f'  all victims reachable; narrowest passage overall {2 * narrowest:.2f} m [{status}]')
    return 2 * narrowest >= MIN_FREE_WIDTH


if __name__ == '__main__':
    if not check_passable():
        raise SystemExit('layout is not passable for the Go2 - fix OBSTACLES / VICTIMS (nothing written)')
    with open(WORLD_OUT, 'w') as f:
        f.write(world_sdf())
    w, h = write_map()
    est, truth = write_victims()
    print(f'wrote {os.path.normpath(WORLD_OUT)}')
    print(f'wrote {os.path.normpath(MISSION_DIR)}/map.png ({w} x {h} px) + map.yaml')
    for (n, ex, ey), (_, tx, ty, _) in zip(est, truth):
        print(f'  {n}: true ({tx:+.2f}, {ty:+.2f})  estimated ({ex:+.2f}, {ey:+.2f})  '
              f'error {math.hypot(ex - tx, ey - ty):.2f} m')
    for o in OBSTACLES:
        print(f'  {o[0]} ({o[1]}): world {tuple(round(v, 2) for v in to_world(o[2], o[3]))}')
