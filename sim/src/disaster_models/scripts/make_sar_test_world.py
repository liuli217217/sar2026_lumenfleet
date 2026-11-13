#!/usr/bin/env python3
"""Generate the 6 m x 8 m SAR *test* world: only for evaluating the algorithm, never
for tuning it (the 5 m x 8 m arena, make_sar_arena_world.py, is the training world).

Outputs (world frame = Go2 start pose, x forward):
  worlds/sar_test_6x8.sdf                          Gazebo world
  missions/test_6x8/victims_ground_truth.yaml      true victims: for the mission monitor only
  missions/test_6x8/ground_truth_map.png + .yaml   walls + obstacles (evaluation only)
  missions/test_6x8/map.png + map.yaml             walls only (+ victims_estimated.yaml)
  --preview DIR: also DIR/sar_test_6x8_preview.sdf with saving cameras and
                 DIR/cameras.txt (see render_world.sh)

Reuses the arena generator (geometry, SDF writer, passability and hidden-victim
checks) with its own tables, plus two new things:
  sandbag    piles of sand bags (0.60 x 0.35 x 0.15 m each, brick pattern)
  spectators real people (Gazebo Fuel models) standing outside the arena, watching;
             taller than the 1.2 m walls, so the camera sees them over the walls.
             They are NOT victims: the robot must not sit down next to them.

Field frame (x right, y up, corner at 0,0). Walls are 5 cm blue foam boards, 1.2 m high:

 y=8 +-----------+-----+-----------+
     |  V2       |     |           |   room C: V2 sits against the north wall,
     | room C    |  N  |  room D  DD   behind a wall of boxes
     |  [boxes]  |corr.| [barrier]  |   room D: V3 lies along the south wall,
     |           |     | [sandbags] |   behind a water barrier + sand bags
 y=5 +----DC-----+     +-----------+
     | [sandbags]   [cone][barrier][barrier][cone]   hall with a road block
 y=3 +---DA------+     +----DB-----+   (DB is on room B's west wall)
     | room A    |  S  | [sandbags] |   room B: V1 lies behind a sand bag wall
     | (empty,   |corr.|   V1       |   room A: no victim, but cluttered
     |  clutter) |  ^  |            |
 y=0 +-----------+ Go2 +-----------+
    x=0        x=2.4  x=3.6        x=6
"""
import argparse
import math
import os

import make_sar_arena_world as A

HERE = os.path.dirname(os.path.abspath(__file__))
A.NAME = NAME = 'sar_test_6x8'
A.WORLD_OUT = os.path.join(HERE, '..', 'worlds', f'{NAME}.sdf')
A.MISSION_DIR = os.path.join(HERE, '..', 'missions', 'test_6x8')
A.FIELD_X, A.FIELD_Y = FX, FY = 6.0, 8.0
A.START_FIELD = (3.0, 0.5, 90.0)

A.WALLS = [
    ('outer_south', 0.0, 0.0, FX, 0.0, []),
    ('outer_east', FX, 0.0, FX, FY, []),
    ('outer_north', FX, FY, 0.0, FY, []),
    ('outer_west', 0.0, FY, 0.0, 0.0, []),
    ('room_a_east', 2.4, 0.0, 2.4, 3.0, []),
    ('room_a_north', 0.0, 3.0, 2.4, 3.0, [('door_a', 0.7, 1.6)]),
    ('room_b_west', 3.6, 0.0, 3.6, 3.0, [('door_b', 0.2, 1.1)]),
    ('room_b_north', 3.6, 3.0, FX, 3.0, []),
    ('room_c_south', 0.0, 5.0, 2.4, 5.0, [('door_c', 1.2, 2.1)]),
    ('room_c_east', 2.4, 5.0, 2.4, FY, []),
    ('room_d_west', 3.6, 5.0, 3.6, FY, [('door_d', 2.0, 2.9)]),
    ('room_d_south', 3.6, 5.0, FX, 5.0, []),
]
A.ROOMS = {
    'room_a': ((0.0, 2.4, 0.0, 3.0), ['door_a']),
    'room_b': ((3.6, FX, 0.0, 3.0), ['door_b']),
    'room_c': ((0.0, 2.4, 5.0, FY), ['door_c']),
    'room_d': ((3.6, FX, 5.0, FY), ['door_d']),
}

# kinds as in make_sar_arena_world.py, plus
#   sandbag (x, y, yaw, length, layers)   pile centre, along `yaw`; 0.35 m deep, 0.15 m per layer
A.OBSTACLES = [
    # hall: a standard road block (cone, two water barriers, cone) across the middle
    ('h_cone_1', 'cone', 2.20, 4.00),
    ('h_barrier_1', 'barrier', 3.00, 4.00, 0.0, 'red'),
    ('h_barrier_2', 'barrier', 4.00, 4.00, 0.0, 'white'),
    ('h_cone_2', 'cone', 4.75, 4.00),
    ('h_sandbag_1', 'sandbag', 0.75, 4.35, 90.0, 0.6, 3),
    # room A: clutter, no victim
    ('a_barrel', 'barrel', 1.00, 1.45),
    ('a_sandbag_1', 'sandbag', 1.75, 0.60, 0.0, 0.9, 3),
    ('a_cone_1', 'cone', 0.40, 2.55),
    ('a_box_1', 'box', 0.45, 0.45, 15.0, 0.45, 0.4, 0.4, 0.0),
    # room B: a 0.6 m sand bag wall from the west wall, V1 lies behind it
    ('b_sandbag_1', 'sandbag', 4.30, 1.35, 0.0, 1.4, 4),
    ('b_cone_1', 'cone', 5.65, 0.35),
    # room C: a 0.8 m wall of boxes between the door and V2
    ('c_box_1', 'box', 0.525, 6.40, 0.0, 0.65, 0.4, 0.4, 0.0),
    ('c_box_2', 'box', 1.175, 6.40, 0.0, 0.65, 0.4, 0.4, 0.0),
    ('c_box_3', 'box', 0.525, 6.40, 0.0, 0.65, 0.4, 0.4, 0.4),
    ('c_box_4', 'box', 1.175, 6.40, 0.0, 0.65, 0.4, 0.4, 0.4),
    # room D: water barrier + sand bags between the door and V3
    ('d_barrier_1', 'barrier', 4.15, 6.70, 0.0, 'red'),
    ('d_sandbag_1', 'sandbag', 4.90, 6.70, 0.0, 0.5, 4),
]

A.VICTIMS = [
    ('victim_1', 'lying', 4.85, 2.25, 0.0),      # room B, behind the sand bags, head east
    ('victim_2', 'sitting', 0.60, 7.50, -90.0),  # room C, against the north wall, behind the boxes
    ('victim_3', 'lying', 4.80, 5.75, 0.0),      # room D, along the south wall, head east
]

# Spectators (helpers and models in make_sar_arena_world.py): (model, x, y), field frame.
A.SPECTATORS = [
    ('casual_female', 2.0, -1.1),
    ('male_visitor_on_phone', 3.9, -1.4),
    ('standing_person', 7.0, 1.6),
    ('casual_female', 6.6, 6.8),        # close: 0.6 m from the east wall
    ('male_visitor_on_phone', 7.3, 4.6),
    ('standing_person', 2.6, 9.0),
    ('casual_female', -1.0, 6.2),
    ('standing_person', -1.3, 2.0),
]
SANDBAG = (0.60, 0.35, 0.15)              # one bag: length, depth, height
SAND = (0.76, 0.66, 0.46)

_box, _footprint, _model = A.obstacle_box, A.obstacle_footprint, A.obstacle_model


def obstacle_box(o):
    if o[1] == 'sandbag':
        _, _, x, y, yaw, length, layers = o
        return (x, y, yaw, length, SANDBAG[1], 0.0, layers * SANDBAG[2])
    return _box(o)


def obstacle_footprint(o):
    if o[1] == 'sandbag':
        return obstacle_box(o)[:5]
    return _footprint(o)


def obstacle_model(o):
    if o[1] != 'sandbag':
        return _model(o)
    name, _, x, y, yaw, length, layers = o
    bl, bd, bh = SANDBAG
    vis = ''
    for k in range(layers):
        # brick pattern: every other layer shifted by half a bag, bags slightly squashed
        off = 0.0 if k % 2 == 0 else bl / 2
        n = int(math.ceil((length + off) / bl))
        for i in range(n):
            a = -length / 2 - off + i * bl
            b = min(a + bl, length / 2)
            a = max(a, -length / 2)
            if b - a < 0.08:
                continue
            shade = 0.9 + 0.1 * ((i + k) % 3) / 2
            rgb = tuple(round(c * shade, 3) for c in SAND)
            # a bag: flattened box with rounded ends (two half cylinders)
            cx, w = (a + b) / 2, b - a - 0.02
            z = bh * (k + 0.5)
            vis += (f"<visual name='bag_{k}_{i}'><pose>{cx:.3f} 0 {z:.3f} 0 0 0</pose>"
                    f"<geometry><box><size>{max(w - bd * 0.5, 0.02):.3f} {bd - 0.02} {bh - 0.01}</size></box></geometry>"
                    f"{A.material(rgb)}</visual>")
            for side in (-1, 1):
                ex = cx + side * (max(w - bd * 0.5, 0.02) / 2)
                vis += (f"<visual name='bag_{k}_{i}_{side + 1}'><pose>{ex:.3f} 0 {z:.3f} 0 0 0</pose>"
                        f"<geometry><cylinder><radius>{(bd - 0.02) / 2:.3f}</radius><length>{bh - 0.01}</length></cylinder></geometry>"
                        f"{A.material(rgb)}</visual>")
    h = layers * bh
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{A.wpose(x, y, 0.0, yaw)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {h / 2} 0 0 0</pose><geometry><box><size>{length} {bd} {h}</size></box></geometry></collision>
        {vis}
      </link>
    </model>"""


A.obstacle_box, A.obstacle_footprint, A.obstacle_model = obstacle_box, obstacle_footprint, obstacle_model


def preview_cameras(out_dir):
    cams = [
        ('top', FX / 2, FY / 2 - 0.01, 13.0, 90.0, 89.9, 50.0),
        ('oblique_sw', -2.8, -3.0, 5.5, 50.0, 36.0, 72.0),
        ('oblique_ne', 8.8, 11.0, 5.5, -128.0, 36.0, 72.0),
        ('start_view', A.START_FIELD[0], A.START_FIELD[1], 0.35, A.START_FIELD[2], 0.0, 69.0),
    ]
    centres = {d: ((x0 + x1) / 2, (y0 + y1) / 2) for (x0, x1, y0, y1), ds in A.ROOMS.values() for d in ds}
    for name, (x1, y1, x2, y2) in A.doors().items():
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        rx, ry = centres[name]
        nx, ny = -(y2 - y1), x2 - x1
        if nx * (rx - mx) + ny * (ry - my) < 0:
            nx, ny = -nx, -ny
        n = math.hypot(nx, ny)
        nx, ny = nx / n, ny / n
        cams.append((f'{name}_view', mx - 0.35 * nx, my - 0.35 * ny, 0.35, math.degrees(math.atan2(ny, nx)), 0.0, 87.0))
    size = {'top': (1000, 1250)}
    sdf = ''.join(A.camera_sensor(n, x, y, z, yaw, pitch, fov, out_dir, *size.get(n, (1280, 800)))
                  for n, x, y, z, yaw, pitch, fov in cams)
    return sdf, [c[0] for c in cams]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--preview', metavar='DIR', help='also write a preview world with saving cameras')
    args = ap.parse_args()
    print('passability:')
    passable = A.check_passable()
    print('victims hidden from the doors:')
    hidden = A.check_hidden()
    print('spectators:')
    outside = A.check_spectators()
    if not (passable and hidden and outside):
        raise SystemExit('layout check failed - fix the tables (nothing written)')
    extra = A.spectators_sdf()
    header = ('5 m x 8 m SAR arena, blue foam-board walls.',
              '6 m x 8 m SAR TEST world (evaluation only), blue foam-board walls, 4 rooms, 3 victims, spectators.')
    with open(A.WORLD_OUT, 'w') as f:
        f.write(A.world_sdf(extra).replace(*header).replace('make_sar_arena_world.py', 'make_sar_test_world.py')
                .replace('<world name="sar_arena">', '<world name="sar_test">'))
    w, h = A.write_mission()
    print(f'wrote {os.path.normpath(A.WORLD_OUT)}')
    print(f'wrote {os.path.normpath(A.MISSION_DIR)}/ (maps {w} x {h} px, victims_ground_truth.yaml)')
    for v in A.VICTIMS:
        print(f'  {v[0]} ({v[1]}): world {tuple(round(c, 2) for c in A.to_world(v[2], v[3]))}')
    if args.preview:
        out = os.path.abspath(args.preview)
        os.makedirs(out, exist_ok=True)
        cams, names = preview_cameras(out)
        path = os.path.join(out, f'{NAME}_preview.sdf')
        with open(path, 'w') as f:
            f.write(A.world_sdf(extra + cams))
        with open(os.path.join(out, 'cameras.txt'), 'w') as f:
            f.write('\n'.join(names) + '\n')
        print(f'wrote {path} (cameras: {", ".join(names)})')


if __name__ == '__main__':
    main()
