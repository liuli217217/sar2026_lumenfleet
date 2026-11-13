#!/usr/bin/env python3
"""Generate the 5 m x 8 m SAR arena (unknown-map mission).

Outputs (all in the *world* frame, whose origin is the Go2 start pose, x forward):
  worlds/sar_arena_5x8.sdf                        Gazebo world (training), with spectators
  worlds/training_map_withoutAud.sdf              --no-audience: the same without spectators
  missions/arena_5x8/victims_ground_truth.yaml    true victims: for the mission monitor only
  missions/arena_5x8/ground_truth_map.png + .yaml walls + obstacles (evaluation only)
  missions/arena_5x8/map.png + map.yaml           walls only + victims_estimated.yaml: lets the
                                                  known-map go2_sar run here as a baseline
  --preview DIR: also DIR/sar_arena_5x8_preview.sdf, the same world plus cameras that save
                 overview / door-view images (see render_sar_arena.sh)

The 4 m x 4 m maze (make_maze_world.py, disaster_maze_4x4.sdf) is kept as is.
Edit the tables below and re-run:  python3 make_sar_arena_world.py

Field frame (x right, y up, corner at 0,0). Walls are 5 cm blue foam boards, 1.2 m high:

 y=8 +---------------------------+-------+
     |            V2             |   V3  |   V2 sits against the north wall of room B,
     |  room B     [boxes]       |   |   |   V3 lies at the dead end of the east corridor
     |  (2 doors)                |       |
 y=5.2+---D1-----------D2--------+       |   D1: x 0.4 .. 1.3, D2: x 2.1 .. 3.0
     | V4      [cone][barrier][barrier]  |   middle zone with a road-block line
 y=3.0+-----------+------DA--------------+   DA: x 3.0 .. 3.9
     |   ^  [barrel] | room A (1 door)   |
     |  Go2       |    [boxes]          |
     |            |       V1            |   V1 lies along the south wall of room A
 y=0 +------------+---------------------+
    x=0         x=2.0                 x=5

Each room has obstacles in its middle so that no part of its victim can be seen from
anywhere in the door opening (checked in 3D with ray casting, check_hidden()).
check_passable() verifies with the planner's robot radius that every victim can be
reached and that no passage on the way is narrower than MIN_FREE_WIDTH.
"""
import argparse
import itertools
import math
import os
import random

import numpy as np

from make_maze_world import astar, distance_field, widest_path, write_png_gray

HERE = os.path.dirname(os.path.abspath(__file__))
NAME = 'sar_arena_5x8'
NO_AUDIENCE_NAME = 'training_map_withoutAud'
WORLD_OUT = os.path.join(HERE, '..', 'worlds', f'{NAME}.sdf')
MISSION_DIR = os.path.join(HERE, '..', 'missions', 'arena_5x8')

FIELD_X, FIELD_Y = 5.0, 8.0
WALL_THICKNESS = 0.05           # foam board
WALL_HEIGHT = 1.2
BOARD_WIDTH = 0.6               # seams between boards (visual only)
FOAM_BLUE = (0.16, 0.42, 0.80)
START_FIELD = (1.1, 0.7, 90.0)  # Go2 start in the field frame (x, y, yaw deg)

# (name, x1, y1, x2, y2, doors) wall centre lines, field frame. A door is
# (name, s0, s1): an opening from s0 to s1 metres along the wall from (x1, y1).
WALLS = [
    ('outer_south', 0.0, 0.0, FIELD_X, 0.0, []),
    ('outer_east', FIELD_X, 0.0, FIELD_X, FIELD_Y, []),
    ('outer_north', FIELD_X, FIELD_Y, 0.0, FIELD_Y, []),
    ('outer_west', 0.0, FIELD_Y, 0.0, 0.0, []),
    ('room_a_west', 2.0, 0.0, 2.0, 3.0, []),
    ('room_a_north', 2.0, 3.0, FIELD_X, 3.0, [('door_a', 1.0, 1.9)]),
    ('room_b_south', 0.0, 5.2, 3.3, 5.2, [('door_b1', 0.4, 1.3), ('door_b2', 2.1, 3.0)]),
    ('room_b_east', 3.3, 5.2, 3.3, FIELD_Y, []),
]
# room name -> (x0, x1, y0, y1) interior bounds and its doors
ROOMS = {
    'room_a': ((2.0, FIELD_X, 0.0, 3.0), ['door_a']),
    'room_b': ((0.0, 3.3, 5.2, FIELD_Y), ['door_b1', 'door_b2']),
}

# Obstacles, field frame. kinds:
#   box     (x, y, yaw, sx, sy, sz, z0)  cardboard box (stacks: same x, y, z0 = height below)
#   barrel  (x, y)                       construction barrel, 0.63 m x 0.96 m
#   cone    (x, y)                       traffic cone, 0.70 m
#   barrier (x, y, yaw, colour)          plastic water-filled road barrier, 1.0 x 0.42 x 0.65 m
OBSTACLES = [
    # room A, middle: a 0.8 m high wall of boxes between the door and V1
    ('a_box_1', 'box', 3.175, 1.90, 0.0, 0.65, 0.4, 0.4, 0.0),
    ('a_box_2', 'box', 3.825, 1.90, 0.0, 0.65, 0.4, 0.4, 0.0),
    ('a_box_3', 'box', 3.175, 1.90, 0.0, 0.65, 0.4, 0.4, 0.4),
    ('a_box_4', 'box', 3.825, 1.90, 0.0, 0.65, 0.4, 0.4, 0.4),
    # room B, middle: three box stacks between both doors and V2
    ('b_box_1', 'box', 1.22, 6.65, 0.0, 0.48, 0.4, 0.4, 0.0),
    ('b_box_2', 'box', 1.22, 6.65, 0.0, 0.48, 0.4, 0.4, 0.4),
    ('b_box_3', 'box', 1.70, 6.65, 0.0, 0.48, 0.4, 0.4, 0.0),
    ('b_box_4', 'box', 1.70, 6.65, 0.0, 0.48, 0.4, 0.4, 0.4),
    ('b_box_5', 'box', 1.70, 6.65, 0.0, 0.48, 0.4, 0.4, 0.8),
    ('b_box_6', 'box', 2.18, 6.65, 0.0, 0.48, 0.4, 0.4, 0.0),
    ('b_box_7', 'box', 2.18, 6.65, 0.0, 0.48, 0.4, 0.4, 0.4),
    # middle zone: a road-block line (cone + two water barriers) splits it into two lanes
    ('m_cone_1', 'cone', 1.85, 4.10),
    ('m_barrier_1', 'barrier', 2.60, 4.10, 0.0, 'red'),
    ('m_barrier_2', 'barrier', 3.60, 4.10, 0.0, 'white'),
    # start corridor
    ('s_barrel', 'barrel', 1.50, 2.05),
    ('s_cone_1', 'cone', 0.30, 2.70),
    ('s_box_1', 'box', 0.30, 1.40, 0.0, 0.4, 0.4, 0.3, 0.0),
    # east corridor
    ('e_box_1', 'box', 4.72, 5.70, 10.0, 0.45, 0.4, 0.4, 0.0),
    # room A corner
    ('a_cone_1', 'cone', 4.75, 2.75),
]

# Victims: Rescue Randy mannequins (1.82 m adult, CC-BY 4.0 / CC0, Open Robotics).
#   lying:   body centre (x, y), head direction [deg]; A-pose, arms 1.36 m across
#   sitting: footprint centre (x, y), facing direction [deg]; back to a wall
VICTIMS = [
    ('victim_1', 'lying', 3.58, 0.72, 0.0),      # room A, along the south wall, behind the boxes
    ('victim_2', 'sitting', 1.70, 7.50, -90.0),  # room B, against the north wall, behind the boxes
    ('victim_3', 'lying', 4.15, 7.03, 90.0),     # east corridor, dead end, head to the wall
    ('victim_4', 'sitting', 0.50, 3.85, 0.0),    # middle zone, against the west wall
]
LYING_LENGTH, LYING_HALF_WIDTH, LYING_HEIGHT = 1.82, 0.20, 0.36   # torso + legs (arms are thin)
LYING_ARM_SPAN = 1.36
SITTING_SIZE = (0.95, 0.75, 0.72)   # (depth along facing, width, height)

# Spectators: (model, x, y) real people standing outside the arena, field frame; they
# face the arena centre. Taller than the walls: the camera sees their heads and
# shoulders over the walls, and they are NOT victims (Gazebo Fuel models).
SPECTATORS = [
    ('standing_person', 1.0, -1.2),
    ('casual_female', 3.3, -1.0),
    ('male_visitor_on_phone', 6.1, 1.4),
    ('standing_person', 6.4, 4.7),
    ('casual_female', 5.7, 6.9),        # close: 0.7 m from the east wall
    ('male_visitor_on_phone', 2.3, 9.1),
    ('casual_female', -1.1, 3.4),
    ('standing_person', -0.8, 6.5),     # close: 0.8 m from the west wall
]
# mesh, scale, extra yaw [deg] so that the model faces +x (measured from renders), collision box (sx, sy, h)
# in the mesh frame
SPECTATOR_MODELS = {
    'casual_female': ('model://casual_female/meshes/casual_female.dae', 1.0, 90.0, (0.76, 0.33, 1.77)),
    'male_visitor_on_phone': ('model://male_visitor_on_phone/meshes/MaleVisitorStatic.obj', 1.0, 0.0,
                              (0.35, 0.6, 1.8)),
    'standing_person': ('model://standing_person/meshes/standing.dae', 1.0, 90.0, (0.6, 0.35, 1.8)),
}

# Checks.
PLANNER_ROBOT_RADIUS = 0.30
MIN_FREE_WIDTH = 0.8            # narrowest passage on the routes [m] (Go2 is 0.31 m wide)
STOP_DISTANCE = 0.75            # the robot must be able to stand this close to every victim
CAMERA_HEIGHTS = (0.30, 0.45)   # Go2 camera / L1 lidar heights for the door-visibility check

# Prior knowledge for the known-map baseline only (the unknown-map mission ignores it).
VICTIM_POSITION_ERROR = 0.3
RANDOM_SEED = 11
MAP_RESOLUTION = 0.05
MAP_MARGIN = 0.3


def to_world(x, y):
    """Field frame -> world frame (origin = Go2 start, x = start heading)."""
    sx, sy, syaw = START_FIELD
    a = -math.radians(syaw)
    dx, dy = x - sx, y - sy
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def yaw_to_world(deg):
    return deg - START_FIELD[2]


def unit(deg):
    return math.cos(math.radians(deg)), math.sin(math.radians(deg))


# ------------------------------------------------------------- geometry model
# Everything solid is described as oriented boxes (field frame):
#   (cx, cy, yaw_deg, sx, sy, z0, z1)
def wall_segments():
    """Solid wall pieces (name, x1, y1, x2, y2) after cutting out the doors."""
    out = []
    for name, x1, y1, x2, y2, doors in WALLS:
        length = math.hypot(x2 - x1, y2 - y1)
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        cuts = [0.0] + [s for _, s0, s1 in sorted(doors, key=lambda d: d[1]) for s in (s0, s1)] + [length]
        for i in range(0, len(cuts), 2):
            a, b = cuts[i], cuts[i + 1]
            if b - a > 1e-6:
                out.append((f'{name}_{i // 2}', x1 + a * ux, y1 + a * uy, x1 + b * ux, y1 + b * uy))
    return out


def doors():
    """name -> (x1, y1, x2, y2) of every door opening."""
    out = {}
    for _, x1, y1, x2, y2, ds in WALLS:
        length = math.hypot(x2 - x1, y2 - y1)
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        for name, s0, s1 in ds:
            out[name] = (x1 + s0 * ux, y1 + s0 * uy, x1 + s1 * ux, y1 + s1 * uy)
    return out


def wall_boxes():
    boxes = []
    for _, x1, y1, x2, y2 in wall_segments():
        length = math.hypot(x2 - x1, y2 - y1)
        # walls overlap by half a thickness at the corners
        boxes.append(((x1 + x2) / 2, (y1 + y2) / 2, math.degrees(math.atan2(y2 - y1, x2 - x1)),
                      length + WALL_THICKNESS, WALL_THICKNESS, 0.0, WALL_HEIGHT))
    return boxes


CONE_HEIGHT, CONE_SCALE = 0.70, 6.4          # mesh is 0.109 m high
BARREL_SCALE, BARREL_RADIUS, BARREL_HEIGHT = 0.62, 0.315, 0.96
BARRIER_SIZE = (1.0, 0.42, 0.65)


def obstacle_box(o):
    """Solid oriented box of an obstacle (cylinders as their inscribed square, which
    under-estimates occlusion and so keeps the visibility check conservative)."""
    name, kind, x, y = o[:4]
    if kind == 'box':
        yaw, sx, sy, sz, z0 = o[4:]
        return (x, y, yaw, sx, sy, z0, z0 + sz)
    if kind == 'barrel':
        s = BARREL_RADIUS * math.sqrt(2.0)
        return (x, y, 0.0, s, s, 0.0, BARREL_HEIGHT)
    if kind == 'cone':
        return (x, y, 0.0, 0.14, 0.14, 0.0, CONE_HEIGHT)   # occludes only near its axis
    if kind == 'barrier':
        return (x, y, o[4], BARRIER_SIZE[0], BARRIER_SIZE[1], 0.0, BARRIER_SIZE[2])
    raise ValueError(kind)


def obstacle_footprint(o):
    """(cx, cy, yaw, sx, sy) or ('circle', cx, cy, r) used for the occupancy grid."""
    name, kind, x, y = o[:4]
    if kind == 'barrel':
        return ('circle', x, y, BARREL_RADIUS)
    if kind == 'cone':
        return ('circle', x, y, 0.16)
    b = obstacle_box(o)
    return b[:5]


def victim_body(v):
    """Oriented box of the victim's body: (cx, cy, yaw, sx, sy, z0, z1), sx along the
    head (lying) / facing (sitting) direction."""
    name, pose, x, y, heading = v
    if pose == 'lying':
        return (x, y, heading, LYING_LENGTH, 2 * LYING_HALF_WIDTH, 0.0, LYING_HEIGHT)
    d, w, h = SITTING_SIZE
    return (x, y, heading, d, w, 0.0, h)


def victim_arms(v):
    """Arm segments ((x1, y1), (x2, y2)) of a lying victim, shoulder -> hand: the
    shoulders are 0.37 m from the head end and 0.2 m out, the hands 0.97 m and 0.68 m."""
    if v[1] != 'lying':
        return []
    cx, cy, yaw = victim_body(v)[:3]
    ux, uy = unit(yaw)
    vx, vy = -uy, ux
    segs = []
    for side in (-1, 1):
        ends = []
        for along, out in ((LYING_LENGTH / 2 - 0.37, 0.2), (LYING_LENGTH / 2 - 0.97, LYING_ARM_SPAN / 2)):
            ends.append((cx + along * ux + side * out * vx, cy + along * uy + side * out * vy))
        segs.append(tuple(ends))
    return segs


def victim_samples(v, step=0.1):
    """Surface sample points (N, 3) of a victim, including the spread arms."""
    cx, cy, yaw, sx, sy, z0, z1 = victim_body(v)
    ux, uy = unit(yaw)
    vx, vy = -uy, ux
    pts = []
    for a in np.arange(-sx / 2, sx / 2 + 1e-6, step):
        for b in np.arange(-sy / 2, sy / 2 + 1e-6, step):
            for z in np.arange(z0 + 0.05, z1 + 1e-6, step):
                pts.append((cx + a * ux + b * vx, cy + a * uy + b * vy, z))
    for (ax, ay), (bx, by) in victim_arms(v):
        for t in np.linspace(0.0, 1.0, 8):
            pts.append((ax + t * (bx - ax), ay + t * (by - ay), 0.06))
    return np.array(pts)


# ------------------------------------------------------------------ world SDF
def material(rgb, extra=''):
    r, g, b = rgb
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular>{extra}</material>')


def wpose(x, y, z, yaw_deg, roll=0.0, pitch=0.0):
    wx, wy = to_world(x, y)
    return f'{wx:.3f} {wy:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {math.radians(yaw_to_world(yaw_deg)):.4f}'


def wall_model(name, x1, y1, x2, y2):
    length = math.hypot(x2 - x1, y2 - y1) + WALL_THICKNESS
    yaw = math.degrees(math.atan2(y2 - y1, x2 - x1))
    size = f'{length:.3f} {WALL_THICKNESS} {WALL_HEIGHT}'
    # darker seam strips on both faces every BOARD_WIDTH (painted on, so no gaps for the lidar)
    seams = ''
    n = int(length / BOARD_WIDTH)
    for k in range(1, n + 1):
        s = -length / 2 + k * BOARD_WIDTH
        if s >= length / 2 - 0.05:
            break
        for side in (-1, 1):
            seams += (f"<visual name='seam_{k}_{side + 1}'><pose>{s:.3f} {side * (WALL_THICKNESS / 2 + 0.001):.4f} 0 0 0 0</pose>"
                      f"<geometry><box><size>0.008 0.002 {WALL_HEIGHT}</size></box></geometry>"
                      f"{material((0.08, 0.22, 0.45))}<cast_shadows>false</cast_shadows></visual>")
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose((x1 + x2) / 2, (y1 + y2) / 2, WALL_HEIGHT / 2, yaw)}</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>{size}</size></box></geometry>{material(FOAM_BLUE)}</visual>
        {seams}
      </link>
    </model>"""


def mesh_visual(uri, scale, albedo=None):
    """Mesh visual; `albedo` replaces the COLLADA material (the cone and barrel meshes
    are emissive and very shiny, which washes their colours out under ogre2)."""
    mat = ''
    if albedo:
        mat = ('<material><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
               f'<pbr><metal><albedo_map>{albedo}</albedo_map><metalness>0</metalness>'
               '<roughness>0.8</roughness></metal></pbr></material>')
    return (f"<visual name='visual'><geometry><mesh><uri>{uri}</uri>"
            f"<scale>{scale[0]} {scale[1]} {scale[2]}</scale></mesh></geometry>{mat}</visual>")


def obstacle_model(o):
    name, kind, x, y = o[:4]
    if kind == 'box':
        yaw, sx, sy, sz, z0 = o[4:]
        # mesh is a 0.397 m cube with its origin at the bottom centre
        scale = (sx / 0.397, sy / 0.397, sz / 0.397)
        geom = f'<box><size>{sx} {sy} {sz}</size></box>'
        return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose(x, y, z0, yaw)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {sz / 2} 0 0 0</pose><geometry>{geom}</geometry></collision>
        {mesh_visual('model://cardboard_box/meshes/cardboard_box.dae', scale)}
      </link>
    </model>"""
    if kind == 'barrel':
        s = BARREL_SCALE
        return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose(x, y, 0.0, 0.0)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {BARREL_HEIGHT / 2} 0 0 0</pose>
          <geometry><cylinder><radius>{BARREL_RADIUS}</radius><length>{BARREL_HEIGHT}</length></cylinder></geometry></collision>
        {mesh_visual('model://construction_barrel/meshes/construction_barrel.dae', (s, s, s),
                     'model://construction_barrel/materials/textures/Construction_Barrel_Diffuse.png')}
      </link>
    </model>"""
    if kind == 'cone':
        s = CONE_SCALE
        return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose(x, y, 0.0, 0.0)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {CONE_HEIGHT / 2} 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>{CONE_HEIGHT}</length></cylinder></geometry></collision>
        {mesh_visual('model://construction_cone/meshes/construction_cone.dae', (s, s, s),
                     'model://construction_cone/materials/textures/Construction_Cone_Diffuse.png')}
      </link>
    </model>"""
    if kind == 'barrier':
        yaw, colour = o[4:]
        rgb = (0.85, 0.08, 0.06) if colour == 'red' else (0.93, 0.93, 0.90)
        lx, ly, lz = BARRIER_SIZE
        # base, tapered body (two steps) and a filler cap: the usual water-filled barrier
        parts = [(0.0, lx, ly, 0.22, 0.11), (0.0, lx * 0.94, ly * 0.70, 0.25, 0.345),
                 (0.0, lx * 0.90, ly * 0.50, 0.18, 0.56)]
        vis = ''.join(f"<visual name='v{i}'><pose>{dx} 0 {zc} 0 0 0</pose><geometry><box><size>{sx:.3f} {sy:.3f} {sz}</size></box></geometry>{material(rgb)}</visual>"
                      for i, (dx, sx, sy, sz, zc) in enumerate(parts))
        vis += (f"<visual name='cap'><pose>0 0 {lz} 0 0 0</pose><geometry><cylinder><radius>0.06</radius>"
                f"<length>0.03</length></cylinder></geometry>{material((0.15, 0.15, 0.15))}</visual>")
        return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose(x, y, 0.0, yaw)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {lz / 2} 0 0 0</pose><geometry><box><size>{lx} {ly} {lz}</size></box></geometry></collision>
        {vis}
      </link>
    </model>"""
    raise ValueError(kind)


def victim_model(v):
    name, pose, x, y, heading = v
    if pose == 'lying':
        # Mesh: standing, feet at the origin, head at +z, arms along x. Roll +90 deg lays it
        # on its back with the head along -y; yaw = heading + 90 deg turns -y to `heading`.
        # In the (rolled) model frame the body runs along +z, the arms along x and the
        # thickness along y, so the collision box is (width, thickness, length).
        hx, hy = unit(heading)
        fx, fy = x - LYING_LENGTH / 2 * hx, y - LYING_LENGTH / 2 * hy    # feet
        return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose(fx, fy, 0.178, heading + 90.0, roll=math.pi / 2)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {LYING_LENGTH / 2} 0 0 0</pose>
          <geometry><box><size>{2 * LYING_HALF_WIDTH} {LYING_HEIGHT} {LYING_LENGTH}</size></box></geometry></collision>
        <visual name='visual'><geometry><mesh><uri>model://rescue_randy/meshes/rescue_randy.dae</uri></mesh></geometry>
          <material><diffuse>1 1 1</diffuse><specular>0.3 0.3 0.3</specular><pbr><metal>
            <albedo_map>model://rescue_randy/materials/textures/rescue_randy_albedo.png</albedo_map>
            <normal_map>model://rescue_randy/materials/textures/rescue_randy_normal.png</normal_map>
            <roughness_map>model://rescue_randy/materials/textures/rescue_randy_roughness.png</roughness_map>
          </metal></pbr></material></visual>
      </link>
    </model>"""
    # Sitting mesh (link offset -0.624 m): footprint x -0.47 .. 0.27, y -0.71 .. 0.30,
    # legs towards -y. yaw = facing + 90 deg; shift so (x, y) is the footprint centre.
    yaw = math.radians(heading + 90.0)
    lx, ly = -0.10, -0.205
    ox = x - (lx * math.cos(yaw) - ly * math.sin(yaw))
    oy = y - (lx * math.sin(yaw) + ly * math.cos(yaw))
    d, w, h = SITTING_SIZE
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{wpose(ox, oy, 0.0, heading + 90.0)}</pose>
      <link name='link'>
        <collision name='collision'><pose>{lx} {ly} {h / 2} 0 0 {math.pi / 2:.4f}</pose>
          <geometry><box><size>{d} {w} {h}</size></box></geometry></collision>
        <visual name='visual'><pose>0 0 -0.624 0 0 0</pose>
          <geometry><mesh><uri>model://rescue_randy_sitting/meshes/rescue_randy.dae</uri></mesh></geometry></visual>
      </link>
    </model>"""


def world_sdf(extra=''):
    body = ''.join(wall_model(*w) for w in wall_segments())
    body += ''.join(obstacle_model(o) for o in OBSTACLES)
    body += ''.join(victim_model(v) for v in VICTIMS)
    return f"""<?xml version="1.0" ?>
<!-- Generated by disaster_models/scripts/make_sar_arena_world.py - edit the script, not this file.
     5 m x 8 m SAR arena, blue foam-board walls. World origin = Go2 start pose. -->
<sdf version="1.10">
  <world name="sar_arena">
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
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.3 0.2 -0.93</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          {material((0.62, 0.62, 0.60))}
        </visual>
      </link>
    </model>{body}{extra}
  </world>
</sdf>
"""


def spectators_sdf():
    out = ''
    cx, cy = FIELD_X / 2, FIELD_Y / 2
    for i, (model, x, y) in enumerate(SPECTATORS):
        uri, scale, yaw0, (sx, sy, h) = SPECTATOR_MODELS[model]
        face = math.degrees(math.atan2(cy - y, cx - x))
        out += f"""
    <model name='spectator_{i + 1}'>
      <static>true</static>
      <pose>{wpose(x, y, 0.0, face + yaw0)}</pose>
      <link name='link'>
        <collision name='collision'><pose>0 0 {h / 2} 0 0 0</pose><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name='visual'><geometry><mesh><uri>{uri}</uri><scale>{scale} {scale} {scale}</scale></mesh></geometry></visual>
      </link>
    </model>"""
    return out


# --------------------------------------------------------- preview cameras
def camera_sensor(name, x, y, z, yaw_deg, pitch_deg, hfov_deg, out_dir, w=1280, h=800):
    """A camera model at field (x, y, z) that saves its frames to out_dir/name."""
    return f"""
    <model name='cam_{name}'>
      <static>true</static>
      <pose>{wpose(x, y, z, yaw_deg, pitch=math.radians(pitch_deg))}</pose>
      <link name='link'>
        <sensor name='{name}' type='camera'>
          <topic>preview/{name}</topic>
          <update_rate>2</update_rate>
          <camera>
            <horizontal_fov>{math.radians(hfov_deg):.4f}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>60</far></clip>
            <save enabled="true"><path>{os.path.join(out_dir, name)}</path></save>
          </camera>
        </sensor>
      </link>
    </model>"""


def preview_cameras(out_dir):
    """Overview shots plus one Go2-height view (0.35 m) from every door into its room."""
    cams = [
        ('top', FIELD_X / 2, FIELD_Y / 2 - 0.01, 10.0, 90.0, 89.9, 32.0),   # portrait, north up
        ('oblique_sw', -2.2, -2.6, 5.2, 55.0, 38.0, 70.0),
        ('oblique_ne', 7.4, 10.4, 5.2, -125.0, 38.0, 70.0),
        ('start_view', START_FIELD[0], START_FIELD[1], 0.35, START_FIELD[2], 0.0, 69.0),
    ]
    centres = {}
    for room, ((x0, x1, y0, y1), ds) in ROOMS.items():
        for d in ds:
            centres[d] = ((x0 + x1) / 2, (y0 + y1) / 2)
    for name, (x1, y1, x2, y2) in doors().items():
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        rx, ry = centres[name]
        nx, ny = -(y2 - y1), x2 - x1                          # door normal
        if nx * (rx - mx) + ny * (ry - my) < 0:
            nx, ny = -nx, -ny                                # ... pointing into the room
        n = math.hypot(nx, ny)
        nx, ny = nx / n, ny / n
        cams.append((f'{name}_view', mx - 0.35 * nx, my - 0.35 * ny, 0.35,
                     math.degrees(math.atan2(ny, nx)), 0.0, 87.0))
    size = {'top': (800, 1280)}
    return ''.join(camera_sensor(n, x, y, z, yaw, pitch, fov, out_dir, *size.get(n, (1280, 800)))
                   for n, x, y, z, yaw, pitch, fov in cams), \
        [c[0] for c in cams]


# ---------------------------------------------------------------- checks
def check_spectators():
    """Spectators must stand outside the arena (with some room), never inside."""
    ok = True
    for model, x, y in SPECTATORS:
        gap = max(-x, x - FIELD_X, -y, y - FIELD_Y)
        print(f'  {model} at ({x}, {y}): {gap:.2f} m outside the arena')
        ok &= gap >= 0.4
    return ok


def ray_hits_box(p, q, box):
    """Does the segment p -> q (3D, field frame) pass through the oriented box?"""
    cx, cy, yaw, sx, sy, z0, z1 = box
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))

    def local(pt):
        dx, dy = pt[0] - cx, pt[1] - cy
        return np.array([c * dx + s * dy, -s * dx + c * dy, pt[2]])
    a, b = local(p), local(q)
    d = b - a
    lo = np.array([-sx / 2, -sy / 2, z0])
    hi = np.array([sx / 2, sy / 2, z1])
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if a[k] < lo[k] or a[k] > hi[k]:
                return False
            continue
        ta, tb = (lo[k] - a[k]) / d[k], (hi[k] - a[k]) / d[k]
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return t1 > 1e-6 and t0 < 1.0 - 1e-6


def door_viewpoints(door_name, room):
    """Robot sensor positions 'at the door': across the opening (minus the robot half
    width), from 0.8 m outside to 0.15 m inside, at the camera / lidar heights."""
    x1, y1, x2, y2 = doors()[door_name]
    (rx0, rx1, ry0, ry1), _ = ROOMS[room]
    length = math.hypot(x2 - x1, y2 - y1)
    ux, uy = (x2 - x1) / length, (y2 - y1) / length
    nx, ny = -uy, ux
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    if nx * ((rx0 + rx1) / 2 - mx) + ny * ((ry0 + ry1) / 2 - my) < 0:
        nx, ny = -nx, -ny
    pts = []
    for s in np.linspace(0.18, length - 0.18, 5):
        for depth in (-0.8, -0.5, -0.25, 0.0, 0.15):
            for z in CAMERA_HEIGHTS:
                pts.append((x1 + s * ux + depth * nx, y1 + s * uy + depth * ny, z))
    return pts


def check_hidden():
    """No sample point of a room's victim may be visible from any viewpoint at any of
    the room's doors (walls and obstacles occlude, victims do not)."""
    occluders = wall_boxes() + [obstacle_box(o) for o in OBSTACLES]
    ok = True
    for room, ((x0, x1, y0, y1), ds) in ROOMS.items():
        inside = [v for v in VICTIMS if x0 < v[2] < x1 and y0 < v[3] < y1]
        for v in inside:
            samples = victim_samples(v)
            for d in ds:
                seen = 0
                for p in door_viewpoints(d, room):
                    for q in samples:
                        if not any(ray_hits_box(p, q, b) for b in occluders):
                            seen += 1
                            break
                status = 'hidden' if seen == 0 else f'VISIBLE from {seen} viewpoints'
                print(f'  {v[0]} ({v[1]}) in {room}: from {d} {status}')
                ok &= seen == 0
    return ok


def occupancy_grid(res=0.05):
    """Ground-truth occupancy (field frame): walls, obstacles, victim bodies."""
    x0, y0 = -0.2, -0.2
    w, h = int(math.ceil((FIELD_X + 0.4) / res)), int(math.ceil((FIELD_Y + 0.4) / res))
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    px, py = x0 + (cols + 0.5) * res, y0 + (rows + 0.5) * res

    def obox(cx, cy, yaw, sx, sy):
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        lx = c * (px - cx) + s * (py - cy)
        ly = -s * (px - cx) + c * (py - cy)
        return (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)

    walls = np.zeros((h, w), bool)
    for cx, cy, yaw, sx, sy, _, _ in wall_boxes():
        walls |= obox(cx, cy, yaw, sx + res, sy + res)   # pad: 5 cm boards on a 5 cm grid
    obstacles = np.zeros((h, w), bool)
    for o in OBSTACLES:
        f = obstacle_footprint(o)
        obstacles |= (np.hypot(px - f[1], py - f[2]) <= f[3]) if f[0] == 'circle' else obox(*f)
    victims = np.zeros((h, w), bool)
    for v in VICTIMS:
        victims |= obox(*victim_body(v)[:5])
        for (ax, ay), (bx, by) in victim_arms(v):      # the robot must not step on the arms
            vx, vy = bx - ax, by - ay
            t = np.clip(((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy), 0.0, 1.0)
            victims |= np.hypot(px - (ax + t * vx), py - (ay + t * vy)) <= 0.05
    return walls, obstacles, victims, (x0, y0, res)


def check_passable():
    walls, obstacles, victims, (x0, y0, res) = occupancy_grid()
    occ = walls | obstacles | victims
    dist = distance_field(occ, res)
    free = dist > PLANNER_ROBOT_RADIUS
    to_cell = lambda x, y: (int((y - y0) / res), int((x - x0) / res))
    ok = True

    start = to_cell(*START_FIELD[:2])
    points = [('start', start)]
    # Only cells connected to the start count as robot positions (a pocket between a
    # victim's arms is free but unreachable).
    reach = np.zeros_like(free)
    reach[start] = True
    frontier = [start]
    while frontier:
        r, c = frontier.pop()
        for n in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= n[0] < free.shape[0] and 0 <= n[1] < free.shape[1] and free[n] and not reach[n]:
                reach[n] = True
                frontier.append(n)
    rows, cols = np.nonzero(reach)
    cx, cy = x0 + (cols + 0.5) * res, y0 + (rows + 0.5) * res
    for v in VICTIMS:
        bx, by, yaw, sx, sy = victim_body(v)[:5]
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        lx = np.abs(c * (cx - bx) + s * (cy - by)) - sx / 2
        ly = np.abs(-s * (cx - bx) + c * (cy - by)) - sy / 2
        d = np.hypot(np.maximum(lx, 0), np.maximum(ly, 0))   # distance to the body box
        k = int(d.argmin())
        print(f'  {v[0]}: closest reachable robot position {d[k]:.2f} m from the body')
        ok &= d[k] <= STOP_DISTANCE
        points.append((v[0], (int(rows[k]), int(cols[k]))))
    for name, (dx1, dy1, dx2, dy2) in doors().items():
        cell = to_cell((dx1 + dx2) / 2, (dy1 + dy2) / 2)
        if not reach[cell]:
            print(f'  {name} is NOT passable')
            ok = False
        else:
            points.append((name, cell))
    for (an, a), (bn, b) in itertools.combinations(points, 2):
        if astar(free, a, b) is None:
            print(f'  NO PATH {an} -> {bn} with the planner radius {PLANNER_ROBOT_RADIUS} m')
            ok = False
    if not ok:
        return False
    narrowest = math.inf
    for (an, a), (bn, b) in itertools.combinations(points, 2):
        if an.startswith('door') or bn.startswith('door'):
            continue
        bottleneck = widest_path(dist, free, a, b, exclude=0.6 / res)
        narrowest = min(narrowest, bottleneck)
    status = 'ok' if 2 * narrowest >= MIN_FREE_WIDTH else 'TOO NARROW'
    print(f'  all victims and doors reachable; narrowest passage on the routes {2 * narrowest:.2f} m [{status}]')
    return 2 * narrowest >= MIN_FREE_WIDTH


# ------------------------------------------------------------------ mission files
def write_world_map(path_png, path_yaml, occupied_field, grid, comment):
    """Rasterise a field-frame occupancy into a world-frame map_server map."""
    gx0, gy0, gres = grid
    corners = [to_world(x, y) for x in (0.0, FIELD_X) for y in (0.0, FIELD_Y)]
    x0 = min(c[0] for c in corners) - MAP_MARGIN
    y0 = min(c[1] for c in corners) - MAP_MARGIN
    x1 = max(c[0] for c in corners) + MAP_MARGIN
    y1 = max(c[1] for c in corners) + MAP_MARGIN
    w = int(math.ceil((x1 - x0) / MAP_RESOLUTION))
    h = int(math.ceil((y1 - y0) / MAP_RESOLUTION))
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    wx = x0 + (cols + 0.5) * MAP_RESOLUTION
    wy = y0 + (h - 1 - rows + 0.5) * MAP_RESOLUTION
    # world -> field (inverse of to_world)
    sx, sy, syaw = START_FIELD
    a = math.radians(syaw)
    fx = sx + wx * math.cos(a) - wy * math.sin(a)
    fy = sy + wx * math.sin(a) + wy * math.cos(a)
    gc = np.floor((fx - gx0) / gres).astype(int)
    gr = np.floor((fy - gy0) / gres).astype(int)
    inside = (gc >= 0) & (gc < occupied_field.shape[1]) & (gr >= 0) & (gr < occupied_field.shape[0])
    occ = np.zeros((h, w), bool)
    occ[inside] = occupied_field[gr[inside], gc[inside]]
    write_png_gray(path_png, np.where(occ, 0, 255).astype(np.uint8))
    with open(path_yaml, 'w') as f:
        f.write(f"""# {comment}
# ROS map_server format. World frame = Go2 start pose.
image: {os.path.basename(path_png)}
resolution: {MAP_RESOLUTION}
origin: [{x0:.3f}, {y0:.3f}, 0.0]   # world (x, y, yaw) of the lower-left pixel
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
    return w, h


def write_mission():
    os.makedirs(MISSION_DIR, exist_ok=True)
    walls, obstacles, victims, grid = occupancy_grid(MAP_RESOLUTION / 2)
    write_world_map(os.path.join(MISSION_DIR, 'map.png'), os.path.join(MISSION_DIR, 'map.yaml'),
                    walls, grid, 'Walls only - prior map for the known-map baseline (go2_sar_knownMap).')
    w, h = write_world_map(os.path.join(MISSION_DIR, 'ground_truth_map.png'),
                           os.path.join(MISSION_DIR, 'ground_truth_map.yaml'), walls | obstacles, grid,
                           'Walls + obstacles - for evaluating maps built online, never for the robot.')
    rng = random.Random(RANDOM_SEED)
    with open(os.path.join(MISSION_DIR, 'victims_ground_truth.yaml'), 'w') as f:
        f.write('# Ground truth - for the mission monitor only, never for the robot.\n')
        f.write('# Body: oriented box, centre (x, y), heading [deg] (lying: head direction,\n')
        f.write('# sitting: facing direction), length along the heading, half width.\n')
        f.write(f'length: {LYING_LENGTH}\nhalf_width: {LYING_HALF_WIDTH}\nvictims:\n')
        for v in VICTIMS:
            cx, cy, yaw, sx, sy = victim_body(v)[:5]
            wx, wy = to_world(cx, cy)
            f.write(f'  - {{name: {v[0]}, pose: {v[1]}, x: {wx:.3f}, y: {wy:.3f}, '
                    f'head_deg: {yaw_to_world(yaw):.1f}, length: {sx}, half_width: {sy / 2}}}\n')
    with open(os.path.join(MISSION_DIR, 'victims_estimated.yaml'), 'w') as f:
        f.write(f'# Rough victim positions (error <= {VICTIM_POSITION_ERROR} m) - known-map baseline only.\n')
        f.write('victims:\n')
        for v in VICTIMS:
            wx, wy = to_world(v[2], v[3])
            r = VICTIM_POSITION_ERROR * math.sqrt(rng.random())
            a = rng.uniform(-math.pi, math.pi)
            f.write(f'  - {{name: {v[0]}, x: {wx + r * math.cos(a):.2f}, y: {wy + r * math.sin(a):.2f}}}\n')
    return w, h


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--preview', metavar='DIR', help='also write a preview world with saving cameras')
    ap.add_argument('--no-audience', action='store_true',
                    help=f'without spectators, written as worlds/{NO_AUDIENCE_NAME}.sdf (the old training world)')
    args = ap.parse_args()
    if args.no_audience:
        SPECTATORS.clear()
        WORLD_OUT = os.path.join(HERE, '..', 'worlds', f'{NO_AUDIENCE_NAME}.sdf')
    print('passability:')
    passable = check_passable()
    print('victims hidden from the doors:')
    hidden = check_hidden()
    print('spectators:')
    outside = check_spectators()
    if not (passable and hidden and outside):
        raise SystemExit('layout check failed - fix OBSTACLES / VICTIMS (nothing written)')
    with open(WORLD_OUT, 'w') as f:
        f.write(world_sdf(spectators_sdf()))
    w, h = write_mission()
    print(f'wrote {os.path.normpath(WORLD_OUT)}')
    print(f'wrote {os.path.normpath(MISSION_DIR)}/ (maps {w} x {h} px, victims_ground_truth.yaml)')
    for v in VICTIMS:
        print(f'  {v[0]} ({v[1]}): world {tuple(round(c, 2) for c in to_world(v[2], v[3]))}')
    if args.preview:
        out = os.path.abspath(args.preview)
        os.makedirs(out, exist_ok=True)
        extra, names = preview_cameras(out)
        path = os.path.join(out, f'{NAME}_preview.sdf')
        with open(path, 'w') as f:
            f.write(world_sdf(spectators_sdf() + extra))
        with open(os.path.join(out, 'cameras.txt'), 'w') as f:
            f.write('\n'.join(names) + '\n')
        print(f'wrote {path} (cameras: {", ".join(names)})')
