# Scout (A1) — Runbook

Scout is a **purely reactive explorer**: no map, no global planner, no SLAM.
It is the backup for the case where SLAM does not come up on the real Go2.

It decides from what it senses right now:

| | |
|---|---|
| **Obstacles / ranges** | 3D lidar |
| **Heading, body tilt** | IMU |
| **Victims** | RGB camera → YOLOv8n (`/sar/person_detections`) |
| **Position** | dead reckoning (commanded velocity × `velocity_scale`), corrected by lidar scan matching |

The position estimate is only used to remember where it has already been, to
stay inside the field and to avoid counting one victim twice — never to plan a
route. Every heading it drives is chosen from the current scan.

States: `EXPLORE` (score 48 candidate headings on clearance + novelty) →
`SCAN` (turn on the spot so the narrow camera sees around) → `APPROACH` →
`SIT` (= victim found) → `STAND_UP`, with `RECOVER` when everything is blocked.

Package `go2_sar_scout`, node name `sar_scout`, mode `scout`.

---

## Run in simulation

Build once (10–15 min). The Gazebo meshes and textures are **not in git**, so
point `GAZEBO_ASSETS` at the asset directory — the build stops immediately with
a clear message if it cannot find them:

```bash
cd sim
GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 IMAGE=smc-sar-sim:s2r ./build.sh
```

Run one mission:

```bash
docker run --rm --device /dev/dri smc-sar-sim:s2r \
  ros2 launch go2_real_bridge real_sar.launch.py \
      mode:=scout sim_test:=true enable_motion:=true gui:=false camera:=none
```

`gui:=true` opens the Gazebo window (needs an X display):

```bash
xhost +local:docker
docker run --rm --device /dev/dri \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  smc-sar-sim:s2r \
  ros2 launch go2_real_bridge real_sar.launch.py \
      mode:=scout sim_test:=true enable_motion:=true gui:=true camera:=none
```

## Run on the robot

Dry run first — **without `enable_motion:=true` the robot does not move**:

```bash
ros2 launch go2_real_bridge real_sar.launch.py mode:=scout
ros2 launch go2_real_bridge real_sar.launch.py mode:=scout enable_motion:=true
```

Before trusting a run, check the sensor geometry. The node name matters —
parameters are keyed by it, and with the wrong one you silently validate
defaults:

```bash
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_scout \
    --params-file install/share/go2_real_bridge/config/real_scout.yaml
```

The ground height it prints for front / left / right / behind must be about
equal and near `ground_z_init`.

## Arguments

| Argument | Meaning |
|---|---|
| `mode:=scout` | this algorithm (A1) |
| `enable_motion:=true` | **required to move**; dry run is the default |
| `sim_test:=true` | run on Gazebo through `fake_go2`, and start the referee |
| `gui:=true` | Gazebo window; `false` = server only |
| `camera:=` | `host` (RealSense outside, default) \| `container` \| `none` (simulation) |
| `debug:=true` | per-tick state lines. Must be a launch argument — `extra_params` **cannot** override it |
| `extra_params:=` | a params file loaded **last**, overriding everything |

## Tuning without rebuilding

`extra_params` is loaded last and wins. Parameters are indexed **by node name**;
anything filed under a name no node uses is silently ignored:

```yaml
sar_scout:
  ros__parameters:
    max_linear_speed: 0.30
```

```bash
docker run --rm --device /dev/dri -v /tmp/tune.yaml:/tune/t.yaml:ro \
  smc-sar-sim:s2r ros2 launch go2_real_bridge real_sar.launch.py \
      mode:=scout sim_test:=true enable_motion:=true camera:=none \
      extra_params:=/tune/t.yaml
```

## Files

| | |
|---|---|
| `image/src/go2_sar_scout/go2_sar_scout/scout_node.py` | the algorithm |
| `image/src/go2_sar_scout/config/scout_params.yaml` | **algorithm tuning — shared by simulation and robot** |
| `image/src/go2_real_bridge/config/real_scout.yaml` | hardware and field values (robot) |
| `image/src/go2_real_bridge/config/sim_test.yaml` | restores the simulation hardware values |

`real_scout.yaml` is loaded in simulation too, so every hardware value added
there needs its simulation counterpart in `sim_test.yaml` — otherwise robot
numbers leak into simulation runs.

## Three things that will cost you an hour

1. **`ros2 launch` ignores SIGTERM, only SIGINT.** `timeout 560 docker run …`
   leaves the container alive after the referee has printed its score. Use
   `timeout -s INT`, or run detached and `docker stop` once you see `SCORE`.
2. **`ROS_DOMAIN_ID` is 79 in the simulation image**, not 0. `docker exec`
   needs `-e ROS_DOMAIN_ID=79` or `ros2 node list` comes back empty.
3. **Do not `grep error` to decide a run failed** — the referee itself prints a
   line containing `localization error`.
