# Runbook — the four SAR algorithms: simulation and deployment on the real Go2

*中文版：[RUNBOOK_REACTIVE_CN.md](RUNBOOK_REACTIVE_CN.md)*

This runbook covers the four algorithms we wrote ourselves — `Scout`, `Reflex`,
`Prior` and `SLAM` — from a clean machine to a scored run in Gazebo, and from
there onto the competition robot.

Everything here runs through **one launch file**,
`go2_real_bridge real_sar.launch.py`, in both simulation and on the robot. That
is deliberate: with `sim_test:=true` the simulation is driven through the same
bridge, the same topic names and the same parameter files the robot uses, so a
score measured in Gazebo says something about the robot.

| | Name | Package | `mode:=` | What it needs to work |
|---|---|---|---|---|
| A1 | **Scout** | `go2_sar_scout` | `scout` | lidar, IMU, camera |
| A2 | **Reflex** | `go2_sar_reflex` | `reflex` | lidar, IMU, camera. **No pose estimate at all** |
| A3 | **Prior** | `go2_sar_prior` | `prior` (alias `known`) | lidar, IMU, camera, **+ a prior map and estimated victim positions** |
| A4 | **SLAM** | `go2_sar_slam` | `slam` (alias `nomap`) | lidar, IMU, camera |

Shared code is in `go2_sar_core` (YOLO detector, point-cloud and map utilities).
`go2_sar_referee` is the Gazebo referee and is **deliberately not installed on
the robot** — it reads simulator ground truth.

`Reflex` is the fallback: it holds no map, runs no scan matching and plans no
path, so it keeps working when the lidar pipeline, the odometry or the CPU
budget will not support SLAM. Prefer it over nothing, not over A4.

---

# Part A — Simulation

## A.0 What you need

- Docker, and a user that can run it.
- A GPU render node at `/dev/dri` (without one, substitute
  `-e LIBGL_ALWAYS_SOFTWARE=1` for `--device /dev/dri`; everything still runs,
  slower).
- A checkout of **`SMC_SAR_Phase2`** next to this repository. The Gazebo meshes
  and textures (~100 MB of unmodified Fuel assets) are **not in git** — see
  [`sim/src/ASSETS.md`](sim/src/ASSETS.md). They are injected at build time.

```
~/SAR/
├── sar2026_lumenfleet/     this repository
└── SMC_SAR_Phase2/         assets source (default location)
```

## A.1 Build the simulation image

```bash
cd sar2026_lumenfleet/sim
./build.sh                                     # assets from ../SMC_SAR_Phase2
GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 ./build.sh   # assets elsewhere
IMAGE=smc-sar-sim:test ./build.sh              # build under another tag
```

Default tag `smc-sar-sim:jazzy`. Takes roughly 10–15 minutes from cold.

If the assets are missing the build **stops immediately** with a message naming
the two directories it wanted — it does not produce a half-built image you
discover is broken 300 seconds into a run.

`build.sh` assembles the context from one repository: `sim/src/` (Gazebo,
CHAMP, the Go2 model, the worlds) plus `image/src/` (the mission packages, the
same files the robot runs). The mission packages are **not duplicated**.

## A.2 Run one mission

```bash
docker run --rm --device /dev/dri smc-sar-sim:jazzy \
  ros2 launch go2_real_bridge real_sar.launch.py \
      mode:=slam sim_test:=true enable_motion:=true gui:=false camera:=none debug:=false
```

| Argument | Meaning |
|---|---|
| `mode:=` | `scout` \| `reflex` \| `prior` \| `slam` (`nomap`/`known` still accepted) |
| `sim_test:=true` | run on Gazebo through `fake_go2`, and start the referee |
| `enable_motion:=true` | **without this the robot does not move** (dry run is the default) |
| `gui:=false` | Gazebo server only. `true` needs an X display |
| `camera:=none` | no RealSense node — the simulated camera is already there |
| `debug:=true` | per-tick state lines; large logs, use when diagnosing |
| `world:=` | another `.sdf`; default is the 5×8 m training arena |
| `extra_params:=` | a params file loaded **last**, overriding everything |

The mission waits ~40 s of simulated time for the CHAMP controllers to stand
the robot up, then runs for 300 s.

## A.3 Reading the result

The referee prints, from Gazebo ground truth:

```
SCORE 2/4 victims found within 300 s (2 sit-downs)
  victim_1: not found
  victim_2: found at t=165.6 s, 0.79 m away
  victim_3: not found
  victim_4: found at t= 20.2 s, 0.65 m away
  localization error (estimate vs ground truth): mean 0.22 m, max 0.36 m
```

- A victim counts only if the robot **actually sits down** within **1.0 m** of
  it. Sit-downs that land next to nothing are counted in the parenthesis but
  score nothing — `(2 sit-downs)` next to `2/4` means both were good;
  `(3 sit-downs)` next to `1/4` means two were spurious.
- `localization error` is reported as `nan` for `Scout` and `Reflex`: they
  publish no pose estimate for the referee to compare against. Not a fault.
- The string `localization error` contains the word "error". Do not grep logs
  for `error` and conclude the run failed — this line and a NumPy
  `RuntimeWarning` about `nan` are the only matches in a clean run.

## A.4 Running all four

Run them **one at a time**. They share a DDS domain, so parallel runs see each
other's topics, and Gazebo is CPU-hungry enough that running two distorts the
timing of both.

> **Gotcha that will cost you half an hour.** `ros2 launch` handles **SIGINT**,
> not SIGTERM. `timeout 560 docker run ...` sends SIGTERM, the container ignores
> it, and your queue stalls after the first mission with the referee's score
> already printed. Either use `timeout -s INT`, or do what the verification
> script does: run detached and stop the container when the score appears.

```bash
for mode in scout reflex prior slam; do
  docker run -d --rm --name "sar_$mode" --device /dev/dri smc-sar-sim:jazzy \
    ros2 launch go2_real_bridge real_sar.launch.py \
        mode:=$mode sim_test:=true enable_motion:=true gui:=false camera:=none >/dev/null
  for i in $(seq 1 620); do
    sleep 1
    docker logs "sar_$mode" > "$mode.log" 2>&1
    grep -q "SCORE " "$mode.log" && { sleep 3; break; }
    docker ps -q -f "name=sar_$mode" | grep -q . || break
  done
  docker logs "sar_$mode" > "$mode.log" 2>&1
  docker stop -t 5 "sar_$mode" >/dev/null
  grep -E "SCORE|found at|not found" "$mode.log" | grep mission_monitor
done
```

About 6 minutes per mission.

## A.5 Changing parameters without rebuilding

`extra_params` is loaded after every other file, so it wins:

```bash
docker run --rm --device /dev/dri -v "$PWD/tune:/tune" smc-sar-sim:jazzy \
  ros2 launch go2_real_bridge real_sar.launch.py mode:=slam sim_test:=true \
      enable_motion:=true gui:=false camera:=none extra_params:=/tune/my.yaml
```

The referee reads it too, so the time limit can be equalised across modes from
outside — necessary if you want two scores to be comparable.

Parameters are keyed by **node name**, which is now the same as the mode:
`sar_scout`, `sar_reflex`, `sar_prior`, `sar_slam`, plus `person_detector`,
`go2_bridge`, `fake_go2`, `mission_monitor`. A section under a name no node
uses is silently ignored — this is the single most common way to "change" a
parameter and have nothing happen.

## A.6 Scores measured so far, and one open question

Gazebo 5×8 arena, 300 s, four victims, one run each, 2026-11-13:

| Algorithm | Score | Note |
|---|---|---|
| A1 Scout | 2/4 | reproducible |
| A2 Reflex | 0/4 | gave up 0.74 m from a victim; see below |
| A3 Prior | 1/4 | penalty self-encirclement, [`NAVIGATION.md`](NAVIGATION.md) |
| A4 SLAM | 2/4 | **was 4/4 before the camera change — unresolved** |

> ⚠️ **Open question: the depth camera cost A4 half its score.** The simulated
> camera was changed from `camera` to `rgbd_camera` on 2026-11-12. Intrinsics
> are identical (1.367 rad FOV, 640×480, 10 Hz), but with the same code A4 fell
> from **4/4** to **1/4–2/4**, and the cause is consistently that the first
> victim is confirmed ~14 s later (t≈1 s before, t≈15–19 s after), which cascades
> through the whole run. Verified against the package rename by running the
> pre-rename code on the post-camera image: it scored 1/4, so the rename is not
> responsible. **Do not read A4's 2/4 as its real capability, and do not tune it
> against these numbers until this is settled.**

> ⚠️ **A2's 0/4 is a stricter gate, not a regression.** The earlier 1/4 came
> from three sit-downs of which two were spurious. A fix to a `dt` bug removed
> the spurious ones; the remaining real target was then refused by
> `victim_fresh_time: 1.5`, because a person lying down drops out of the camera's
> vertical field of view at close range and the last sighting goes stale. The
> robot reached 0.74 m with `sit_distance: 0.75` and gave up. Relaxing the
> freshness window once inside `sit_distance` is the obvious fix and is not
> done yet.

---

# Part B — The real Go2

> **Nothing in Part B's motion sections has been run on a competition robot
> yet.** The sensor, network and dry-run steps were verified on robot `seek`
> (2026-10-26); the mission itself has only ever moved a simulated robot. Treat
> every motion step as first-time, with an operator holding the physical
> controller.

## B.0 Before you touch anything

1. An operator with the physical remote in hand, eyes on the robot, thumb ready.
2. Clear floor: at least 2 m ahead, and know where the walls are.
3. Know how to stop it three ways: the remote, `ros2 topic pub /sar/estop`,
   `ros2 run go2_real_bridge sport_cli damp` (motors limp — the robot *drops*;
   emergency only).

## B.1 Network and DDS — every shell, host and container

```bash
source /opt/ros/jazzy/setup.bash                 # host shells only
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'$SAR_ROBOT_IFACE'"/></Interfaces></General></Domain></CycloneDDS>'
ros2 daemon stop
```

Verify before going further:

```bash
echo $SAR_ROBOT_IFACE          # `end0` on seek — robot-specific
ip -br addr show $SAR_ROBOT_IFACE
ros2 topic list | grep utlidar
```

An empty topic list means, in order: ROS not sourced, `CYCLONEDDS_URI` not set
or naming the wrong interface, stale daemon.

The container entrypoint builds `CYCLONEDDS_URI` for you from `SAR_ROBOT_IFACE`
or `ROBOT_IFACE` and **refuses to start** if neither is set and the interface
does not exist. Use host networking.

## B.2 Build and enter the robot image

```bash
cd ~/sar2026_lumenfleet/image
docker build -t lumen-jazzy:sar .
sar-run lumen-jazzy:sar bash
```

This is a different image from the simulation one: no Gazebo, plus RealSense,
RTAB-Map and Nav2. It contains `go2_sar_core`, `go2_sar_scout`,
`go2_sar_reflex`, `go2_sar_prior`, `go2_sar_slam` and `go2_real_bridge` — the
same sources the simulation built, and **not** `go2_sar_referee`.

Files created inside the container are owned by root, and `/tmp` is lost on
exit. Keep anything you want to survive in the mounted repository.

## B.3 Check who is already driving the robot

**Two controllers commanding one robot is the unsafe case**, not merely
redundant.

```bash
ros2 node list
ros2 topic info /api/sport/request -v
ros2 topic list | grep utlidar
```

- `/utlidar/*` already present → the organizers' driver is running. Use those
  topics; do **not** start a second driver.
- Something already publishing to `/api/sport/request` → find out what it is
  before adding `go2_bridge` on top.

## B.4 Camera

The D435i driver runs **on the host**, not in our container (the image has no
librealsense), and only one process may stream at a time (`pgrep -a realsense`).

```bash
ros2 launch sar_go2_example camera_tf.launch.py     # driver + the organizers' calibration TF
```

Then launch with `camera:=host` (the default). Use `camera:=container` only if
you have passed the USB device through and nothing else is streaming.

The detector reads `/camera/camera/color/image_raw`. Confirm it is alive before
the mission — a mission with no images finds nothing and gives no error.

## B.5 Verify the sensor geometry — do not skip this

```bash
CFG=/opt/lumen_ws/install/share/go2_real_bridge/config

# A1 Scout and A2 Reflex -- the node MUST be renamed. ROS 2 keys parameter
# files by node name; check_sensors is called sar_slam, so without the remap
# the sar_scout: / sar_reflex: section matches nothing, is dropped without a
# word, and the check validates built-in defaults instead of what the mission
# will actually use. The node prints the geometry it ended up with, and warns
# when no overrides reached it -- read that line before trusting the result.
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_scout  --params-file $CFG/real_scout.yaml
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_reflex --params-file $CFG/real_reflex.yaml

# A3 Prior and A4 SLAM -- already named sar_slam, no remap needed
ros2 run go2_real_bridge check_sensors --ros-args --params-file $CFG/real_nomap.yaml
```

Sends no motion. Prints message rates for odometry, IMU, lidar and camera, and
— the part that matters — the **ground height in front / left / right / behind**
after applying `lidar_xyz` / `lidar_rpy` and levelling with the IMU.

- The four ground heights must be about equal and near `ground_z_init`
  (`-0.32`). If they are not, the lidar extrinsic is wrong and **every ground
  and obstacle height test in every algorithm is wrong with it**.
- Check how many points land inside `body_box` — that is the robot seeing its
  own legs, which the algorithms cut away.

Two values are **robot-specific and must be measured on the robot you are
given**:

- `camera_xyz` (`[0.33, 0.0, 0.04]`): the organizers' per-robot D435i
  calibration adds a few centimetres, and the URDF's mount disagrees with it.
  Trust the calibration.
- `lidar_xyz` / `lidar_rpy`: `[0,0,0]` is correct **only because** `cloud_in` is
  `/utlidar/cloud_base`, which the firmware has already transformed into
  `base_link`. If you point `cloud_in` at the raw `/utlidar/cloud` instead, you
  must restore the URDF values (`[0.28945, 0, -0.046825]`, rpy `[0, 2.8782, 0]`)
  — and applying them to `cloud_base` transforms the cloud twice.

## B.6 Dry run — no motion

```bash
ros2 launch go2_real_bridge real_sar.launch.py mode:=slam
```

`enable_motion` defaults to `false`: the full stack runs, the algorithm plans
and decides, and **no motion request reaches the robot**. Watch for:

- `go2_bridge` logging which source feeds `odom_out` and `imu_out`;
- the detector reporting `persons: N`;
- the mission's own state transitions (`WAIT → SPIN → EXPLORE → …`).

Only when all three look right, go on.

## B.7 First motion, smallest possible

With the operator ready:

```bash
ros2 run go2_real_bridge sport_cli stand_up
ros2 run go2_real_bridge sport_cli move 0.3 0 0 1      # 1 s forward, then StopMove
```

Then the mission, motion enabled:

```bash
ros2 launch go2_real_bridge real_sar.launch.py mode:=slam enable_motion:=true
# A3 needs the prior map and estimated victim positions:
ros2 launch go2_real_bridge real_sar.launch.py mode:=prior \
    mission:=/missions/real_field enable_motion:=true
```

The bridge clips to `max_vx 0.5`, `max_vy 0.3`, `max_wz 0.8` and stops the robot
if commands go stale for `cmd_timeout` (0.5 s).

## B.8 Emergency stop

```bash
ros2 topic pub --once /sar/estop std_msgs/Bool "{data: true}"      # StopMove, ignore all motion
ros2 topic pub --once /sar/estop std_msgs/Bool "{data: false}"     # release
```

The bridge logs `EMERGENCY STOP` and refuses every motion command until
released. The physical remote still outranks all of this.

## B.9 How the parameters reach the robot

Each mission loads four files, later ones winning:

```
<algorithm>_params.yaml   algorithm tuning      shared by simulation and robot
real_common.yaml          bridge + detector     shared
real_<mode>.yaml          HARDWARE AND FIELD    the robot's values
sim_test.yaml             (simulation only)     puts the simulated values back
```

The split matters: `real_*.yaml` holds only what is a property of the robot or
the field — camera intrinsics, lidar extrinsics, ground height, the walking
floor, body width, virtual walls. Everything else stays in the algorithm's own
`*_params.yaml`, so **what is tuned in simulation is what runs on the robot**.

`real_*.yaml` is loaded in simulation too. Every hardware value in it therefore
needs its simulated counterpart in `sim_test.yaml`, or the robot's numbers leak
into the simulation. Keep the two sides in step when you add a value.

| Mode | Algorithm file | Real file |
|---|---|---|
| `scout` | `go2_sar_scout/config/scout_params.yaml` | `real_scout.yaml` |
| `reflex` | `go2_sar_reflex/config/reflex_params.yaml` | `real_reflex.yaml` |
| `prior` | `go2_sar_prior/config/prior_params.yaml` | `real_known.yaml` |
| `slam` | `go2_sar_slam/config/slam_params.yaml` | `real_nomap.yaml` |

## B.10 Measure these on the robot you are given

Everything below is a number nobody can obtain from a simulator. The files ship
a value so the stack starts, but it is a placeholder until measured. Ordered by
what breaks first.

### 1. Lidar extrinsic and ground height — before anything else

```bash
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_scout --params-file $CFG/real_scout.yaml
```

The ground height printed for **front / left / right / behind** must be about
equal and near `ground_z_init` (−0.32). If they differ by direction, the lidar
extrinsic is wrong, and **every ground test, every obstacle height and every
victim range in every algorithm is wrong with it**. Nothing below is worth
doing until this is flat.

`lidar_xyz: [0,0,0]` is correct only because `go2_bridge`'s `cloud_in` is
`/utlidar/cloud_base`, which the firmware has already put in `base_link`. If
you switch to the raw `/utlidar/cloud`, restore the URDF values
(`[0.28945, 0, −0.046825]`, rpy `[0, 2.8782, 0]`); applying them to
`cloud_base` transforms the cloud twice.

### 2. Camera — confirm camera_info arrives

Both Scout and Reflex now take the camera model from `camera_info`. Watch for
this line within a few seconds of starting:

```
camera model from camera_info: fx=... cx=... cy=... (WIDTHxHEIGHT)
```

If instead you see the warning `no camera_info on ... yet -- still using the
fallback camera model`, the node is computing bearings from a 640×480 sim
camera. The organizers' launch streams 1280×720, which **doubles every
bearing**; for Reflex it also corrupts the apparent-size tests that reject
false positives, so it silently rejects real victims. Fix the topic before
running — do not run with the warning showing.

Sanity check: `fx` should be roughly `width / (2·tan(hfov/2))`. For a 640-wide
stream at ~69° that is about 465; the simulated camera reports 392.9.

### 3. `camera_xyz` — base_link to the colour optical centre

Shipped `[0.33, 0.0, 0.04]`, read off the Go2 URDF chain. The organizers'
per-robot D435i calibration adds a few centimetres and the URDF's own RealSense
mount disagrees with it. **Use the calibration**, not the URDF. A few
centimetres here is a few centimetres of victim-position error, which matters
against a 1.0 m scoring radius.

### 4. `velocity_scale` — Scout only, and it decides whether Scout works

Scout has no SLAM. Its position is dead reckoning:
`travelled = commanded_v × velocity_scale × dt`. The shipped `0.6` was
calibrated against CHAMP in simulation. On the real robot it will be a
different number, and if it is wrong Scout's whole position estimate drifts —
it sits where it thinks the victim is, not where the victim is.

```bash
python3 /validation/move_distance.py --live --speed 0.3 --distance 2.0
```

Measure the real travel with a tape, then `velocity_scale = real / commanded`.
Repeat at two speeds; if the ratio is not constant, the model does not fit and
`pose_source: odom` is the better choice — `/utlidar/robot_odom` is the robot's
own ~150 Hz odometry and is probably better than CHAMP's, which is the only
reason dead reckoning won in simulation.

### 5. `field_bounds` — Scout only

Shipped `[0,0,0,0]`, which means **no virtual walls**. Measure the real field
and set `[x_min, x_max, y_min, y_max]` in the frame whose origin is the robot's
start pose. A wrong box either traps the robot or lets it leave the field;
leaving it unbounded relies entirely on the lidar seeing the walls.

### 6. `min_forward_speed` and `scan_angular_speed`

`min_forward_speed: 0.30` is set in both real files because the real Go2 does
not move below that — `0.22` is a CHAMP value and on the robot it sits under
the minimum executable speed, producing crawling or twitching. Confirm 0.30 is
really the floor on your robot; raise it if the robot still marks time.

`scan_angular_speed: 0.60` (Scout) is a guess: CHAMP and the real Go2 do not
turn on the spot at the same rate, and `go2_bringup` clips `max_wz` at 0.8. Time
a full 360° turn and set it so a scan takes a sensible share of the mission.

### 7. Detection thresholds — on real images, before the first scored run

`detection_min_score` (Scout 0.50, Reflex 0.40) and Reflex's apparent-size
rejections were tuned against Gazebo's low-poly actors and the simulated
spectators behind the arena wall. Real people, real lighting and real occlusion
put YOLO's scores somewhere else entirely. Record a bag of the real arena with
people in it, run the detector over it, and look at the score distribution for
real victims versus bystanders before choosing the floor.

### 8. Sit timing

`go2_bridge` turns a body pose below `sit_threshold` (−0.04) into a real Sit and
settles for `rise_settle` (3.0 s). Scout's `stand_up_duration` is set to 3.5 s
so it exceeds that. If the robot starts walking while still rising, raise it.

---

# Part C — Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Build stops: "Gazebo meshes and textures not found" | assets not beside the repo | `GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 ./build.sh` |
| Container keeps running after `SCORE` | `ros2 launch` ignores SIGTERM | `docker stop`, or `timeout -s INT` |
| Robot never moves, no error | `enable_motion` left at `false` | add `enable_motion:=true` |
| Robot never moves, real robot, motion enabled | `min_forward_speed 0.22` below the robot's floor | set `0.30` (B.9.1) |
| A parameter change does nothing | section keyed to a node name nothing uses | key it to `sar_scout` / `sar_reflex` / `sar_prior` / `sar_slam` |
| Mission finds nothing, detector silent | no images on the camera topic | check `camera:=`, and `pgrep -a realsense` on the host |
| Ground heights differ by direction in `check_sensors` | lidar extrinsic wrong | B.5 |
| `ros2 topic list` empty on the robot | ROS/DDS env or stale daemon | B.1 |
| `Unable to parse the value of parameter robot_description as yaml` | a `: ` inside an XML comment in the xacro | remove it; `ParameterValue(value_type=str)` in the launch also guards against it |
| Log full of "error" | the referee's `localization error` line | not a failure (A.3) |
| `no camera_info on ... yet` warning | camera_info topic wrong or driver not running | B.10.2 — do not run the mission with this showing |
| `check_sensors` warns "no parameter overrides reached this node" | the file's section is keyed to another node name | add `-r __node:=sar_scout` / `sar_reflex` (B.5) |
| A real-robot value showed up in a simulation run | a `real_*.yaml` entry with no `sim_test.yaml` counterpart | add it to `sim_test.yaml` (B.9) |

---

# Part D — What is not tested

- **No mission has ever moved a real robot.** Everything in Part B from §B.7 on
  is first-time.
- Every number in §B.10 — the sim-to-real plumbing is in place and verified in
  simulation, but the values themselves are placeholders until measured.
- `min_forward_speed` for **A3 Prior and A4 SLAM** on the real robot: still the
  CHAMP value 0.22 in `real_known.yaml` / `real_nomap.yaml`. A1 and A2 are
  fixed; these two are not.
- A4's score since the depth-camera change (A.6) — unexplained.
- A2's close-range sit gate (A.6) — diagnosed, not fixed.
- A3's penalty self-encirclement — diagnosed in
  [`NAVIGATION.md`](NAVIGATION.md), not fixed.
