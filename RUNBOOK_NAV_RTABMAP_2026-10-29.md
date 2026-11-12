# Runbook: navigation and RTAB-Map on `seek` (2026-10-29)

Continues `RUNBOOK_TESTED_2026-10-26.md`. Image: `lumen-jazzy:nav` (branch
`mapping-nav-explore-hybrid`). Each section says what ran, what was only reported by the
operator, and what was not tested. Robot-specific values may differ on other robots.

## 0. Setup used on `seek`
- `$SAR_ROBOT_IFACE` = `end0`. DDS exports in every shell (see the first runbook, section 0).
- `go2_driver` is NOT launched: `/utlidar/*` is already published by the robot. Always pass
  `driver:=false` (switch added to `go2_online.launch.py`). Running the driver adds a second
  command path and a second `odom -> base_link` source.
- `camera:=false camera_tf:=false` (the Go2 front-camera bridge defaults to interface `eth1`,
  `seek` uses `end0`; `camera_tf` publishes `base_link -> camera_face`, not our camera frame).
- Exactly one source per transform: `retime_utlidar` publishes `odom -> base_link`;
  `slam_toolbox` publishes `map -> odom`. Do not run `odom_tf.py` or `receive_time_adapter`
  with these stacks.

## 1. Stand the robot up (keyboard)
```bash
sar-run lumen-jazzy:nav bash
python3 /validation/keyboard_control.py --live      # u: stand up, x: exit
```
Refuses to start while a `go2_cmd_vel_bridge` node exists (even disabled): stop the
navigation launch first. Keys: w/s a/d q/e, +/- speed, Space stop, u stand, r recovery,
j then y stand down. Operator keeps the physical remote.

## 2. Navigation with slam_toolbox + Nav2 (Stage 1 = plan only, Stage 2 = moves)
```bash
sar-run lumen-jazzy:nav bash
ros2 launch go2_bringup go2_navigation_slam.launch.py \
  driver:=false camera:=false camera_tf:=false use_rviz:=false            # Stage 1
# add enable_motion:=true for Stage 2
```
Without `enable_motion:=true` the bridge prints `DISABLED`: Nav2 plans but the robot cannot
move (`Failed to make progress`, then the recovery `spin` times out). With it, Nav2's
`/cmd_vel` goes to `/api/sport/request` through `go2_cmd_vel_bridge` (caps 0.40 m/s,
0.80 rad/s). The recovery behaviors (`spin`, `backup`) really move the robot: operator ready.

### Jazzy fixes needed in the config and launch (found today)
| Symptom in the log | Cause | Fix (repo) |
|---|---|---|
| `class nav2_navfn_planner/NavfnPlanner ... does not exist` | Humble plugin names | `plugin: "pkg::Class"` in the Nav2 YAMLs (pushed) |
| `StopZone.points ... is of type string ... double_array` | Jazzy wants a string | `points: "[[x,y], [x,y], ...]"` in the Nav2 YAMLs |
| `planner_server` stuck: `Timed out waiting for transform base_link to map` | `slam_toolbox` is a lifecycle node and stayed unconfigured | `go2_mapping_slam_toolbox.launch.py` now configures and activates it |
| bridge `DISABLED`, topics wrong | launch never set `enabled`, `cmd_vel_topic`, `request_topic` | explicit params, `enable_motion` argument (default false) |

If the image does not yet contain the points/slam fixes, apply by hand in the container
that runs the launch (idempotent, lost on exit):
```bash
F=/opt/lumen_ws/install/share/go2_bringup/config/nav2_params_slam.yaml
sed -i -E 's#(plugin: "nav2_[a-z_0-9]+)/([A-Za-z0-9]+")#\1::\2#' $F
python3 - "$F" <<'EOF'
import re, sys
p = sys.argv[1]
s = open(p).read()
def conv(m):
    nums = [x.strip() for x in m.group(2).split(',') if x.strip()]
    if len(nums) % 2 or len(nums) < 6:
        return m.group(0)
    pairs = ', '.join('[%s, %s]' % (nums[i], nums[i + 1]) for i in range(0, len(nums), 2))
    return '%s"[%s]"' % (m.group(1), pairs)
s, n = re.subn(r'(^\s*points:\s*)\[([-0-9.,\s]+)\]\s*$', conv, s, flags=re.M)
open(p, 'w').write(s)
EOF
# then, after launching:
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
ros2 run tf2_ros tf2_echo map base_link                       # wait for a transform
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```
Check: `ros2 lifecycle get /<node>` is `active [3]` for controller_server, planner_server,
smoother_server, behavior_server, bt_navigator, velocity_smoother, collision_monitor.
Never call the startup service on a stack that is already partly active: reset (3) first.

### Verified / reported
- Verified: all seven Nav2 nodes `active [3]`, `/plan` and the costmaps published, the scan
  publisher and all subscribers are BEST_EFFORT (no silent QoS drop), Nav2 followed a path
  and aborted for lack of progress while the bridge was disabled.
- Reported working by the operator: goals sent from RViz with the bridge enabled.
- Open: `collision_monitor` toggles between "slowdown 50%" and "normal" every 0.05-0.1 s
  (something at the edge of the SlowZone; not investigated). `max_points` is deprecated in
  favour of `min_points`. Inflation (0.40) is below the inscribed radius (0.402): the MPPI
  controller warns of slow collision checks; `robot_radius: 0.4` is large for a ~1.8 m maze
  corridor. All unverified how they affect paths.

### RViz (robot host, TurboVNC)
```bash
source /opt/ros/jazzy/setup.bash   # plus the DDS exports
vglrun rviz2 -d ~/lumen_fleet/sar2026_lumenfleet/image/src/go2_bringup/config/nav2_rviz.rviz
```
Fixed Frame `map`; goals with the **2D Goal Pose** tool (`/goal_pose`). Foxglove 3D fails in
the VirtualBox VM (no WebGL).

## 3. RTAB-Map mapping with the depth camera, driving with the keyboard
The operator found the RTAB-Map map clearly better than the 2D lidar map. (Subjective;
no metric comparison was made.) Lidar is still needed for obstacle detection and the
Nav2 costmaps, so this is a mapping option, not a replacement.

Stop the navigation launch first (one command source). Terminals:
```bash
# 1 container: TF + retimed odometry
sar-run lumen-jazzy:nav bash
ros2 launch go2_bringup go2_online.launch.py driver:=false camera:=false camera_tf:=false rviz:=false

# 2 host: RealSense with aligned depth
source /opt/ros/jazzy/setup.bash; source /opt/sar/realsense_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
  rgb_camera.color_profile:=640,480,30 depth_module.depth_profile:=640,480,30

# 3 host: calibration TF only
source ~/lumen_fleet/sar_new_ws/install/setup.bash
ros2 launch sar_go2_example camera_tf.launch.py camera:=false

# 4 container: join the camera to base_link (front camera from the URDF)
ros2 run tf2_ros static_transform_publisher \
  --x 0.33 --y 0 --z 0.04 --roll -1.5708 --pitch 0 --yaw -1.5708 \
  --frame-id base_link --child-frame-id go2_front_camera_optical

# 5 container: RTAB-Map (install with apt if the image predates the Dockerfile change)
mkdir -p /home/lumenfleet/lumen_fleet/rtab_map_data
ros2 run rtabmap_slam rtabmap --ros-args \
  -p frame_id:=base_link -p odom_frame_id:=odom \
  -p subscribe_depth:=true -p subscribe_rgb:=true -p approx_sync:=true \
  -r rgb/image:=/camera/camera/color/image_raw \
  -r depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
  -r rgb/camera_info:=/camera/camera/color/camera_info \
  -r odom:=/utlidar/robot_odom_retimed \
  -p database_path:=/home/lumenfleet/lumen_fleet/rtab_map_data/rtabmap_depth_$(date +%H%M).db

# 6 container: keyboard
python3 /validation/keyboard_control.py --live
```
Warning `Could not find a connection between 'odom' and 'base_link'` = terminal 1 is not
running (nothing else should publish that transform). Stop RTAB-Map with a single Ctrl-C and
check the database is not 0 MB. Keep `rtab_map_data/` out of git.

Not done: combining lidar with the depth camera in RTAB-Map, feeding the RTAB-Map map to
Nav2, and any quantitative comparison. Hybrid explorer, `explore_lite`, waypoint scripts,
`reactive_avoid` on this robot, and the TurboVNC RViz Nav2 panel (`nav2_rviz_plugins` on the
host) were not tested.

## 3b. Safety zones and the detector/rescue workflow (updated 2026-11-12)

### collision_monitor zones are direction-aware now

`StopZone` / `SlowZone` in `nav2_params*.yaml` are `type: velocity_polygon`, not plain
polygons. A plain polygon decides only on "is a scan point inside this box" and ignores
which way the command would move the robot, so once an obstacle is inside StopZone every
command is zeroed -- including the one that would back out. Measured on a 12-goal
benchmark: 2/12, with nine goals travelling exactly 0.00 m, and both recovery behaviours
failing 10/10 because their escape commands were zeroed too. With direction-aware zones
the same 12 goals score 12/12.

What to watch for on the robot:

* The log must NOT contain `Velocity is not covered by any of the velocity polygons`.
  If it does, a commanded velocity fell outside every sub-polygon's range and the node
  **keeps the previous polygon** -- a silent, stale-geometry failure. Widen
  `linear_min`/`linear_max`/`theta_min`/`theta_max` until it stops.
* `collision_monitor/stop_zone` in RViz flips its box when you command reverse. That is
  the intended behaviour, not a glitch.
* Forward protection is unchanged: the lateral half-widths and the forward extents are
  the same numbers as before; only the trailing extent was cut back.

`stop_pub_timeout` is 10.0 s (was 2.0). The monitor stops publishing after that long of
continuous zero output, and downstream cannot tell that apart from a dead pipeline.
Nothing else publishes `cmd_vel_raw` in this stack, so keeping it talking is free.

### Detector + rescue workflow (`sar_detector_nav_integration/`)

```bash
ros2 launch sar_rescue_manager rescue_mission.launch.py          # rescue_executor:=real (default)
ros2 launch sar_rescue_manager rescue_mission.launch.py rescue_executor:=mock   # dry run
```

* **`rescue_executor` now defaults to `real`**, which sits the robot down for real
  (`StopMove` -> `Sit` -> `RiseSit`) when it reaches a victim. The old `mock` executor
  only answers "done" and changes no posture -- use it for dry runs where the robot must
  stay standing. Operator ready before the first run: the robot really sits.
* `referee_adapter` starts alongside and republishes completions as `/sar/events`
  (`MISSION_START`, `SIT <id>`) so `mission_monitor` can score this workflow with the same
  referee as the other algorithms. It only subscribes; outside simulation nothing listens
  and it is inert.
* Still unverified end to end: this workspace has not run under Docker, DDS or a real
  Nav2 (see `sar_detector_nav_integration/RESCUE_WORKFLOW.md`). Treat the first run as a
  first test, not a rerun.

## 4. Saving work
```bash
# laptop
git add <specific files> && git commit -m "..." && git push origin mapping-nav-explore-hybrid
# robot (read-only token typed into the command, never saved)
git fetch https://<PAT>@github.com/iuenao/sar2026_lumenfleet.git mapping-nav-explore-hybrid
git merge --ff-only FETCH_HEAD && cd image && docker build -t lumen-jazzy:nav .
```
`running_instructions_26_10_2026.md` contains the Tailscale key: never commit it.
