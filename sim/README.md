# sim —— Gazebo 搜救场景

本目录是**带伤员的 Gazebo 仿真场景**，用来在上真机之前测量「180 秒内能找到几个伤员」。

它和 `image/validation/sim/` 不是一回事。那边是一个光线投射的合成传感器源（4 种纯色表面
加棋盘格），用来测导航和建图，**世界里没有任何人形物体，YOLO 必然检出 0**。这边是真正的
Gazebo 世界，伤员是带真实照片贴图的 mesh，检测器能认。

2026-11-12 从 `SMC_SAR_Phase2` 搬过来，这样整条「场景 → 任务 → 真机」的链路在一个仓库里。

## 目录

```
sim/
├── build.sh        构建 smc-sar-sim:jazzy
├── Dockerfile      ROS 2 Jazzy + Gazebo Harmonic
├── entrypoint.sh
└── src/
    ├── disaster_models/          世界、模型、任务（含伤员真值）
    ├── unitree_go2_sim/          Gazebo 启动 + ros_gz 桥
    ├── unitree_go2_description/  Go2 模型
    └── champ, champ_base, champ_msgs   步态控制
```

**任务侧的包不在这里**。`go2_sar_core` / `go2_sar_scout` / `go2_sar_reflex` /
`go2_sar_prior` / `go2_sar_slam` / `go2_sar_referee` / `go2_real_bridge`
仍然只有 `image/src/` 下那一份，构建时由 `build.sh` 一起打进镜像。这是有意的：
`real_sar.launch.py sim_test:=true` 让仿真走**和真机完全相同的桥接、话题名和参数**，
所以这里的结果对真机才有意义。

## 场地与伤员

默认场地 `disaster_models/worlds/sar_arena_5x8.sdf`：5×8 m，带路障，**场外还有 8 个观众**
（天然的误检干扰项，检测器必须不把他们算进去）。

伤员在 `disaster_models/missions/arena_5x8/`：

| 文件 | 给谁看 |
|---|---|
| `victims_estimated.yaml` | 机器人（有地图方案的先验，误差 ≤0.3 m） |
| `victims_ground_truth.yaml` | **只有裁判 `mission_monitor` 能看**，机器人永远看不到 |
| `map.yaml` / `map.png` | 有地图方案的先验地图（只有墙） |

4 个伤员，2 躺 2 坐，真值带朝向和身体包围盒。

其他可用场地：`disaster_maze_4x4.sdf`、`sar_test_6x8.sdf`、`training_map_withoutAud.sdf`
（无观众版），对应 `missions/maze_4x4`、`missions/test_6x8`。

## 构建

```bash
cd sim && ./build.sh
```

`--network=host` 是必需的，不是可选优化：在默认 bridge 网络下本机容器解析不了
`archive.ubuntu.com` 和 `packages.ros.org`，每个 `apt-get` 要约 390 秒才超时失败，
整条 RUN 以 exit 100 结束。`build.sh` 已经带上了。

## 跑

```bash
# 无地图方案
docker run --rm --device /dev/dri smc-sar-sim:jazzy \
  ros2 launch go2_real_bridge real_sar.launch.py \
      mode:=nomap sim_test:=true enable_motion:=true gui:=false camera:=none debug:=true

# 有地图方案
... mode:=known ...
```

- 要看 Gazebo 界面：去掉 `gui:=false`，并加 `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw`
- 没有 GPU：把 `--device /dev/dri` 换成 `-e LIBGL_ALWAYS_SOFTWARE=1`
- `camera:=none` 指的是**不起 RealSense 驱动**，不是没有相机。仿真里图像由 Gazebo 出，
  `sim_test.yaml` 把 `person_detector.image_topic` 指到 `/rgb_image`，YOLO 照常工作。

判据：日志末尾的 `SCORE n/4 victims found within T s`，过程中每救一个打一条
`EVENT SIT track_N measured_distance=... t=...`。

## 已有成绩

无地图方案，2026-11-09（代码为当时版本）：

```
EVENT SIT track_1 ... t=8.4 s
EVENT SIT track_4 ... t=48.5 s
EVENT SIT track_5 ... t=79.3 s
EVENT SIT track_6 ... t=114.6 s
EVENT DONE found 4/4 in 119.1 s; icp 978/978 scans corrected
SCORE 4/4 victims found within 300 s (4 sit-downs)
  localization error (estimate vs ground truth): mean 0.23 m, max 0.40 m
```

4 个都在 114.6 s 内找到，所以换 180 s 时限同样是 4/4。

## 已知问题

1. **两种模式的时限不一致**。`real_sar.launch.py` 里写死了
   `time_limit: 300.0 if mode == 'nomap' else 180.0`，`real_nomap.yaml` 是 300、
   `real_known.yaml` 是 180，两边注释都说「set to the competition rule」。
   **三个方案要公平对比必须先统一这个数。**
2. **slam+navigation 方案不在这个镜像里**。`build.sh` 只打包任务侧三个包，
   没有 `go2_bringup`、Nav2、RTAB-Map，也没有 `sar_detector_nav_integration/`。
   要把第三个方案拉进同一场地比较，需要扩充这个构建。
3. **reactive 栈的速度低于真机最小可执行速度**。`sar_params.yaml` 的
   `min_forward_speed: 0.22` 是按仿真调的，真机实测前进最小值是 0.30 m/s。
   仿真里不影响（理想执行器），真机上会爬行或抽动——**这是 sim2real 最大的落差**。
