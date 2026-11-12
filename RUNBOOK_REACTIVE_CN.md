# 操作手册 —— 四个搜救算法：仿真与真机 Go2 部署

*English: [RUNBOOK_REACTIVE_EN.md](RUNBOOK_REACTIVE_EN.md)*

这份手册覆盖我们自己写的四个算法 —— `Scout`、`Reflex`、`Prior`、`SLAM` ——
从一台干净的机器到 Gazebo 里跑出一个分数，再到真机上。


这里的一切都走**同一个 launch 文件** `go2_real_bridge real_sar.launch.py`，
仿真和真机都是。这是有意的：加上 `sim_test:=true`，仿真走的是**和真机完全相同的
桥接、话题名和参数文件**，所以在 Gazebo 里测出来的分数对真机才有意义。

| | 名称 | 包 | `mode:=` | 需要什么才能工作 |
|---|---|---|---|---|
| A1 | **Scout** | `go2_sar_scout` | `scout` | 雷达、IMU、相机 |
| A2 | **Reflex** | `go2_sar_reflex` | `reflex` | 雷达、IMU、相机。**完全不用位姿估计** |
| A3 | **Prior** | `go2_sar_prior` | `prior`（别名 `known`） | 雷达、IMU、相机，**外加先验地图和伤员估计位置** |
| A4 | **SLAM** | `go2_sar_slam` | `slam`（别名 `nomap`） | 雷达、IMU、相机 |

共用代码在 `go2_sar_core`（YOLO 检测器、点云和地图工具）。
`go2_sar_referee` 是 Gazebo 裁判，**有意不装到机器人上** —— 它读的是仿真器真值。

`Reflex` 是兜底方案：不建图、不做扫描匹配、不规划路径，所以当雷达链路、里程计
或算力撑不起 SLAM 时它还能工作。它的定位是「比没有强」，不是「比 A4 强」。

---

# 第一部分 —— 仿真

## A.0 需要什么

- Docker，以及一个能跑 Docker 的用户。
- `/dev/dri` 下的 GPU 渲染节点（没有的话，把 `--device /dev/dri` 换成
  `-e LIBGL_ALWAYS_SOFTWARE=1`，一切照跑，只是慢）。
- 本仓库**旁边**有一份 **`SMC_SAR_Phase2`** 的检出。Gazebo 的网格和贴图
  （约 100 MB 未经修改的 Fuel 资产）**不在 git 里** —— 见
  [`sim/src/ASSETS.md`](sim/src/ASSETS.md)。它们在构建时被注入。

```
~/SAR/
├── sar2026_lumenfleet/     本仓库
└── SMC_SAR_Phase2/         资产来源（默认位置）
```

## A.1 构建仿真镜像

```bash
cd sar2026_lumenfleet/sim
./build.sh                                     # 资产取自 ../SMC_SAR_Phase2
GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 ./build.sh   # 资产在别处
IMAGE=smc-sar-sim:test ./build.sh              # 换一个 tag 构建
```

默认 tag `smc-sar-sim:jazzy`。冷启动大约 10–15 分钟。

资产缺失时构建会**立刻停下**，并明确指出它想要的那两个目录 —— 不会先构建出一个
半成品镜像、让你跑到 300 秒才发现是坏的。

`build.sh` 从**一个**仓库组装上下文：`sim/src/`（Gazebo、CHAMP、Go2 模型、世界）
加上 `image/src/`（任务包，和机器人跑的是同一份文件）。任务包**不重复存放**。

## A.2 跑一次任务

```bash
docker run --rm --device /dev/dri smc-sar-sim:jazzy \
  ros2 launch go2_real_bridge real_sar.launch.py \
      mode:=slam sim_test:=true enable_motion:=true gui:=false camera:=none debug:=false
```

| 参数 | 含义 |
|---|---|
| `mode:=` | `scout` \| `reflex` \| `prior` \| `slam`（`nomap`/`known` 仍可用） |
| `sim_test:=true` | 经 `fake_go2` 跑 Gazebo，并启动裁判 |
| `enable_motion:=true` | **不加这个机器人不会动**（默认是空跑） |
| `gui:=false` | 只起 Gazebo 服务端。`true` 需要 X 显示 |
| `camera:=none` | 不起 RealSense 节点 —— 仿真相机本来就在 |
| `debug:=true` | 每个控制周期一行状态；日志很大，排查时才用 |
| `world:=` | 换一个 `.sdf`；默认是 5×8 m 训练场 |
| `extra_params:=` | **最后**加载的参数文件，覆盖其他一切 |

任务会先等约 40 秒仿真时间让 CHAMP 控制器把机器人站起来，然后跑 300 秒。

## A.3 怎么看结果

裁判根据 Gazebo 真值打印：

```
SCORE 2/4 victims found within 300 s (2 sit-downs)
  victim_1: not found
  victim_2: found at t=165.6 s, 0.79 m away
  victim_3: not found
  victim_4: found at t= 20.2 s, 0.65 m away
  localization error (estimate vs ground truth): mean 0.22 m, max 0.36 m
```

- 只有机器人**真的坐下**并且距伤员 **1.0 m 以内**才算分。坐在空地上的那些会计入
  括号但不得分 —— `2/4` 配 `(2 sit-downs)` 说明两次都有效；`1/4` 配
  `(3 sit-downs)` 说明有两次是假的。
- `Scout` 和 `Reflex` 的 `localization error` 会是 `nan`：它们不发布位姿估计，
  裁判没东西可比。这不是故障。
- `localization error` 这串字里含 "error"。**不要**用 `grep error` 判断跑挂了 ——
  一次干净的运行里，匹配项只有这一行和一条关于 `nan` 的 NumPy `RuntimeWarning`。

## A.4 四个都跑

**一次只跑一个**。它们共用一个 DDS 域，并行会互相看到对方的话题；而且 Gazebo 很吃
CPU，并行跑会让两边的时序都失真。

> **会让你白白损失半小时的坑。** `ros2 launch` 响应的是 **SIGINT**，不是 SIGTERM。
> `timeout 560 docker run ...` 发的是 SIGTERM，容器不理，于是裁判的分数早就打出来了，
> 队列却卡在第一个任务。要么用 `timeout -s INT`，要么照验证脚本的做法：后台运行，
> 看到分数出现就停容器。

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

每个任务约 6 分钟。

## A.5 不重建镜像改参数

`extra_params` 在所有其他文件之后加载，所以它说了算：

```bash
docker run --rm --device /dev/dri -v "$PWD/tune:/tune" smc-sar-sim:jazzy \
  ros2 launch go2_real_bridge real_sar.launch.py mode:=slam sim_test:=true \
      enable_motion:=true gui:=false camera:=none extra_params:=/tune/my.yaml
```

裁判也读它，所以可以从外部把各模式的时限统一 —— 想让两个分数可比，这是必须的。

参数按**节点名**索引，现在节点名和 mode 是同一个词：`sar_scout`、`sar_reflex`、
`sar_prior`、`sar_slam`，另外还有 `person_detector`、`go2_bridge`、`fake_go2`、
`mission_monitor`。**挂在一个没有节点使用的名字下面的配置段会被静默忽略** ——
这是「改了参数却什么都没发生」最常见的原因。

## A.6 目前测到的分数，以及一个未解的问题

Gazebo 5×8 场地，300 秒，四名伤员，各跑一次，2026-11-13：

| 算法 | 得分 | 备注 |
|---|---|---|
| A1 Scout | 2/4 | 可复现 |
| A2 Reflex | 0/4 | 在距伤员 0.74 m 处放弃；见下 |
| A3 Prior | 1/4 | 罚函数自围困，见 [`NAVIGATION.zh.md`](NAVIGATION.zh.md) |
| A4 SLAM | 2/4 | **换相机之前是 4/4 —— 未解决** |

> ⚠️ **未解问题：深度相机让 A4 掉了一半分。** 2026-11-12 把仿真相机从 `camera`
> 换成了 `rgbd_camera`。内参完全相同（1.367 rad FOV、640×480、10 Hz），但**同样的
> 代码**下 A4 从 **4/4** 掉到 **1/4–2/4**，原因始终是**第一名伤员的确认晚了约 14 秒**
> （换之前 t≈1 s，换之后 t≈15–19 s），之后整条轨迹就岔开了。已经用「拆包前的代码 +
> 换相机后的镜像」做过对照，得 1/4，所以**和包重命名无关**。
> **不要把 A4 的 2/4 当成它的真实能力，在这件事查清之前也不要照这些数字调参。**

> ⚠️ **A2 的 0/4 是门槛变严，不是退步。** 之前的 1/4 来自三次坐下，其中两次是假的。
> 修掉一个 `dt` bug 之后假的没了；剩下那个真目标又被 `victim_fresh_time: 1.5` 拒绝了
> —— 躺在地上的人在近距离会掉出相机的垂直视场，最后一次看见的时间就过期了。机器人
> 走到 0.74 m（`sit_distance` 是 0.75）然后放弃。显而易见的修法是**进入 `sit_distance`
> 之后放宽新鲜度窗口**，还没做。

---

# 第二部分 —— 真机 Go2

> **第二部分里涉及运动的步骤，没有任何一条在比赛机器人上跑过。** 传感器、网络和
> 空跑步骤在机器人 `seek` 上验证过（2026-10-26）；任务本身只动过仿真机器人。
> 把每一个运动步骤都当成第一次，操作员手里必须拿着物理遥控器。

## B.0 动手之前

1. 一名操作员手持遥控器、眼睛盯着机器人、拇指随时能按。
2. 地面清空：正前方至少 2 m，并且知道墙在哪。
3. 知道三种停法：遥控器、`ros2 topic pub /sar/estop`、
   `ros2 run go2_real_bridge sport_cli damp`（电机卸力 —— 机器人会**摔到地上**，
   仅限紧急情况）。

## B.1 网络和 DDS —— 每个 shell，主机和容器都要

```bash
source /opt/ros/jazzy/setup.bash                 # 仅主机 shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'$SAR_ROBOT_IFACE'"/></Interfaces></General></Domain></CycloneDDS>'
ros2 daemon stop
```

往下走之前先验证：

```bash
echo $SAR_ROBOT_IFACE          # seek 上是 `end0` —— 每台机器不同
ip -br addr show $SAR_ROBOT_IFACE
ros2 topic list | grep utlidar
```

话题列表为空，按顺序查：ROS 没 source、`CYCLONEDDS_URI` 没设或写错了网卡、
daemon 是旧的。

容器入口脚本会从 `SAR_ROBOT_IFACE` 或 `ROBOT_IFACE` 自动拼出 `CYCLONEDDS_URI`，
两个都没设、或网卡不存在时**直接拒绝启动**。用 host 网络模式。

## B.2 构建并进入真机镜像

```bash
cd ~/sar2026_lumenfleet/image
docker build -t lumen-jazzy:sar .
sar-run lumen-jazzy:sar bash
```

这和仿真镜像不是同一个：没有 Gazebo，多了 RealSense、RTAB-Map 和 Nav2。里面装了
`go2_sar_core`、`go2_sar_scout`、`go2_sar_reflex`、`go2_sar_prior`、`go2_sar_slam`
和 `go2_real_bridge` —— 和仿真构建的是同一份源码，**不含** `go2_sar_referee`。

容器里新建的文件属主是 root，`/tmp` 退出即失。想保留的东西放在挂载的仓库里。

## B.3 查清谁已经在控制机器人

**两个控制器同时指挥一台机器人是「不安全」，不只是「冗余」。**

```bash
ros2 node list
ros2 topic info /api/sport/request -v
ros2 topic list | grep utlidar
```

- 已经有 `/utlidar/*` → 主办方的驱动在跑。用那些话题，**不要**再起第二个驱动。
- 已经有东西在往 `/api/sport/request` 发布 → 先搞清楚那是什么，再考虑把
  `go2_bridge` 叠上去。

## B.4 相机

D435i 驱动跑在**主机上**，不在我们的容器里（镜像里没有 librealsense），而且同一时间
只能有一个进程在推流（`pgrep -a realsense`）。

```bash
ros2 launch sar_go2_example camera_tf.launch.py     # 驱动 + 主办方的标定 TF
```

然后用 `camera:=host`（默认值）启动。只有在你已经把 USB 设备透传进容器、并且没有别的
进程在推流时，才用 `camera:=container`。

检测器读的是 `/camera/camera/color/image_raw`。任务开始前确认它是活的 ——
**没有图像的任务什么都找不到，而且不会报错。**

## B.5 验证传感器几何 —— 这一步不能跳

```bash
CFG=/opt/lumen_ws/install/share/go2_real_bridge/config

# A1 Scout 和 A2 Reflex —— 必须重命名节点。ROS 2 按节点名索引参数文件，而
# check_sensors 这个节点叫 sar_slam，不加 remap 的话 sar_scout: / sar_reflex:
# 段匹配不上、被静默丢弃，于是检查的是内置默认值而不是任务真正会用的值。
# 节点会把最终生效的几何打印出来，没有任何覆盖到达时还会告警——先读那一行，
# 再决定要不要相信结果。
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_scout  --params-file $CFG/real_scout.yaml
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_reflex --params-file $CFG/real_reflex.yaml

# A3 Prior 和 A4 SLAM —— 本来就叫 sar_slam，不需要 remap
ros2 run go2_real_bridge check_sensors --ros-args --params-file $CFG/real_nomap.yaml
```

不发任何运动指令。打印里程计、IMU、雷达、相机的消息频率，以及最关键的那部分：
套用 `lidar_xyz` / `lidar_rpy` 并用 IMU 调平之后，**前 / 左 / 右 / 后四个方向的地面高度**。

- 四个方向的地面高度必须大致相等，并且接近 `ground_z_init`（`-0.32`）。如果不是，
  雷达外参就是错的，**所有算法里的每一项地面和障碍物高度判断都跟着错**。
- 看一下有多少点落在 `body_box` 里 —— 那是机器人看见自己的腿，算法会把它切掉。

有两个值**因机器而异，必须在你拿到的那台机器上实测**：

- `camera_xyz`（`[0.33, 0.0, 0.04]`）：主办方给每台机器的 D435i 标定会在此基础上
  多出几厘米，而且 URDF 里的安装位置和标定对不上。**以标定为准。**
- `lidar_xyz` / `lidar_rpy`：`[0,0,0]` 是对的，但**前提是** `cloud_in` 指向
  `/utlidar/cloud_base` —— 固件已经把它转换到 `base_link` 了。如果你改成指向原始的
  `/utlidar/cloud`，就必须把 URDF 的值填回去（`[0.28945, 0, -0.046825]`，
  rpy `[0, 2.8782, 0]`）—— 而把这些值套到 `cloud_base` 上等于转换了两次。

## B.6 空跑 —— 不动

```bash
ros2 launch go2_real_bridge real_sar.launch.py mode:=slam
```

`enable_motion` 默认 `false`：整套栈照常运行，算法照常规划和决策，
**但没有任何运动请求会到达机器人**。要看：

- `go2_bridge` 打印出 `odom_out` 和 `imu_out` 各自是由哪个源喂的；
- 检测器报 `persons: N`；
- 任务自己的状态迁移（`WAIT → SPIN → EXPLORE → …`）。

三样都正常，才往下走。

## B.7 第一次运动，尽可能小

操作员就位之后：

```bash
ros2 run go2_real_bridge sport_cli stand_up
ros2 run go2_real_bridge sport_cli move 0.3 0 0 1      # 前进 1 秒，然后 StopMove
```

然后是开启运动的任务：

```bash
ros2 launch go2_real_bridge real_sar.launch.py mode:=slam enable_motion:=true
# A3 需要先验地图和伤员估计位置：
ros2 launch go2_real_bridge real_sar.launch.py mode:=prior \
    mission:=/missions/real_field enable_motion:=true
```

桥接层会把速度限制在 `max_vx 0.5`、`max_vy 0.3`、`max_wz 0.8`，并且在指令超过
`cmd_timeout`（0.5 秒）没更新时停住机器人。

## B.8 急停

```bash
ros2 topic pub --once /sar/estop std_msgs/Bool "{data: true}"      # StopMove，忽略一切运动
ros2 topic pub --once /sar/estop std_msgs/Bool "{data: false}"     # 解除
```

桥接层会打印 `EMERGENCY STOP`，并且在解除之前拒绝每一条运动指令。
物理遥控器的权限仍然高于这一切。

## B.9 参数是怎么走到机器人上的

每个任务加载四个文件，后面的覆盖前面的：

```
<算法>_params.yaml      算法调参            仿真和真机共享
real_common.yaml        桥接 + 检测器        共享
real_<模式>.yaml        硬件和场地           机器人的值
sim_test.yaml           （仅仿真）           把仿真的值还原回去
```

这个划分很重要：`real_*.yaml` **只放属于机器人或场地的东西**——相机内参、雷达外参、
地面高度、步速下限、机身宽度、虚拟墙。其余全部留在算法自己的 `*_params.yaml` 里，
这样**在仿真里调出来的参数就是真机上跑的参数**。

`real_*.yaml` **在仿真里也会加载**。所以它里面每一个硬件值，`sim_test.yaml` 里都必须有
对应的仿真值去还原，否则机器人的数字会灌进仿真。加值的时候两边要一起加。

| 模式 | 算法文件 | 真机文件 |
|---|---|---|
| `scout` | `go2_sar_scout/config/scout_params.yaml` | `real_scout.yaml` |
| `reflex` | `go2_sar_reflex/config/reflex_params.yaml` | `real_reflex.yaml` |
| `prior` | `go2_sar_prior/config/prior_params.yaml` | `real_known.yaml` |
| `slam` | `go2_sar_slam/config/slam_params.yaml` | `real_nomap.yaml` |

## B.10 拿到机器人之后必须实测的

下面每一项都是仿真器给不出来的数字。文件里都填了值，**但在实测之前那只是占位**。
按「错了之后哪个先崩」排序。

### 1. 雷达外参和地面高度 —— 第一件事

```bash
ros2 run go2_real_bridge check_sensors --ros-args -r __node:=sar_scout --params-file $CFG/real_scout.yaml
```

它打印的**前 / 左 / 右 / 后**四个方向的地面高度必须大致相等，并且接近
`ground_z_init`（−0.32）。如果随方向变化，雷达外参就是错的，**所有算法里的每一项
地面判断、每一个障碍物高度、每一个伤员距离都跟着错**。这一条不平之前，下面任何一项
都不值得做。

`lidar_xyz: [0,0,0]` 之所以正确，**前提是** `go2_bridge` 的 `cloud_in` 指向
`/utlidar/cloud_base`，固件已经把它转到 `base_link` 了。如果改用原始的
`/utlidar/cloud`，必须填回 URDF 的值（`[0.28945, 0, −0.046825]`，
rpy `[0, 2.8782, 0]`）；把这些值套到 `cloud_base` 上等于转换了两次。

### 2. 相机 —— 确认 camera_info 真的收到了

Scout 和 Reflex 现在都从 `camera_info` 取相机模型。启动后几秒内要看到这一行：

```
camera model from camera_info: fx=... cx=... cy=... (宽x高)
```

如果看到的是告警 `no camera_info on ... yet -- still using the fallback camera
model`，说明节点正在用 640×480 的仿真相机算方位角。主办方的 launch 推的是
1280×720，**方位角会直接差一倍**；对 Reflex 还会毁掉它用来排除误检的表观尺寸判断，
**静默地把真伤员筛掉**。先把话题修对，**不要带着这个告警开跑**。

粗略校验：`fx` 应该约等于 `宽 / (2·tan(hfov/2))`。640 宽、约 69° 的话大概是 465；
仿真相机报的是 392.9。

### 3. `camera_xyz` —— base_link 到彩色光心

文件里填的是 `[0.33, 0.0, 0.04]`，取自 Go2 的 URDF 链。主办方给每台机器的 D435i
标定会在此基础上多出几厘米，而且 URDF 自己的 RealSense 安装位置和标定对不上。
**以标定为准**，不要用 URDF。这里差几厘米就是伤员位置差几厘米，而判定半径只有 1.0 m。

### 4. `velocity_scale` —— 只影响 Scout，但决定 Scout 能不能用

Scout 没有 SLAM，它的位置来自航位推算：
`走过的距离 = 指令速度 × velocity_scale × dt`。文件里的 `0.6` 是在仿真里对 CHAMP
标定的。真机上一定是另一个数，而这个数错了，**Scout 的整个位置估计就漂**——它会坐在
"它以为伤员在的地方"，而不是伤员真正在的地方。

```bash
python3 /validation/move_distance.py --live --speed 0.3 --distance 2.0
```

用卷尺量实际走了多远，`velocity_scale = 实际 / 指令`。**用两个速度各测一次**；
如果比值不是常数，说明这个模型不成立，应该改用 `pose_source: odom` ——
`/utlidar/robot_odom` 是机器人自己的 ~150 Hz 里程计，多半比 CHAMP 的好，而
CHAMP 的里程计差正是航位推算在仿真里胜出的唯一原因。

### 5. `field_bounds` —— 只影响 Scout

文件里填的是 `[0,0,0,0]`，意思是**没有虚拟墙**。量出真实场地，填
`[x_min, x_max, y_min, y_max]`，坐标系的原点是机器人的起始位姿。填错的框要么把机器人
关住、要么让它跑出场；不填则完全依赖雷达看见墙。

### 6. `min_forward_speed` 和 `scan_angular_speed`

两个真机文件里都设了 `min_forward_speed: 0.30`，因为真 Go2 在这之下不动——`0.22`
是 CHAMP 的值，在真机上低于最小可执行速度，结果是爬行或抽搐。**确认 0.30 在你那台机器
上确实是下限**；如果还在原地踏步就往上调。

`scan_angular_speed: 0.60`（Scout）是猜的：CHAMP 和真 Go2 的原地偏航速率不同，而且
`go2_bringup` 把 `max_wz` 卡在 0.8。**给一次 360° 转圈计时**，调到让一次扫描占任务时间
的比例合理为止。

### 7. 检测阈值 —— 用真实图像，在第一次计分跑之前

`detection_min_score`（Scout 0.50、Reflex 0.40）以及 Reflex 的表观尺寸排除，都是针对
Gazebo 的低模人物和墙外的仿真观众标定的。真人、真光照、真遮挡下 YOLO 的分数分布完全
不同。**录一段真实场地带人的 bag**，把检测器跑一遍，看真伤员和旁观者各自的分数分布，
再决定这个下限。

### 8. 坐下时序

`go2_bridge` 把低于 `sit_threshold`（−0.04）的 body pose 转成真正的 Sit，并等待
`rise_settle`（3.0 s）。Scout 的 `stand_up_duration` 设成了 3.5 s 就是为了超过它。
如果机器人还在起身就开始走，往上调。

---

# 第三部分 —— 排错

| 现象 | 原因 | 处理 |
|---|---|---|
| 构建停在 "Gazebo meshes and textures not found" | 资产不在仓库旁边 | `GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 ./build.sh` |
| 打印了 `SCORE` 之后容器还在跑 | `ros2 launch` 不理 SIGTERM | `docker stop`，或 `timeout -s INT` |
| 机器人完全不动，也不报错 | `enable_motion` 还是 `false` | 加 `enable_motion:=true` |
| 真机上开了运动还是不动 | `min_forward_speed 0.22` 低于真机的下限 | 改成 `0.30`（B.9.1） |
| 改了参数毫无反应 | 配置段挂在一个没有节点使用的名字下 | 挂到 `sar_scout` / `sar_reflex` / `sar_prior` / `sar_slam` |
| 任务什么都找不到，检测器无输出 | 相机话题上没有图像 | 检查 `camera:=`，以及主机上 `pgrep -a realsense` |
| `check_sensors` 里四个方向地面高度不一致 | 雷达外参错了 | 见 B.5 |
| 真机上 `ros2 topic list` 为空 | ROS/DDS 环境或 daemon 是旧的 | 见 B.1 |
| `Unable to parse the value of parameter robot_description as yaml` | xacro 的 XML 注释里有 `冒号+空格` | 去掉它；launch 里的 `ParameterValue(value_type=str)` 也能挡住 |
| 日志里一堆 "error" | 裁判的 `localization error` 那行 | 不是故障（见 A.3） |
| 出现 `no camera_info on ... yet` 告警 | camera_info 话题错了或驱动没跑 | 见 B.10.2 —— **带着这个告警不要开跑** |
| `check_sensors` 告警 "no parameter overrides reached this node" | 文件的段挂在别的节点名下 | 加 `-r __node:=sar_scout` / `sar_reflex`（见 B.5） |
| 仿真跑里出现了真机的值 | 某个 `real_*.yaml` 项在 `sim_test.yaml` 里没有对应 | 补到 `sim_test.yaml`（见 B.9） |

---

# 第四部分 —— 哪些没测过

- **没有任何一次任务驱动过真机。** 第二部分 B.7 之后的一切都是第一次。
- B.10 里的每一个数字 —— sim2real 的管线已经打通并在仿真里验证过，但**数值本身在实测
  之前都只是占位**。
- **A3 Prior 和 A4 SLAM** 在真机上的 `min_forward_speed`：`real_known.yaml` /
  `real_nomap.yaml` 里仍然是 CHAMP 的 0.22。A1、A2 已经修了，这两个没有。
- A4 在换深度相机之后的得分（A.6）—— 没有解释。
- A2 的近距离坐下门槛（A.6）—— 已诊断，未修复。
- A3 的罚函数自围困 —— 已在 [`NAVIGATION.zh.md`](NAVIGATION.zh.md) 诊断，未修复。
