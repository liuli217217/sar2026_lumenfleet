# Detector + Nav 独立集成副本

标记：`DETECTOR_NAV_INTEGRATION`。本目录是新增的集成工作区，原 `image/` 和 `sar-victim-detector/` 不修改。
支持两个运行模式：

- **观测模式**（本文）：Nav2 / 原 hybrid_explorer 继续探索，集成节点只定位、去重、显示和保存，不发导航目标。
- **任务闭环模式**：使用新增 managed_explorer 和 sar_rescue_manager，完成“探索 → 发现 → 接近 → 模拟救援 → 恢复探索”。详见 [RESCUE_WORKFLOW.md](RESCUE_WORKFLOW.md)。该模式不可再启动原探索器。

真实坐下/趴下动作尚未接入；mock 完成不代表物理救援动作完成。

团队拉取与版本说明见 [SHARING.md](SHARING.md)。本目录 Nav 是 `ccefd8a` 的独立快照，
仓库外层 `image/` 已有后续 Nav 优化（发布准备时为 `2b7ffcf`），不会自动同步到本副本。

真机测试见 [2026-11-11 测试计划](TEST_PLAN_2026-11-11.md)，包含分阶段通过条件、命令和数据记录表。

## 目录与来源

- `image/`：从当前导航分支完整复制的镜像工作区，原有 Nav 源文件保持字节一致。
- `image/src/victim_detector_integrated/`：队友 detector 的独立副本，ROS 包和 Python 模块均已重命名；模型随副本安装。
- `image/src/sar_target_mapping/`：新编写的输入关联、深度定位、目标记录与组合 launch。
- `image/src/sar_rescue_manager/`：新编写的任务协调、持久伤员记录、接近点规划，以及重命名的探索器副本。
- `image/Dockerfile.detector`：在 Nav 基础镜像之上增加 detector 的构建文件。
- `SOURCE_BASELINE.json`：复制时的 Git commit 和原文件 SHA256。
- `CHANGES.md`：逐文件修改清单。
- `COLCON_IGNORE`：防止从仓库根目录扫描时同时发现原包和本副本；请进入本目录的 `image/` 构建。

## Input → 处理 → Output

```text
RGB ──→ victim_detector_integrated ──→ detections_confirmed ─┐
                                                        │
对齐 RGB 的 Depth ─────────→ 按时间戳缓存、匹配 ─────────────┤
RGB CameraInfo ───────────→ 内参及畸变校正 ─────────────────┤
map ← camera_optical 的带时间 TF ──────────────────────────┤
                                                        ↓
                                               sar_target_mapper
                                                        ↓
                                         map 目标记录 + MarkerArray + JSON

原 Nav / hybrid_explorer：使用原地图、点云和运动链路，未修改。
```

| 输入 | 类型 | 用途 |
|---|---|---|
| RGB（默认 `/lumen/retimed/rgb`） | sensor_msgs/Image | YOLO 输入；检测输出继承此 header |
| `/sar_detector_integrated/detections_confirmed` | vision_msgs/Detection2DArray | 类别、置信度、像素框、短期 track ID |
| Depth（默认 `/lumen/retimed/depth`） | sensor_msgs/Image | 必须已对齐到 RGB；16UC1 毫米或 32FC1 米 |
| CameraInfo（默认 `/lumen/retimed/rgb_info`） | sensor_msgs/CameraInfo | 必须对应 RGB 几何和相同 optical frame；K、D 和尺寸 |
| `/tf`、`/tf_static` | TF | 使用检测时间查询 `map ← 图像 optical frame` |

| 输出 | 类型 | 用途 |
|---|---|---|
| `/sar_detector_integrated/detections` | Detection2DArray | 每个成功处理帧的原始检测，可为空 |
| `/sar_detector_integrated/detections_confirmed` | Detection2DArray | 累计多次命中的二维检测；bridge 关闭，漏检不沿用旧框 |
| `/sar_integration/targets` | std_msgs/String，JSON | 当前全部目标记录，transient-local 供后加入的订阅者读取 |
| `/sar_integration/targets/markers` | visualization_msgs/MarkerArray | RViz 球形和文本标记，Fixed Frame 设为 map |
| `/sar_integration/status` | std_msgs/String，JSON | 已定位/拒绝/待处理数量、缓存大小、检测到达年龄 |
| `output_file` | JSON 文件 | 当前运行记录快照，默认 `/tmp/sar_targets.json` |

每条记录包含 `id, class_id, position[x,y,z], observations, confidence, first_stamp_ns, last_stamp_ns, source_track_id`。
`confidence` 是最近一次模型分数，不是空间定位置信度；`id` 是本次运行的地图记录编号。

### 数据如何关联

1. 深度最多缓存 30 帧；检测结果晚于图像到达时，从缓存选择时间最接近的深度。
2. 容许默认 ±0.08 秒误差；检测超过 2 秒、零时间戳或明显来自未来时拒绝。等待 TF/深度最长 2 秒。
3. CameraInfo 可为静态标定，允许较早时间，但要求相同 optical frame 和图像尺寸。
4. 框中心附近 40% 区域采样，取有效深度中位数；有效值不足或深度离散过大则拒绝。
5. 对无畸变图像按 `X=(u-cx)Z/fx, Y=(v-cy)Z/fy` 反投影；非零畸变使用 OpenCV 校正光线。
6. 使用检测时间戳的 TF 转成 map 坐标。缺 TF 时等待，不使用 latest TF 替代。
7. 同类别、平面距离 0.6 米内合并，位置取观测均值；不会把 track ID 当成永久伤员 ID。
8. 发布全部记录和标记，并用临时文件替换方式保存 JSON。新进程从空记录开始，不读取上次文件。

## 构建：只操作这个副本

在机器人上，从仓库根目录执行：

```bash
cd sar_detector_nav_integration/image
docker build -t lumen-jazzy:nav-integration-base .
docker build -f Dockerfile.detector -t lumen-jazzy:detector-nav .
```

第二个镜像添加 ROS vision_msgs 与三个新包，并在 `/opt/detector-venv` 安装模型依赖。
Nav 和 mapper 继续使用系统 Python；仅 detector 用虚拟环境 Python，避免 pip 覆盖 Nav 的 NumPy/OpenCV。
副本约束 NumPy<2、opencv-python<4.12；这是基础安装范围，不是已验证的可复现 GPU 版本锁。
原 detector 部署文档中关于 pip OpenCV 的操作不适用于这里：这里仅在独立 venv 中安装兼容 NumPy 1.x 的版本。
默认 CPU，不能据此认定 Jetson GPU 已可用；启用 CUDA 前必须在镜像内验证 CUDA kernel 和真实模型推理。
离线使用必须提前构建，且用包内权重；本机没有 Docker，以上镜像构建尚未执行。

## 运行与 Nav 的关系

沿用机器人的容器启动方式（host 网络、正确网卡、相同 ROS_DOMAIN_ID），将镜像名改为 `lumen-jazzy:detector-nav`。
挂载一个可写目录用于结果保存，例如容器内 `/results`；不挂载时容器删除会丢失 `/tmp` 结果。

1. 相机驱动、建图、Nav2 按原 runbook 分别启动，不重复启动 TF 或机器人驱动。
2. 本地建图流程已发布 `/lumen/retimed/rgb`、`depth`、`rgb_info` 时，在另一个容器终端启动：

```bash
ros2 launch sar_target_mapping detector_mapping.launch.py \
  output_file:=/results/sar_targets_run01.json
```

3. 比赛环境使用原始相机 topic 时，显式改三个参数（实际名称先用 topic list 确认）：

```bash
ros2 launch sar_target_mapping detector_mapping.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  output_file:=/results/sar_targets_run01.json
```

相机三个输入必须与现有定位 TF 使用一致的时间基准。重打戳不能恢复采集时间；独立到达时间会引入定位误差。
相机 optical frame 链必须由现有建图/相机启动文件提供，本集成不另建 TF。

4. 先保持 Nav `enable_motion:=false`，验证检测和标记，再按团队已有实机验证流程运行探索。
5. RViz 添加 MarkerArray，topic 为 `/sar_integration/targets/markers`，可靠性 Reliable、耐久性 Transient Local。
6. 检查：

```bash
ros2 topic echo /sar_detector_integrated/status
ros2 topic echo /sar_integration/status
ros2 topic echo /sar_integration/targets --once --qos-durability transient_local
```

GPU 环境验证通过后，可用 `device:=cuda:0`；`max_rate_hz` 默认 5，标注图默认关闭。
若不使用 Docker，先在可导入 ROS 和 Ultralytics 的 Python 环境中安装三个新包，然后将
`detector_python:=/实际/python3/绝对路径`；该参数用于启动 detector 的解释器前缀。

## 如何与队友更新后的 Nav 合并

不复制回本目录整个 image，以免覆盖队友修改。只移植：

1. `src/victim_detector_integrated/`。
2. `src/sar_target_mapping/`。
3. `src/sar_rescue_manager/`（需要任务闭环时）。
4. `Dockerfile.detector`（可继续 FROM 队友最新 Nav 镜像，通过 `--build-arg NAV_IMAGE=镜像名` 指定）。

本次未修改 Nav launch、Nav 参数、hybrid_explorer 或 cmd_vel 桥接。
所有集成代码及关键修改处标记 `DETECTOR_NAV_INTEGRATION`；详细文件来源见 CHANGES.md。

## 验证与限制

已运行：任务闭环模块的状态、规划与 action 生命周期测试（见 RESCUE_WORKFLOW.md），以及 6 个纯 Python 单元测试（深度格式/字节序/行填充、非法深度、内参、TF、空间去重、真实 detector 初始化与推理阈值）；原跟踪测试在重命名副本上全部通过。
未运行：Docker 构建、ROS topic/launch 联调、真实模型、GPU、RGB-D bag 回放或实机导航。

- 中位深度得到的是框内可见表面估计，可能仍落在背景，不能当作伤员精确中心。
- 0.6 米去重半径可能合并相邻目标，或因定位漂移产生重复，需要实际数据调参。
- 建图闭环修正后，历史 map 坐标不会随之优化；当前适用于首轮标记验证，非最终语义地图。
- 深度缓存 30 帧在 30 Hz 时约 1 秒；较慢推理会导致匹配失败并拒绝观测，可按内存预算调整。
- 未自动截取或保存截图；记录仅含数值与地图标记。
- 队友报告的模型指标未在这里复测；时序实验来自合成序列，不等同于实机召回。
