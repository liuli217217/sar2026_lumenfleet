# DETECTOR_NAV_INTEGRATION：探索—接近—模拟救援闭环

本次里程碑是单伤员的模拟救援闭环，同时具备多目标队列、完成记录和有限重试。
没有真实坐下/趴下请求；`COMPLETED` 仅代表模拟动作完成，ledger 中明确保存 `completion_mode: mock`。
代码已实现并做无 ROS 的自动化测试；尚未完成 Docker、DDS、真实 Nav2 和实机验证。

## 1. 控制权与流程

```text
managed_explorer ── PoseStamped 候选探索点 ─┐
                                          ↓
detector → mapper → 空间伤员记录 → sar_rescue_manager → NavigateToPose → 原 Nav2
                                          │
                                          ├→ ComputePathToPose：接近点路径验证
                                          └→ mock_rescue：模拟动作，绝不发布运动/姿态指令
```

原始 `hybrid_explorer.py` 原样保留。新增 `managed_explorer.py` 是有来源注释的重命名副本。
它只发送候选点，不再持有 Nav2 action。协调器是本工作流唯一的 NavigateToPose 客户端。
不能同时启动原 `hybrid_explorer`、`explore_lite`、其他自动任务发起器或用 RViz 手动发目标。
协调器不能阻止外部客户端抢占 Nav2；这一点需要启动流程保证。

| 状态 | 行为 / 离开条件 |
|---|---|
| DISABLED | 默认状态，不发送导航目标；等待显式 enable |
| EXPLORING | 接收探索候选点，转发单个 Nav2 任务；持续选择新鲜、确认后的伤员 |
| PAUSING_EXPLORATION | 禁止探索候选点；取消当前任务，等待最终结果 |
| SELECTING_APPROACH | 在伤员周围 1m 环上生成 12 个候选点，检查全局代价地图、未知区和机器人圆形空间，再逐点请求路径 |
| APPROACHING | 发送接近点任务；终点朝向伤员，不因检测抖动反复换目标 |
| VERIFYING_TARGET | 确认实际到达、目标距离合理、里程计持续静止 1s，并有到达之后的新目标观测 |
| RESCUING | 使用唯一 token 请求模拟动作，等待匹配结果，机器人应保持站立和静止 |
| EXPLORING | 持久化完成状态之后恢复探索，选择下一伤员 |
| STOPPING | 禁用服务触发；等待导航/规划最终结果，然后转为 DISABLED |
| FAULT | 取消请求但不再发新目标，需诊断后重启；不会把失败当作完成 |

取消请求的 ACK 不代表机器人任务已经终止。即使取消发生在 action 尚未 accepted 时，
协调器也会等待响应，拿到 handle 后再取消，最终结果到达前不会交接到下一个任务。
取消/接受状态超过 5s 仍不确定时进入 FAULT；不会凭超时就假定机器人已停。

## 2. Input / Output 接口

| 接口 | 类型 | 输入 / 输出与使用方式 |
|---|---|---|
| `/sar_integration/targets` | std_msgs/String JSON | mapper → manager；只选择 mannequin，忽略 cone/first_aid |
| `/sar_rescue/exploration_goal` | geometry_msgs/PoseStamped | managed_explorer → manager；候选点，非直接运动命令 |
| `/sar_rescue/exploration_control` | std_msgs/String JSON | manager → explorer；enabled/busy，0.2s 心跳，超过 0.8s 无心跳停止提议 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | manager 输入；必须 map frame |
| `/global_costmap/costmap_updates` | map_msgs/OccupancyGridUpdate | manager 输入；应用增量数据，避免只使用初次完整快照 |
| `/local_costmap/costmap[_updates]` | OccupancyGrid / OccupancyGridUpdate | managed_explorer 输入；判断短距离前方是否阻塞 |
| `odom_topic` | nav_msgs/Odometry | manager 输入；默认 `/utlidar/robot_odom_retimed`，用于数据新鲜度和静止确认 |
| TF `map ← base_link` | TF | manager 输入；机器人当前地图位置，要求时间有效且新鲜 |
| `navigate_to_pose` | nav2_msgs/action/NavigateToPose | manager → Nav2；探索与接近共用同一所有权槽 |
| `compute_path_to_pose` | nav2_msgs/action/ComputePathToPose | manager → Nav2；候选点可达性、终点和结果状态验证 |
| `/sar_rescue/set_enabled` | std_srvs/srv/SetBool | 人工控制；true 启动任务，false 开始取消并等待停止 |
| `/sar_rescue/action_request` | std_msgs/String JSON | manager → mock；token、target_id、mode=mock |
| `/sar_rescue/action_result` | std_msgs/String JSON | mock → manager；回传 token、target_id、mode、success，错误 token 无效 |
| `/sar_rescue/status` | std_msgs/String JSON | manager 输出；状态、原因、当前目标、action 占用、全部目标记录 |
| `ledger_file` | JSON 文件 | coordinator 独占写入；保存候选/确认/执行/完成/暂缓状态与重试次数 |

### 目标确认与持久化

- mapper 的 track 确认后，manager 仍要求至少 3 个不同观测时间、跨度至少 0.4s 的地图观测。
- mapper 每次输出完整列表；重复快照不累计 hits，避免旧数据被误认为新证据。
- 仅选择最近 3s 观测过的目标，按距离优先；进行中的目标不会被新目标抢占。
- 同一位置 0.6m 内匹配持久目标编号；track ID 改变不重新计数。
- 每目标默认最多尝试 2 次；失败后等待 30s 并需要新鲜观测才能重试。耗尽次数保留 DEFERRED。
- 完成状态用 fsync + 原子替换落盘后才恢复探索；磁盘写入失败进入 FAULT。
- 重启后 IN_PROGRESS 转为 DEFERRED，COMPLETED 保留；进程仍然从 DISABLED 启动。
- `mission_id` 标识同一张地图/同一场任务；换地图应换 mission_id 和 ledger 文件路径。
  ID 不匹配或 JSON 损坏时启动失败，防止无意覆盖。路径不要由两个 manager 同时使用。
- mapper 的原始观测文件与 manager 的 ledger 是两种文件：前者记录感知结果，后者记录任务完成情况。

## 3. 构建与运行顺序

使用 README.md 的两个 Docker 构建命令重建；增量镜像现在包含三个集成包。
启动容器时沿用原比赛网卡和 host 网络方式；持久化挂载 `/results`，不能只依赖容器内 /tmp。

### 第一步：只验证启动与接口

1. 单独启动相机与原建图流程。
2. 单独启动原 `go2_navigation_rtabmap.launch.py`，保持 `enable_motion:=false`。
3. 不启动原探索器或独立 detector_mapping launch（组合 launch 已包含 detector + mapper）。
4. 启动新任务组合：

```bash
ros2 launch sar_rescue_manager rescue_mission.launch.py \
  mission_id:=arena_trial_01 \
  output_file:=/results/observations_trial01.json \
  ledger_file:=/results/mission_trial01.json
```

原始比赛相机 topic 或 odom 名不同，则覆盖 rgb_topic、depth_topic、camera_info_topic、odom_topic。
默认 CPU、限速 5Hz；GPU 验证后才能设置 device:=cuda:0。
状态应为 DISABLED，Nav2 不会因这个组合 launch 自动获得任务。

```bash
ros2 topic echo /sar_rescue/status
ros2 topic echo /sar_integration/status
```

### 第二步：人工启用任务

在仿真、隔离环境，或完成团队实机准备后，调用：

```bash
ros2 service call /sar_rescue/set_enabled std_srvs/srv/SetBool '{data: true}'
```

启用条件：Nav2 两个 action server 就绪，全局代价地图新鲜（3s），里程计新鲜（0.5s），TF 新鲜（1s）。
运动桥接仍由原 Nav launch 的 enable_motion 独立控制。桥接关闭时可以验证接口与规划，
但不会实际到达，因此不能期待完整的接近成功或模拟救援完成。
真实走动要在对应环境中启用原运动桥接，且保持唯一控制源。

### 第三步：停止

```bash
ros2 service call /sar_rescue/set_enabled std_srvs/srv/SetBool '{data: false}'
```

等待 `/sar_rescue/status` 到 DISABLED，确认机器人停止后再关闭进程。
该服务不是物理急停。协调器被 kill/崩溃时，Nav2 可能继续执行已提交目标；
explorer 的心跳过期只阻止新候选点，不是 Nav2 的进程死亡看门狗。

## 4. 自动验证命令

从仓库根目录运行：

```bash
python3 -m unittest discover -s sar_detector_nav_integration/image/src/sar_rescue_manager/test -v
python3 -m unittest discover -s sar_detector_nav_integration/image/src/sar_target_mapping/test -v
python3 sar_detector_nav_integration/image/src/victim_detector_integrated/tools/test_tracker.py \
  sar_detector_nav_integration/image/src/victim_detector_integrated/victim_detector_integrated/detector_node.py
```

mission 测试执行真实 manager 类的状态处理函数，用 fake message/future/client 替代 ROS transport。
它验证状态与 action 生命周期，但不验证 ROS 消息版本、DDS QoS、launch 或 Nav2 实际行为。

## 5. 下一次机器人联调的验收顺序

1. 机器人静止：检查三个相机输入、TF 和不同观测时间，验证目标确认与地图位置。
2. 无运动桥接：状态启动、启用条件、服务禁用、无可达目标处理。
3. 低速、一个已知 mannequin：探索 → 取消 → 接近 → 新观测 + 静止 → mock → 恢复。
4. 回头再次看同一伤员：确认 COMPLETED 不重复救援；重启并读取同一 ledger 再验证。
5. 两个伤员：完成当前任务后再选下一个；不可达目标不堵住后续任务。
6. 失败注入：断深度、断 TF、取消无响应、mock 无响应，确认不会错误标记完成。

尚未实现真实姿态动作、机器人状态确认/恢复站立、地图闭环后的历史目标重投影、
探索完成判定，以及真实机器人端 ROS 集成测试。坐下动作接入前还需要明确 API 和完成反馈。
当前 mock 阶段不会触发任何趴下、坐下或站起指令。
