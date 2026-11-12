# 仿真传感器源（无真机跑 SLAM / Nav2）

在没有 Go2、没有 RealSense、没有录包的情况下，把 SLAM 和 Nav2 整条链路跑起来并在
GUI 里看结果。合成一个 10 × 7 × 2.6 m 带两根柱子的房间，机器人由 `/cmd_vel` 驱动。

| 文件 | 作用 |
|---|---|
| `fake_go2_sensors.py` | 合成 3D 点云 / RGB-D / 里程计 / TF / 关节角，话题名与真机一致 |
| `run_sim_nav.sh` | 一条命令起：仿真源 + robot_state_publisher + SLAM(+rtabmap_viz) + Nav2(+RViz) |
| `run_sim_slam.sh` | 只跑 SLAM + GUI，不起 Nav2 |
| `patrol.sh` | 在容器里循环发 6 个 Nav2 航点，让机器人持续走动 |

用法、验收检查和排障见仓库外层的 `ToRun.md`。

发布的话题（与真机链路对齐，所以 launch 文件不用改一行）：

```
/utlidar/cloud_deskewed_retimed   PointCloud2   base_link，喂 RTAB-Map
/utlidar/cloud_base_retimed       PointCloud2   喂 body_filter -> /utlidar/scan
/utlidar/robot_odom_retimed       Odometry      + TF odom -> base_link
/lumen/retimed/{rgb,depth,rgb_info}             喂 RTAB-Map 的 RGB-D 输入
/joint_states                     JointState    12 个腿关节，站立 / 对角 trot
TF base_link -> camera_link                     光学坐标系约定
```

注意：`base_link` 取离地 0.32 m（Go2 站立时的机身中心高度）。把它放成 0 会让地面
回波落进 `pointcloud_to_laserscan` 的 `[-0.15, 1.0]` 高度带，整个地板被当成障碍物，
Nav2 哪儿都规划不出来。
