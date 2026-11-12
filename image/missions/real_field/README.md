# real_field：真机有地图任务的先验知识（模板）

`mode:=known` 用到的两个文件（`real_sar.launch.py mission:=/missions/real_field`）：

| 文件 | 内容 |
|------|------|
| `map.png` + `map.yaml` | 只有墙的地图，ROS map_server 格式，0.05 m/像素，黑 = 墙 |
| `victims_estimated.yaml` | 每个伤员的大致位置（世界坐标，m） |

现在的文件复制自仿真训练场地 `disaster_models/missions/arena_5x8`（训练场地是按真实场地建模的）。
**比赛前按实际场地改**：

- **世界坐标系 = Go2 的出发位姿**：原点是出发点（机身中心），x 轴是出发时的朝向，y 轴朝左。
  `map.yaml` 的 `origin` 是地图左下角像素在这个坐标系里的位置。
- 场地布局变了：改 `disaster_models/scripts/make_sar_arena_world.py` 里的 `WALLS` / `START_FIELD` 后运行它，
  再把 `missions/arena_5x8/` 下的这三个文件复制过来；或者直接用图像编辑器改 `map.png`（墙画黑）。
- 伤员估计位置：按组委会给的粗略位置，换算到上面的坐标系（误差 ≤ 0.3 m 时效果最好）。
- 改完后在宿主机上用 `MISSIONS=<这个目录的父目录> run_container.sh` 挂载进容器，不需要重建镜像。
