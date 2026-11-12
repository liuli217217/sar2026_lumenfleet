# DETECTOR_NAV_INTEGRATION 修改清单

原文件均未修改。复制基线：`ccefd8ad18373088a7ca7684fb1eca66a8775038`。

## 原样复制的导航代码

原 image/ 逐文件复制，字节比较通过。未修改 Nav launch、导航参数、hybrid_explorer 和运动桥接。

## 2026-11-12：`go2_bringup` 整包同步到仓库主树当前状态

**做了什么**：把 `image/src/go2_bringup/` 整包从本仓库主树同步过来（31 个文件）。
同步后主树与本副本 `diff -rq` 完全一致。

**为什么**：基线 `ccefd8a` 之后主树的 `go2_bringup` 有十余个提交，其中两类是这个
工作区的运动链路能不能动起来的前提：

| 来源 | 内容 | 不带会怎样 |
|---|---|---|
| 主树（上游） | `collision_debounce` 的三次修复：死区下调、低速指令抬到最小值而非清零、chatter 闭锁封顶 | 真机实测：`/cmd_vel` 恒为零而 `/cmd_vel_raw` 全程有可用指令 |
| 主树（本 PR 前一个提交） | StopZone/SlowZone 改方向感知的 `velocity_polygon` | 贴近障碍进入停止区后再也出不来，12 目标只成功 2 个 |

只摘其中一两个修复叠到旧基线上，会得到一套从未存在、也从未测过的 Nav2 配置
（旧基线的足迹、`inflation_radius`、MPPI 速度上限、`velocity_smoother` 上限都还是老值，
而这些修复的数值正是按新值调的）。所以整包同步，而不是摘樱桃。

**尚未验证**：本工作区**整体**仍未在 Docker/DDS/真实 Nav2 下跑过
（见 `RESCUE_WORKFLOW.md` 开头）。同步只保证运动链路不再自锁，不代表闭环能跑通。

## 2026-11-12：新增两个节点（纯加法，不改状态机）

| 文件 | 作用 |
|---|---|
| `sar_rescue_manager/referee_adapter.py` | 把 rescue_manager 的 ledger 翻译成 `mission_monitor` 计分用的 `/sar/events`（`MISSION_START` / `SIT`）。只订阅，发不了目标、速度或服务调用 |
| `sar_rescue_manager/real_rescue.py` | 真的坐下：`StopMove → Sit → RiseSit`，和两个 reactive 方案同一套姿态变化。`mock_rescue` 只回「完成」，不做任何姿态变化 |
| `sar_rescue_manager/sport_api_shim.py` | 三个 sport 协议常量的本地副本，避免为发三个整数把整个 `go2_real_bridge` 拖进构建 |

`rescue_manager` 的状态机**一行未改**。

## 重命名 detector 副本中的修改

### `image/src/victim_detector_integrated/package.xml`

来源：`sar-victim-detector/package.xml`。

```diff
--- sar-victim-detector/package.xml

+++ image/src/victim_detector_integrated/package.xml

@@ -2,5 +2,5 @@

 <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
 <package format="3">
-  <name>victim_detector</name>
+  <name>victim_detector_integrated</name>
   <version>0.1.0</version>
   <description>Minimal ROS 2 victim detection node for the IEEE SMCS SAR competition</description>
@@ -8,4 +8,9 @@

   <license>MIT</license>
 
+  <!-- DETECTOR_NAV_INTEGRATION: dependencies used by node and launch. -->
+  <exec_depend>std_msgs</exec_depend>
+  <exec_depend>ament_index_python</exec_depend>
+  <exec_depend>launch</exec_depend>
+  <exec_depend>launch_ros</exec_depend>
   <exec_depend>rclpy</exec_depend>
   <exec_depend>sensor_msgs</exec_depend>
```

### `image/src/victim_detector_integrated/setup.py`

来源：`sar-victim-detector/setup.py`。

```diff
--- sar-victim-detector/setup.py

+++ image/src/victim_detector_integrated/setup.py

@@ -3,5 +3,5 @@

 from setuptools import setup
 
-package_name = "victim_detector"
+package_name = "victim_detector_integrated"
 
 setup(
@@ -25,5 +25,5 @@

     entry_points={
         "console_scripts": [
-            "detector_node = victim_detector.detector_node:main",
+            "detector_node = victim_detector_integrated.detector_node:main",
         ],
     },
```

### `image/src/victim_detector_integrated/setup.cfg`

来源：`sar-victim-detector/setup.cfg`。

```diff
--- sar-victim-detector/setup.cfg

+++ image/src/victim_detector_integrated/setup.cfg

@@ -1,4 +1,4 @@

 [develop]
-script_dir=$base/lib/victim_detector
+script_dir=$base/lib/victim_detector_integrated
 [install]
-install_scripts=$base/lib/victim_detector
+install_scripts=$base/lib/victim_detector_integrated
```

### `image/src/victim_detector_integrated/launch/sar_detector.launch.py`

来源：`sar-victim-detector/launch/sar_detector.launch.py`。

```diff
--- sar-victim-detector/launch/sar_detector.launch.py

+++ image/src/victim_detector_integrated/launch/sar_detector.launch.py

@@ -4,16 +4,16 @@

 
 真机 (RealSense D435i)：
-    ros2 launch victim_detector sar_detector.launch.py
+    ros2 launch victim_detector_integrated sar_detector.launch.py
 
 仿真 (Gazebo D455)：
-    ros2 launch victim_detector sar_detector.launch.py image_topic:=/d455/image
+    ros2 launch victim_detector_integrated sar_detector.launch.py image_topic:=/d455/image
 
 搜救推荐配置（分类别阈值 + 时序聚合，压误报同时提有效召回）：
-    ros2 launch victim_detector sar_detector.launch.py \\
+    ros2 launch victim_detector_integrated sar_detector.launch.py \\
         class_conf:="mannequin:0.10,cone:0.20,first_aid:0.20" \\
         track_enable:=true
 
 离线部署换权重：
-    ros2 launch victim_detector sar_detector.launch.py \\
+    ros2 launch victim_detector_integrated sar_detector.launch.py \\
         weights:=/opt/models/sar_3class.pt
 
@@ -34,5 +34,5 @@

 # 不传任何参数就能跑。离线部署想换成容器内的绝对路径时再覆盖。
 _DEFAULT_WEIGHTS = os.path.join(
-    get_package_share_directory("victim_detector"), "models", "sar_3class.pt")
+    get_package_share_directory("victim_detector_integrated"), "models", "sar_3class.pt")
 
 # 名称 -> (默认值, 类型, 说明)
@@ -75,7 +75,7 @@

 
     node = Node(
-        package="victim_detector",
+        package="victim_detector_integrated",
         executable="detector_node",
-        name="sar_detector",
+        name="sar_detector_integrated",
         output="screen",
         emulate_tty=True,
```

### `image/src/victim_detector_integrated/victim_detector_integrated/detector_node.py`

来源：`sar-victim-detector/victim_detector/detector_node.py`。

```diff
--- sar-victim-detector/victim_detector/detector_node.py

+++ image/src/victim_detector_integrated/victim_detector_integrated/detector_node.py

@@ -145,5 +145,6 @@

 
     def __init__(self):
-        super().__init__("sar_detector")
+        # DETECTOR_NAV_INTEGRATION: renamed copy; upstream detector is untouched.
+        super().__init__("sar_detector_integrated")
 
         p = self.declare_parameter
@@ -194,4 +195,8 @@

         self.name_override = _str_list(g("class_names"))
         self.class_conf = _conf_map(g("class_conf"))
+        # DETECTOR_NAV_INTEGRATION: let lower per-class thresholds reach filtering.
+        if not 0 <= self.conf <= 1 or any(not 0 <= v <= 1 for v in self.class_conf.values()):
+            raise ValueError("Confidence thresholds must be in [0, 1]")
+        self.inference_conf = min([self.conf, *self.class_conf.values()])
         self.track_enable = bool(g("track_enable"))
         self.publish_annotated = bool(g("publish_annotated"))
@@ -304,5 +309,5 @@

         t0 = time.perf_counter()
         try:
-            res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf,
+            res = self.model.predict(frame, imgsz=self.imgsz, conf=self.inference_conf,
                                      iou=self.iou, verbose=False,
                                      **self._extra())[0]
@@ -322,5 +327,5 @@

             conf = float(b.conf)
             # 分类别阈值：这一类单独设了就用它的，否则沿用全局 conf
-            thr = self.class_conf.get(self._name_of(cid))
+            thr = self.class_conf.get(self._name_of(cid), self.conf)
             if thr is not None and conf < thr:
                 continue
```

## 全新文件

- `image/Dockerfile.detector`：增量构建与独立推理 venv。
- `image/src/victim_detector_integrated/requirements-integration.txt`：模型依赖约束。
- `image/src/sar_target_mapping/`：全新 ROS 包；core.py 提供投影/坐标变换/关联，node.py 提供缓存/TF/记录/输出，launch/ 提供组合启动，test/ 提供测试。
- `README.md`：输入输出、集成方法、运行和合并说明。
- `SOURCE_BASELINE.json`：源文件指纹与 commit。
- `COLCON_IGNORE`：防止外层重复发现 ROS 包。

Python 模块目录和 ament resource 已从 victim_detector 重命名为 victim_detector_integrated。
原 detector 文档保留为历史材料；请以本目录 README.md 为集成运行说明。


## 任务闭环扩展（本次执行）

原 image/ 导航内容仍未修改。新增包 `image/src/sar_rescue_manager/`：

| 文件 | 来源 / 修改与集成位置 |
|---|---|
| sar_rescue_manager/core.py | 新增：持久目标 Registry、任务状态、空间关联、接近点和增量 costmap 更新 |
| sar_rescue_manager/action_slot.py | 新增：异步 action 所有权，pending acceptance 时取消，等待最终结果 |
| sar_rescue_manager/node.py | 新增：唯一 Nav2 客户端，任务状态机、停止确认、mock 协议、状态发布 |
| sar_rescue_manager/managed_explorer.py | 复制自 image/src/go2_bringup/scripts/hybrid_explorer.py 并重命名；删除 action 客户端，改为发布探索候选点；新增协调器心跳抑制和局部代价地图增量更新；网格查询支持 origin 旋转和负坐标 |
| sar_rescue_manager/mock_rescue.py | 新增：带 token 的模拟动作请求/结果，不发送机器人 API |
| launch/rescue_mission.launch.py | 新增：组合启动 detector、mapper、managed explorer、manager、mock；Nav/建图仍单独启动 |
| test/test_mission.py | 新增：纯逻辑、真实 callback + fake action transport 测试；不等同 ROS 集成测试 |
| package.xml、setup.py、setup.cfg、resource/ | 新增：包依赖、可执行入口、启动文件安装 |

已有集成文件的增量修改：

- `image/Dockerfile.detector`：COPY 并编译 sar_rescue_manager；基础 Nav Dockerfile 不动。
- `README.md`：区分观测模式与任务闭环模式，补充合并清单。
- `RESCUE_WORKFLOW.md`：新增完整状态、input/output、构建与启停、验证和限制说明。

全部新源码以 `DETECTOR_NAV_INTEGRATION` 标明。managed_explorer 文件头包含原文件来源；
关键控制交接代码另有标记。源 baseline 中的原始导航文件及 detector 原件均保持不变。

## 真机测试准备

新增 `TEST_PLAN_2026-11-11.md`：逐阶段测试、前置条件、启停命令、通过/停止条件、rosbag和结果表。README 增加入口；本次未修改运行代码。

## GitHub发布准备

新增 SHARING.md，说明团队拉取方法及快照版本差异；更新README和测试计划，跟踪测试命令改为副本内路径，不依赖未上传的原detector仓库。外层Nav快进到2b7ffcf，保留队友12个新提交；集成副本没有同步改写。
