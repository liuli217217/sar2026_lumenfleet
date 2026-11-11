# 团队拉取与版本说明

发布分支：`mapping-nav-explore-hybrid`。

已有克隆、工作区干净时：

```bash
cd ~/sar2026_lumenfleet
git switch mapping-nav-explore-hybrid
git pull --ff-only origin mapping-nav-explore-hybrid
```

若本地有未提交修改，先自行提交或保存；若分支已分叉，先核对差异，不使用 reset --hard 或强制覆盖。
新克隆：

```bash
git clone --branch mapping-nav-explore-hybrid https://github.com/iuenao/sar2026_lumenfleet.git
```

私有仓库需要自己的 GitHub 访问权限。无需获取原 sar-victim-detector 仓库或初始化 submodule；
独立目录包含重命名的 detector、副本模型、定位、任务协调、测试和构建文件。

## 重要版本区别

- 外层 image/：队友维护的 Nav，本次发布准备已同步到 2b7ffcf，保留其12个新提交。
- sar_detector_nav_integration/image/：复制时 ccefd8a 的独立快照，不包含上述后续12个提交。
- managed_explorer：从旧探索器派生的集成副本，不包含队友后续新增的探索 watchdog/blacklist/backup 修复。
- 不把副本整个覆盖回外层 image/。选择新 Nav 基础镜像前，需要按 README 合并流程验证接口和控制行为。

本次只上传 sar_detector_nav_integration/；原 sar-victim-detector/ 是独立 Git 仓库，未作为嵌套仓库提交。

先读 README.md，再读 RESCUE_WORKFLOW.md 和 TEST_PLAN_2026-11-11.md。
27项本地自动测试及原跟踪测试通过；Docker、ROS、GPU与真机未验证；救援动作仅 mock。

跟踪测试已随副本携带，在主仓库根目录运行：

```bash
python3 sar_detector_nav_integration/image/src/victim_detector_integrated/tools/test_tracker.py \
  sar_detector_nav_integration/image/src/victim_detector_integrated/victim_detector_integrated/detector_node.py
```
