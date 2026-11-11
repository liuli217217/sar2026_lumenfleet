"""启动 SAR 检测节点。

默认就用包里自带的三类模型（cone / first_aid / mannequin），直接跑：

真机 (RealSense D435i)：
    ros2 launch victim_detector_integrated sar_detector.launch.py

仿真 (Gazebo D455)：
    ros2 launch victim_detector_integrated sar_detector.launch.py image_topic:=/d455/image

搜救推荐配置（分类别阈值 + 时序聚合，压误报同时提有效召回）：
    ros2 launch victim_detector_integrated sar_detector.launch.py \\
        class_conf:="mannequin:0.10,cone:0.20,first_aid:0.20" \\
        track_enable:=true

离线部署换权重：
    ros2 launch victim_detector_integrated sar_detector.launch.py \\
        weights:=/opt/models/sar_3class.pt

注意：命令行传空字符串要写成 keep_classes:="''"，
直接写 keep_classes:="" 会被 shell 吃掉，launch 报 malformed argument。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 包里自带的三类权重。装好之后这个路径一定存在，所以默认指向它，
# 不传任何参数就能跑。离线部署想换成容器内的绝对路径时再覆盖。
_DEFAULT_WEIGHTS = os.path.join(
    get_package_share_directory("victim_detector_integrated"), "models", "sar_3class.pt")

# 名称 -> (默认值, 类型, 说明)
# 类型必须显式写出：launch 传进来的都是字符串，
# 不声明 value_type 的话 int/float/bool 参数会类型不匹配而报错。
ARGS = {
    "image_topic":       ("/camera/camera/color/image_raw", str,   "输入图像话题"),
    "best_effort_qos":   ("true",       bool,  "realsense 必须 true；Gazebo 桥接用 false"),
    "queue_depth":       ("1",          int,   "1 = 只处理最新帧"),
    "weights":           (_DEFAULT_WEIGHTS, str, "权重路径；默认用包里自带的三类模型"),
    "device":            ("auto",       str,   "auto | cpu | cuda:0"),
    "imgsz":             ("640",        int,   "推理分辨率"),
    "conf":              ("0.25",       float, "置信度阈值"),
    "iou":               ("0.45",       float, "NMS IoU"),
    "half":              ("false",      bool,  "FP16；Jetson 上建议 true"),
    # 默认空 = 三类全要。注意：命令行想显式传空必须写 keep_classes:="''" ——
    # 直接写 keep_classes:="" 会被 shell 吃成 keep_classes:= 而报格式错误。
    "keep_classes":      ("",           str,   "保留的类 id，逗号分隔；空 = 全部"),
    "class_names":       ("",           str,   "覆盖类名，逗号分隔；留空用模型自带"),
    "class_conf":        ("",           str,   "分类别阈值，如 mannequin:0.10,cone:0.20；空 = 都用全局 conf"),
    "track_enable":      ("false",      bool,  "时序聚合：连续多帧命中才确认"),
    "track_iou":         ("0.3",        float, "帧间关联的 IoU 阈值"),
    "track_min_hits":    ("3",          int,   "连续命中几帧才确认"),
    "track_max_misses":  ("5",          int,   "丢几帧后丢弃该目标"),
    "track_bridge":      ("false",      bool,  "丢帧期间继续上报（提召回，框会滞后）"),
    "publish_annotated": ("true",       bool,  "是否发布标注图（上线建议关掉省带宽）"),
    "max_rate_hz":       ("0.0",        float, "推理限速；Orin 50W 建议 5~10"),
    "log_every_n":       ("30",         int,   "每 N 帧打一行日志"),
}


def generate_launch_description():
    decls = [DeclareLaunchArgument(name, default_value=default, description=doc)
             for name, (default, _t, doc) in ARGS.items()]

    params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=typ)
        for name, (_d, typ, _doc) in ARGS.items()
    }

    node = Node(
        package="victim_detector_integrated",
        executable="detector_node",
        name="sar_detector_integrated",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )
    return LaunchDescription(decls + [node])
