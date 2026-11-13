FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

SHELL ["/bin/bash", "-c"]

# Build tools, diagnostic dependencies, and recording/playback tools.
# Check-Date=false tolerates a host clock running behind the archive's Release
# timestamps; drop it once the host clock is NTP-synced.
RUN apt-get update -o Acquire::Check-Date=false -o Acquire::Retries=5 && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-yaml \
    python3-setuptools \
    python3-opencv \
    nlohmann-json3-dev \
    ros-jazzy-ament-index-python \
    ros-jazzy-rclpy \
    ros-jazzy-rcl-interfaces \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-ros-py \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-transport \
    ros-jazzy-rosbag2-storage-default-plugins \
    ros-jazzy-map-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-cv-bridge \
    ros-jazzy-slam-toolbox \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-navigation2 \
    ros-jazzy-rviz2 \
    ros-jazzy-rtabmap-slam \
    ros-jazzy-rtabmap-viz \
    ros-jazzy-vision-msgs \
    ros-jazzy-realsense2-camera \
    python3-scipy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# YOLOv8n for the SAR person detector runs on the CPU through onnxruntime
# (~0.15-0.2 s per frame on the Orin; the detector runs at 4 Hz).
RUN pip3 install --no-cache-dir --break-system-packages onnxruntime==1.20.1

WORKDIR /opt/lumen_ws

# Message/interface packages first (build order doesn't strictly matter --
# colcon topologically sorts -- but grouped here for readability).
COPY src/unitree_api/ src/unitree_api/
COPY src/unitree_go/ src/unitree_go/
COPY src/go2_interfaces/ src/go2_interfaces/
COPY src/explore_lite_msgs/ src/explore_lite_msgs/

# Sensor/bridge/driver packages.
COPY src/lumen_sensor_tests/ src/lumen_sensor_tests/
COPY src/go2_cmd_vel_bridge/ src/go2_cmd_vel_bridge/
COPY src/go2_description/ src/go2_description/
COPY src/go2_driver/ src/go2_driver/
COPY src/go2_camera_bridge/ src/go2_camera_bridge/
COPY src/go2_rviz/ src/go2_rviz/

# Mapping / navigation / exploration.
COPY src/go2_bringup/ src/go2_bringup/
COPY src/explore_lite/ src/explore_lite/

# Search and rescue missions (merged from SMC_SAR_Phase2): the known-map
# navigator, the no-map explorer, the reactive explorer, the YOLO person
# detector, and the Go2 sport-API bridge they drive the robot through.
COPY src/go2_sar_core/ src/go2_sar_core/
COPY src/go2_sar_scout/ src/go2_sar_scout/
COPY src/go2_sar_reflex/ src/go2_sar_reflex/
COPY src/go2_sar_prior/ src/go2_sar_prior/
COPY src/go2_sar_slam/ src/go2_sar_slam/
# go2_sar_referee is deliberately absent: it reads Gazebo ground truth
# and has no business on the robot.
COPY src/go2_real_bridge/ src/go2_real_bridge/

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --merge-install \
        --packages-select \
            unitree_api unitree_go go2_interfaces explore_lite_msgs \
            lumen_sensor_tests go2_cmd_vel_bridge \
            go2_description go2_driver go2_camera_bridge go2_rviz \
            go2_bringup explore_lite \
            go2_sar_core go2_sar_scout go2_sar_reflex go2_sar_prior go2_sar_slam go2_real_bridge \
    && rm -rf build log

# Known-map mission data (prior wall map + estimated victim positions). The SAR
# launch files look for it at /missions/<field>, same path as in the
# SMC_SAR_Phase2 image it came from.
COPY missions/ /missions/

# Standalone trial scripts (not a colcon package); run directly with python3,
# e.g. `python3 /validation/move_distance.py --live ...`.
COPY validation/ /validation/

COPY entrypoint.sh /lumen_entrypoint.sh
RUN chmod +x /lumen_entrypoint.sh

ENTRYPOINT ["/lumen_entrypoint.sh"]
CMD ["bash"]
