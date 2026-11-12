FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

SHELL ["/bin/bash", "-c"]

# Build tools, diagnostic dependencies, and recording/playback tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-yaml \
    python3-setuptools \
    ros-jazzy-ament-index-python \
    ros-jazzy-rclpy \
    ros-jazzy-rcl-interfaces \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-ros-py \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-transport \
    ros-jazzy-rosbag2-storage-default-plugins \
    `# the SAR missions:` \
    python3-opencv \
    python3-scipy \
    python3-pip \
    ros-jazzy-vision-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# YOLOv8n for the SAR person detector runs on the CPU through onnxruntime
# (~0.15-0.2 s per frame on the Orin; the detector runs at 4 Hz).
RUN pip3 install --no-cache-dir --break-system-packages onnxruntime==1.20.1

WORKDIR /opt/lumen_ws

COPY src/lumen_sensor_tests/ src/lumen_sensor_tests/
COPY src/unitree_api/ src/unitree_api/
COPY src/unitree_go/ src/unitree_go/
COPY src/go2_cmd_vel_bridge/ src/go2_cmd_vel_bridge/

# Search and rescue: the shared core, the two reactive missions and the bridge
# they drive the robot through. go2_sar_referee is deliberately absent -- it
# reads Gazebo ground truth and has no business on the robot. go2_sar_prior and
# go2_sar_slam belong here too but are not in this branch yet, and a COPY of a
# path that does not exist fails the build.
COPY src/go2_sar_core/ src/go2_sar_core/
COPY src/go2_sar_scout/ src/go2_sar_scout/
COPY src/go2_sar_reflex/ src/go2_sar_reflex/
COPY src/go2_real_bridge/ src/go2_real_bridge/

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --merge-install \
        --packages-select lumen_sensor_tests unitree_api unitree_go go2_cmd_vel_bridge \
            go2_sar_core go2_sar_scout go2_sar_reflex go2_real_bridge \
    && rm -rf build log

# Standalone trial scripts (not a colcon package); run directly with python3,
# e.g. `python3 /validation/move_distance.py --live ...`.
COPY validation/ /validation/

COPY entrypoint.sh /lumen_entrypoint.sh
RUN chmod +x /lumen_entrypoint.sh

ENTRYPOINT ["/lumen_entrypoint.sh"]
CMD ["bash"]
