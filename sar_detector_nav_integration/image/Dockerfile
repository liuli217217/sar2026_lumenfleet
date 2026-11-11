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
    && rm -rf /var/lib/apt/lists/*

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

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --merge-install \
        --packages-select \
            unitree_api unitree_go go2_interfaces explore_lite_msgs \
            lumen_sensor_tests go2_cmd_vel_bridge \
            go2_description go2_driver go2_camera_bridge go2_rviz \
            go2_bringup explore_lite \
    && rm -rf build log

# Standalone trial scripts (not a colcon package); run directly with python3,
# e.g. `python3 /validation/move_distance.py --live ...`.
COPY validation/ /validation/

COPY entrypoint.sh /lumen_entrypoint.sh
RUN chmod +x /lumen_entrypoint.sh

ENTRYPOINT ["/lumen_entrypoint.sh"]
CMD ["bash"]
