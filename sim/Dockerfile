# Gazebo simulation image for the SAR missions -- ROS 2 Jazzy + Gazebo Harmonic.
#
# Why Jazzy and not Humble (the version this sim was developed on): the robot
# models and the ros2_control plugin target the Harmonic generation
# (`gz-sim`, `gz_ros2_control/GazeboSimSystem`). Humble's own ros_gz /
# gz_ros2_control debs are built against Fortress, so on Humble those two have
# to be compiled from source. Jazzy pairs with Harmonic out of the box -- and
# it is what the real robot image built from image/Dockerfile runs, so the
# simulation and the robot are on the same ROS distribution.
#
# Built by build.sh, which assembles the context from this one repo:
#   sim/src/     Gazebo side: champ*, unitree_go2_*, disaster_models
#   image/src/   the SAR mission packages (go2_sar_*, go2_real_bridge) --
#                the same files the real robot runs
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
# Acquire::Retries: this network drops archive.ubuntu.com connections mid-build.
RUN apt-get -o Acquire::Retries=5 update && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-robot-localization \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-vision-msgs \
        ros-jazzy-rviz2 \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-numpy python3-scipy python3-opencv python3-yaml python3-pip \
        procps \
    && rm -rf /var/lib/apt/lists/*

# Same inference stack as the robot image: YOLOv8n through onnxruntime on CPU.
RUN pip3 install --no-cache-dir --break-system-packages onnxruntime==1.20.1

WORKDIR /ws
COPY src /ws/src
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --event-handlers console_direct+ \
    && rm -rf build log

COPY entrypoint.sh /sim_entrypoint.sh
RUN chmod +x /sim_entrypoint.sh
ENTRYPOINT ["/sim_entrypoint.sh"]
CMD ["bash"]
