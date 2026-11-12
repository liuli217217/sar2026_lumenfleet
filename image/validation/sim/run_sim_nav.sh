#!/bin/bash
# Nav2 + RTAB-Map SLAM + RViz against the synthetic Go2 feed -- no robot.
#
#   ./run_sim_nav.sh              # SLAM = lidar + camera
#   ./run_sim_nav.sh lidar        # SLAM = lidar only
#   DRIVE=circle ./run_sim_nav.sh # ignore Nav2, just walk a fixed circle
#
# Pipeline inside the container:
#   fake_go2_sensors.py  -> cloud / cloud_base / odom+TF / rgb-d / joint_states
#   go2_description robot.launch.py  -> robot_state_publisher, so the Go2 model
#                                       itself shows up and walks in RViz
#   go2_rtabmap_*.launch.py          -> /map  + rtabmap_viz GUI
#   go2_navigation_rtabmap.launch.py -> body_filter, pointcloud_to_laserscan,
#                                       Nav2, collision_monitor, RViz
#   RViz "2D Nav Goal" -> Nav2 -> /cmd_vel -> the simulated robot drives.
#
# The host launch/ directory is mounted over the installed one, so launch-file
# edits take effect on the next run with no colcon build / image rebuild.
set -e

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
# <repo>/image/validation/sim -> <repo>/image/src/go2_bringup/{launch,config}
LAUNCH_DIR="${LAUNCH_DIR:-$(cd "$SIM_DIR/../../src/go2_bringup/launch" && pwd)}"
CONFIG_DIR="${CONFIG_DIR:-$(cd "$SIM_DIR/../../src/go2_bringup/config" && pwd)}"
IMAGE="${IMAGE:-lumen-jazzy:nav}"
CONTAINER=lumen_sim_nav

case "${1:-both}" in
    lidar) SLAM_FILE=go2_rtabmap_lidar.launch.py ;;
    *)     SLAM_FILE=go2_rtabmap_lidar_camera.launch.py ;;
esac

LOOPBACK_DDS='<CycloneDDS><Domain>
  <General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces>
  <AllowMulticast>false</AllowMulticast></General>
  <Discovery><Peers><Peer address="localhost"/></Peers></Discovery>
</Domain></CycloneDDS>'

# Detached by default: an attached `docker run` forwards SIGTERM into the
# container when its own shell goes away, which tears the stack down with a
# SIGKILL 10 s later. Set DETACH=0 to stream the logs instead.
[ "${DETACH:-1}" = "1" ] && DETACH_FLAG=-d || DETACH_FLAG=-i

# RViz and rtabmap_viz need a real GL context. Without the GPU device they get
# no DRM access, every display fails to initialise and the window comes up
# empty -- the MESA/"failed to load driver: iris" lines in the log are that,
# not a cosmetic warning. /dev/dri is ACL'd to the logged-in user, which is the
# uid this container runs as. No GPU -> fall back to software rendering.
if [ -e /dev/dri ]; then
    GPU_FLAGS=(--device /dev/dri)
else
    GPU_FLAGS=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi
docker run --rm $DETACH_FLAG --name "$CONTAINER" \
    --user "$(id -u):$(id -g)" -e HOME=/tmp "${GPU_FLAGS[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e CYCLONEDDS_URI="$LOOPBACK_DDS" -e ROS_DOMAIN_ID=78 \
    -e DRIVE="${DRIVE:-cmd_vel}" \
    -v "$SIM_DIR:/sim:ro" \
    -v "$LAUNCH_DIR:/opt/lumen_ws/install/share/go2_bringup/launch:ro" \
    -v "$CONFIG_DIR:/opt/lumen_ws/install/share/go2_bringup/config:ro" \
    "$IMAGE" bash -c "
        python3 /sim/fake_go2_sensors.py --ros-args -p drive:=\${DRIVE:-cmd_vel} &
        ros2 launch go2_description robot.launch.py use_sim_time:=False &
        ros2 launch go2_bringup $SLAM_FILE &
        sleep 6
        exec ros2 launch go2_bringup go2_navigation_rtabmap.launch.py use_rviz:=true
    "

[ "${DETACH:-1}" = "1" ] && cat <<MSG
started detached. follow it with:
    docker logs -f $CONTAINER
patrol the room:
    docker exec -d $CONTAINER bash -c 'bash /sim/patrol.sh > /tmp/patrol.log 2>&1'
stop everything:
    docker stop $CONTAINER
MSG
exit 0
