#!/bin/bash
# Run the RTAB-Map launch files against the synthetic Go2 feed -- no robot.
#
#   ./run_sim_slam.sh                 # lidar + camera (go2_rtabmap_lidar_camera)
#   ./run_sim_slam.sh lidar           # lidar only     (go2_rtabmap_lidar)
#
# DDS is pinned to loopback so nothing leaks onto the lab network, and the
# launch directory is bind-mounted, so editing a .launch.py needs no rebuild.
set -e

SIM_DIR="$(cd "$(dirname "$0")" && pwd)"
# <repo>/image/validation/sim -> <repo>/image/src/go2_bringup/{launch,config}
LAUNCH_DIR="${LAUNCH_DIR:-$(cd "$SIM_DIR/../../src/go2_bringup/launch" && pwd)}"
CONFIG_DIR="${CONFIG_DIR:-$(cd "$SIM_DIR/../../src/go2_bringup/config" && pwd)}"
IMAGE="${IMAGE:-lumen-jazzy:nav}"
CONTAINER=lumen_sim_slam

case "${1:-both}" in
    lidar) LAUNCH_FILE=go2_rtabmap_lidar.launch.py ;;
    *)     LAUNCH_FILE=go2_rtabmap_lidar_camera.launch.py ;;
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
    -v "$SIM_DIR:/sim:ro" -v "$LAUNCH_DIR:/lx:ro" \
    "$IMAGE" bash -c "python3 /sim/fake_go2_sensors.py & exec ros2 launch /lx/$LAUNCH_FILE"

[ "${DETACH:-1}" = "1" ] && echo "started detached: docker logs -f $CONTAINER | docker stop $CONTAINER"
exit 0
