#!/usr/bin/env bash
# Build the Gazebo simulation image, smc-sar-sim:jazzy.
#
# This used to live in SMC_SAR_Phase2/SimDocker/build.sh and assemble its
# context from two repositories. The Gazebo side now lives here in sim/src,
# so the build has a single source tree and no sibling-checkout assumption:
#
#   sim/src/        Gazebo side: champ*, unitree_go2_*, disaster_models
#   image/src/      the SAR mission packages themselves (go2_sar_*,
#                   go2_real_bridge), shared with the robot image
#
# The mission packages are NOT duplicated here. They are the same files the
# real robot runs, which is the point: real_sar.launch.py with sim_test:=true
# drives Gazebo through the same bridge, the same topic names and the same
# parameters as the robot, so a result here means something about the robot.
#
#   ./build.sh
#   IMAGE=smc-sar-sim:test ./build.sh      # build under another tag
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
IMAGE="${IMAGE:-smc-sar-sim:jazzy}"

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

copy() { rsync -a --exclude __pycache__ --exclude '*.pyc' --exclude build \
             --exclude install --exclude log "$1/" "$2/"; }

mkdir -p "$STAGE/src"

# Gazebo side.
for p in champ champ_base champ_msgs unitree_go2_description unitree_go2_sim disaster_models; do
    copy "$HERE/src/$p" "$STAGE/src/$p"
done

# Meshes and textures are not in this repository -- see sim/src/ASSETS.md.
# They are ~100 MB of unmodified Gazebo Fuel assets; keeping them out of git
# keeps the history from carrying 100 MB of binaries forever.
ASSETS="${GAZEBO_ASSETS:-$REPO/../SMC_SAR_Phase2}"
missing=0
[ -d "$ASSETS/disaster_models/models" ] || missing=1
[ -d "$ASSETS/unitree_go2_ros2/unitree_go2_description/meshes" ] || missing=1
if [ "$missing" = 1 ]; then
    cat >&2 <<MSG
error: Gazebo meshes and textures not found under
         $ASSETS
       Expected:
         \$ASSETS/disaster_models/models/
         \$ASSETS/unitree_go2_ros2/unitree_go2_description/meshes/
       They are not kept in this repository (see sim/src/ASSETS.md). Point
       GAZEBO_ASSETS at a checkout of SMC_SAR_Phase2:
         GAZEBO_ASSETS=/path/to/SMC_SAR_Phase2 ./build.sh
MSG
    exit 1
fi
copy "$ASSETS/disaster_models/models" "$STAGE/src/disaster_models/models"
copy "$ASSETS/unitree_go2_ros2/unitree_go2_description/meshes" \
     "$STAGE/src/unitree_go2_description/meshes"

# Mission side, from the robot workspace -- single source of truth.
# unitree_api / unitree_go are what go2_bridge speaks.
# go2_sar_reflex, go2_sar_prior and go2_sar_slam join this list when they land
# in this branch, which carries Scout (A1) only.
for p in go2_sar_core go2_sar_referee go2_sar_scout \
         go2_real_bridge unitree_api unitree_go; do
    copy "$REPO/image/src/$p" "$STAGE/src/$p"
done

cp "$HERE/Dockerfile" "$HERE/entrypoint.sh" "$STAGE/"

echo "building $IMAGE from $STAGE"
# --network=host is required, not cosmetic: in the default bridge network this
# machine's containers cannot resolve archive.ubuntu.com or packages.ros.org,
# and each apt-get takes ~390 s to time out before failing the whole RUN with
# exit 100. Sharing the host network stack inherits its resolver and the build
# proceeds normally.
docker build --network=host -t "$IMAGE" "$STAGE"

cat <<MSG

built $IMAGE. Run a mission headless:

  docker run --rm --device /dev/dri $IMAGE \\
    ros2 launch go2_real_bridge real_sar.launch.py \\
        mode:=scout sim_test:=true enable_motion:=true gui:=false camera:=none debug:=true

Add -e LIBGL_ALWAYS_SOFTWARE=1 instead
of --device /dev/dri on a machine with no GPU. The referee prints
"SCORE n/4 victims found within T s" at the end.
MSG
