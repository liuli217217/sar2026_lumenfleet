#!/bin/bash
# Walk the simulated Go2 around the room forever, one Nav2 goal at a time.
# Waypoints keep >=0.6 m off the walls and clear of both pillars.
#   start:  docker exec -d lumen_sim_nav bash /sim/patrol.sh
#   stop:   docker exec lumen_sim_nav pkill -f patrol.sh
source /opt/ros/jazzy/setup.bash
source /opt/lumen_ws/install/setup.bash

WAYPOINTS=("8.5 1.5" "8.5 5.5" "5.0 5.8" "1.5 5.5" "1.5 1.2" "5.0 1.2")

while true; do
    for wp in "${WAYPOINTS[@]}"; do
        read -r X Y <<< "$wp"
        echo "[patrol] -> ($X, $Y)"
        timeout 120 ros2 action send_goal /navigate_to_pose \
            nav2_msgs/action/NavigateToPose \
            "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}}" \
            2>&1 | tail -2
    done
done
