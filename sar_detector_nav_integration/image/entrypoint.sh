#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/lumen_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Preserve an explicitly supplied CycloneDDS configuration.
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    # Organizers provide SAR_ROBOT_IFACE.
    # Locally, supply ROBOT_IFACE when starting the container.
    robot_iface="${SAR_ROBOT_IFACE:-${ROBOT_IFACE:-}}"

    if [[ -z "$robot_iface" ]]; then
        echo "No robot interface configured." >&2
        echo "Supply SAR_ROBOT_IFACE, ROBOT_IFACE, or CYCLONEDDS_URI." >&2
        exit 1
    fi

    if [[ ! "$robot_iface" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
        echo "Invalid network interface name: $robot_iface" >&2
        exit 1
    fi

    if [[ ! -d "/sys/class/net/$robot_iface" ]]; then
        echo "Network interface not found: $robot_iface" >&2
        echo "Check the interface and use host networking." >&2
        exit 1
    fi

    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${robot_iface}\"/></Interfaces></General></Domain></CycloneDDS>"
fi

exec "$@"
