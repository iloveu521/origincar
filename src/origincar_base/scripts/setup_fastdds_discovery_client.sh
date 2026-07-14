#!/usr/bin/env bash
# Source this in EVERY ROS2 terminal on X5 and PC before launching nodes/RViz.
# Usage:
#   source setup_fastdds_discovery_client.sh 192.168.3.20 11811 42

SERVER_IP="${1:-192.168.3.20}"
SERVER_PORT="${2:-11811}"
DOMAIN_ID="${3:-42}"

export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DISCOVERY_SERVER="${SERVER_IP}:${SERVER_PORT}"

# Keep previous UDP-only FastDDS profile if you already use it to disable SHM.
if [ -f /root/fastdds.xml ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml
  export FASTDDS_DEFAULT_PROFILES_FILE=/root/fastdds.xml
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}"
echo "Tip: run 'ros2 daemon stop' once after changing discovery settings."
