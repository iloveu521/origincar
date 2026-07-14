#!/usr/bin/env bash
set -euo pipefail

# Run on X5. Example:
#   ./start_fastdds_discovery_server.sh 192.168.3.20 11811
SERVER_IP="${1:-192.168.3.20}"
SERVER_PORT="${2:-11811}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "Starting FastDDS discovery server: ${SERVER_IP}:${SERVER_PORT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec fastdds discovery --server-id 0 --ip-address "${SERVER_IP}" --port "${SERVER_PORT}"
