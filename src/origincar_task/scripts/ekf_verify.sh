#!/bin/bash
# Minimal EKF verification launch — no URDF, no joint_state_publisher
# Save memory by avoiding the heavy robot_description (342MB)
#
# Usage:
#   chmod +x ekf_verify.sh && ./ekf_verify.sh

set -e

echo "=== Starting minimal bringup for EKF verification ==="

# 1. Static TF frames (required for EKF)
echo "[1/4] Publishing static transforms..."
ros2 run tf2_ros static_transform_publisher 0.092 0 0 0 0 0 base_footprint base_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint gyro_link &
ros2 run tf2_ros static_transform_publisher 0.083 0 0.102 0 0 0 base_link laser &
sleep 1

# 2. IMU filter: /imu/data_raw → /imu/data (Madgwick orientation estimation)
echo "[2/4] Starting IMU filter..."
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
    -p fixed_frame:=base_footprint \
    -p use_mag:=false \
    -p publish_tf:=false \
    -p world_frame:=enu \
    -p orientation_stddev:=0.05 &
sleep 1

# 3. EKF: fuse /odom + /imu/data → /odom_combined
echo "[3/4] Starting EKF node..."
ros2 run robot_localization ekf_node --ros-args \
    -r odometry/filtered:=odom_combined \
    -p frequency:=30.0 \
    -p sensor_timeout:=0.1 \
    -p two_d_mode:=false \
    -p map_frame:=map \
    -p odom_frame:=odom \
    -p base_link_frame:=base_footprint \
    -p world_frame:=odom \
    -p publish_tf:=false \
    -p imu0:=/imu/data \
    -p imu0_config:="[false,false,false,true,true,true,true,true,true,false,false,false,true,true,true]" \
    -p imu0_remove_gravitational_acceleration:=true \
    -p odom0:=/odom \
    -p odom0_config:="[true,true,false,false,false,true,false,false,false,false,false,false,false,false,false]" &
sleep 1

echo "[4/4] Starting origincar_base (serial driver)..."
echo "    This publishes /odom + /imu/data_raw"
ros2 run origincar_base origincar_base_node &

echo ""
echo "=== All nodes started ==="
echo "Verify:  ros2 topic echo /odom_combined --once"
echo "Monitor: ros2 topic hz /odom_combined"
echo "TF tree: ros2 run tf2_tools view_frames"
echo ""
echo "Press Ctrl+C to stop all."
wait
