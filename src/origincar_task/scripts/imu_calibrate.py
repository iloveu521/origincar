#!/usr/bin/env python3
"""
IMU Calibration Script
Reads a ros2 bag file (or live topic) of /imu/data_raw, computes:
  - Gyroscope bias (x, y, z) from static data
  - Accelerometer bias (x, y, z) from static data

Usage:
  # From bag file:
  python3 imu_calibrate.py --bag imu_static/ --duration 60

  # From live topic:
  python3 imu_calibrate.py --live --duration 60

Output: calibration values to paste into origincar_base.cpp
"""

import argparse
import math
import sys
from collections import defaultdict

import numpy as np


def calibrate_from_data(records, duration_sec):
    """Compute gyro and accel bias from recorded data points."""
    gyro_x = np.array([r['gx'] for r in records])
    gyro_y = np.array([r['gy'] for r in records])
    gyro_z = np.array([r['gz'] for r in records])
    accel_x = np.array([r['ax'] for r in records])
    accel_y = np.array([r['ay'] for r in records])
    accel_z = np.array([r['az'] for r in records])

    # Gyro bias = mean (should be ~0 when static)
    gyro_bias_x = np.mean(gyro_x)
    gyro_bias_y = np.mean(gyro_y)
    gyro_bias_z = np.mean(gyro_z)

    # Gyro noise = std
    gyro_std_x = np.std(gyro_x)
    gyro_std_y = np.std(gyro_y)
    gyro_std_z = np.std(gyro_z)

    # Accel: expected [0, 0, 9.81] in ENU (gravity pointing down = +Z)
    # Note: depends on sensor orientation. Adjust expected vector if needed.
    accel_bias_x = np.mean(accel_x) - 0.0
    accel_bias_y = np.mean(accel_y) - 0.0
    accel_bias_z = np.mean(accel_z) - 9.81   # gravity down in ENU

    accel_std_x = np.std(accel_x)
    accel_std_y = np.std(accel_y)
    accel_std_z = np.std(accel_z)

    print("=" * 60)
    print("IMU Calibration Results")
    print("=" * 60)
    print(f"Data points: {len(records)}")
    print(f"Duration:   {duration_sec:.1f}s")
    print()
    print("--- Gyroscope Bias (rad/s) ---")
    print(f"  bias_x = {gyro_bias_x:.8f}  (std={gyro_std_x:.8f})")
    print(f"  bias_y = {gyro_bias_y:.8f}  (std={gyro_std_y:.8f})")
    print(f"  bias_z = {gyro_bias_z:.8f}  (std={gyro_std_z:.8f})")
    print()
    print("--- Accelerometer Bias (m/s²) ---")
    print(f"  bias_x = {accel_bias_x:.8f}  (std={accel_std_x:.8f})")
    print(f"  bias_y = {accel_bias_y:.8f}  (std={accel_std_y:.8f})")
    print(f"  bias_z = {accel_bias_z:.8f}  (std={accel_std_z:.8f})")
    print()
    print("--- Code to paste into origincar_base.cpp ---")
    print(f"// Gyro bias compensation (subtract these from raw*GYROSCOPE_RATIO)")
    print(f"static const double GYRO_BIAS_X = {gyro_bias_x:+.8f};")
    print(f"static const double GYRO_BIAS_Y = {gyro_bias_y:+.8f};")
    print(f"static const double GYRO_BIAS_Z = {gyro_bias_z:+.8f};")
    print()
    print(f"// Accel bias compensation (subtract these from raw/ACCEl_RATIO)")
    print(f"static const double ACCEL_BIAS_X = {accel_bias_x:+.8f};")
    print(f"static const double ACCEL_BIAS_Y = {accel_bias_y:+.8f};")
    print(f"static const double ACCEL_BIAS_Z = {accel_bias_z:+.8f};")


def read_from_bag(bag_path):
    """Extract IMU data from a ros2 bag using ros2bag API."""
    try:
        from rosbag2_py import (SequentialReader, StorageOptions,
                                ConverterOptions, StorageFilter)
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
    except ImportError:
        print("ERROR: rosbag2_py not available. Install: "
              "sudo apt install ros-$ROS_DISTRO-rosbag2",
              file=sys.stderr)
        return []

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    # Filter for /imu/data_raw
    topic_filter = '/imu/data_raw'
    reader.set_filter(
        StorageFilter(topics=[topic_filter])
    )

    records = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, Imu)
        records.append({
            't': timestamp * 1e-9,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
        })

    return records


def read_from_csv(csv_path):
    """Read IMU data from CSV (time, gx, gy, gz, ax, ay, az)."""
    import csv
    records = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        # Skip header if present
        first = next(reader, None)
        if first and not first[0].replace('.', '').replace('-', '').isdigit():
            pass  # skip header
        else:
            reader = csv.reader(f)
            if first:
                records.append(_parse_csv_row(first))
        for row in reader:
            if len(row) >= 7:
                records.append(_parse_csv_row(row))
    return records


def _parse_csv_row(row):
    return {
        't': float(row[0]),
        'gx': float(row[1]),
        'gy': float(row[2]),
        'gz': float(row[3]),
        'ax': float(row[4]),
        'ay': float(row[5]),
        'az': float(row[6]),
    }


def main():
    parser = argparse.ArgumentParser(
        description='IMU Calibration Tool for OriginCar')
    parser.add_argument('--bag', type=str,
                        help='Path to ros2 bag directory')
    parser.add_argument('--csv', type=str,
                        help='Path to CSV file (t,gx,gy,gz,ax,ay,az)')
    parser.add_argument('--live', action='store_true',
                        help='Read from live /imu/data_raw topic')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Recording duration in seconds (default: 60)')
    args = parser.parse_args()

    records = []

    if args.bag:
        print(f"Reading from bag: {args.bag}")
        records = read_from_bag(args.bag)
    elif args.csv:
        print(f"Reading from CSV: {args.csv}")
        records = read_from_csv(args.csv)
    elif args.live:
        print("Live mode not yet implemented. Use --bag with a recorded bag.")
        sys.exit(1)
    else:
        parser.print_help()
        sys.exit(1)

    if not records:
        print("No IMU data found!", file=sys.stderr)
        sys.exit(1)

    duration = records[-1]['t'] - records[0]['t']
    calibrate_from_data(records, duration)


if __name__ == '__main__':
    main()
