#!/usr/bin/env python3
"""
Odom Self-Drive Calibration Script for OriginCar
Two modes:
  --mode linear  : Drive straight forward, calibrate X/Y scale
  --mode angular : Rotate in place, calibrate angular Z scale

Usage:
  # Linear: drive ~1m forward
  python3 odom_calibrate.py --mode linear --distance 1.0

  # Angular: rotate ~360deg in place clockwise
  python3 odom_calibrate.py --mode angular --angle 360
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class OdomCalibrator:
    def __init__(self, node):
        self.node = node
        self.latest_odom = None
        self.lock = threading.Lock()
        self.sub = node.create_subscription(
            Odometry, '/odom', self.callback, 10)
        self.cmd_pub = node.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10)

    def callback(self, msg: Odometry):
        with self.lock:
            self.latest_odom = msg

    def get_pose(self):
        with self.lock:
            if self.latest_odom is None:
                return None
            m = self.latest_odom
            x = m.pose.pose.position.x
            y = m.pose.pose.position.y
            qx = m.pose.pose.orientation.x
            qy = m.pose.pose.orientation.y
            qz = m.pose.pose.orientation.z
            qw = m.pose.pose.orientation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            return (x, y, yaw)

    def drive(self, linear_x=0.0, angular_z=0.0):
        """Publish Ackermann control command.
        angular_z positive = steering left (CCW), negative = right (CW)."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.speed = linear_x
        cmd.drive.steering_angle = angular_z
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.drive(0.0, 0.0)

    def wait_for_odom(self, timeout=3.0):
        for _ in range(int(timeout * 10)):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.get_pose() is not None:
                return True
        return False

    def spin_for(self, seconds):
        for _ in range(int(seconds * 10)):
            rclpy.spin_once(self.node, timeout_sec=0.1)


def do_linear(args, cal):
    """Linear calibration: drive straight, compare distances."""
    # Record start
    cal.spin_for(0.5)
    start = cal.get_pose()
    print(f"\nStart: x={start[0]:.4f}, y={start[1]:.4f}, yaw={math.degrees(start[2]):.1f}°")

    # Drive forward (speed in m/s, steering=0 → straight)
    duration = args.distance / args.speed
    print(f"\n>>> Driving forward at {args.speed} m/s for {duration:.1f}s <<<")
    cal.drive(linear_x=args.speed, angular_z=0.0)

    for t in range(int(duration), 0, -1):
        print(f"    {t}...", flush=True)
        time.sleep(1.0)
    frac = duration - int(duration)
    if frac > 0:
        time.sleep(frac)

    cal.stop()
    print("    Stopped.")
    time.sleep(0.5)

    # Record end
    cal.spin_for(0.5)
    end = cal.get_pose()
    print(f"End:   x={end[0]:.4f}, y={end[1]:.4f}, yaw={math.degrees(end[2]):.1f}°")

    # Compute
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    odom_dist = math.sqrt(dx * dx + dy * dy)
    yaw_drift = math.degrees(abs(end[2] - start[2]))

    print(f"\nOdom dx={dx:.4f}, dy={dy:.4f}, dist={odom_dist:.4f} m, yaw_drift={yaw_drift:.2f}°")

    # Ask user
    print("\n" + "-" * 40)
    try:
        actual = float(input(f"Actual distance in meters [default={args.distance}]: ")
                        or args.distance)
    except (EOFError, ValueError):
        actual = args.distance

    correction = actual / odom_dist
    error_pct = (odom_dist - actual) / actual * 100

    print("\n" + "=" * 60)
    print("Linear Calibration Result")
    print("=" * 60)
    print(f"Actual: {actual:.4f} m  |  Odom: {odom_dist:.4f} m  |  Error: {error_pct:+.2f}%")
    print(f"Correction: {correction:.6f}")
    print()
    print(f"static const double ODOM_LINEAR_SCALE_X = {0.944997 * correction:.6f};")
    print(f"static const double ODOM_LINEAR_SCALE_Y = {1.032157 * correction:.6f};")


def do_angular(args, cal):
    """Angular calibration: drive in a circle with fixed steering, count rotations.

    For Ackermann: angular velocity = speed / wheelbase * tan(steering_angle)
    The robot drives in a circle. User marks when it completes 1 full rotation."""
    # Record start
    cal.spin_for(0.5)
    start = cal.get_pose()
    print(f"\nStart: x={start[0]:.4f}, y={start[1]:.4f}, yaw={math.degrees(start[2]):.1f}°")

    # For circle driving: speed + non-zero steering angle
    # steering_angle in rad (positive = left turn, counter-clockwise)
    # direction: cw=right turn(negative steering), ccw=left turn(positive steering)
    steer_dir = -1.0 if args.direction == 'cw' else 1.0
    steer_rad = math.radians(args.steer_deg)

    # Estimate duration for 1 full circle
    # Assume typical Ackermann: ω ≈ v * tan(steer) / wheelbase
    # With steer=20°, speed=0.3, wheelbase~0.3m: ω ≈ 0.36 rad/s → ~17s per circle
    # Add 50% margin: the user watches and stops manually
    est_omega = args.speed * math.tan(steer_rad) / 0.3  # rough estimate
    est_omega = max(est_omega, 0.1)
    duration = math.radians(args.angle) / est_omega * 1.5

    print(f"\n>>> Driving in circle: speed={args.speed} m/s, "
          f"steer={args.steer_deg}° {'right(CW)' if steer_dir < 0 else 'left(CCW)'} <<<")
    print(f"    Estimated {duration:.0f}s for {args.angle}° rotation")
    print("    Robot will drive in a circle. Watch the heading!")
    print()
    print("    Press Ctrl+C when robot completes the rotation!")

    cal.drive(linear_x=args.speed, angular_z=steer_dir * steer_rad)

    try:
        # Drive until user interrupts or timeout
        start_time = time.time()
        while time.time() - start_time < duration * 2:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    cal.stop()
    print("    Stopped.")
    time.sleep(0.5)

    # Record end
    cal.spin_for(0.5)
    end = cal.get_pose()
    print(f"End:   x={end[0]:.4f}, y={end[1]:.4f}, yaw={math.degrees(end[2]):.1f}°")

    # Compute yaw change (handle wrap-around)
    odom_yaw_change = math.degrees(end[2] - start[2])
    while odom_yaw_change > 180:
        odom_yaw_change -= 360
    while odom_yaw_change < -180:
        odom_yaw_change += 360
    odom_abs = abs(odom_yaw_change)

    actual_deg = args.angle
    correction = actual_deg / odom_abs if odom_abs > 1.0 else 1.0
    error_pct = (odom_abs - actual_deg) / actual_deg * 100

    print(f"\nPosition drift: dx={end[0]-start[0]:.4f}, dy={end[1]-start[1]:.4f} m")
    print(f"Odom yaw change: {odom_abs:.2f}°  (raw: {odom_yaw_change:.2f}°)")

    print("\n" + "=" * 60)
    print("Angular Calibration Result")
    print("=" * 60)
    print(f"Actual: {actual_deg:.1f}°  |  Odom: {odom_abs:.2f}°  |  Error: {error_pct:+.2f}%")
    print(f"Correction: {correction:.6f}")
    print()
    print(f"static const double ODOM_ANGULAR_SCALE = {1.0 * correction:.6f};")


def main():
    parser = argparse.ArgumentParser(
        description='Odom Self-Drive Calibration for OriginCar')
    parser.add_argument('--mode', choices=['linear', 'angular'], default='linear',
                        help='Calibration mode (default: linear)')

    # Linear params
    parser.add_argument('--distance', type=float, default=1.0,
                        help='Linear: target distance in meters (default: 1.0)')
    parser.add_argument('--speed', type=float, default=0.2,
                        help='Linear: forward speed in m/s (default: 0.2)')

    # Angular params (Ackermann circle driving)
    parser.add_argument('--angle', type=float, default=360.0,
                        help='Angular: target rotation in degrees (default: 360)')
    parser.add_argument('--steer-deg', type=float, default=20.0,
                        help='Angular: steering angle in degrees (default: 20)')
    parser.add_argument('--direction', choices=['cw', 'ccw'], default='cw',
                        help='Angular: rotation direction (default: cw)')

    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('odom_calibrator')
    cal = OdomCalibrator(node)

    print("Waiting for /odom data...")
    if not cal.wait_for_odom():
        print("ERROR: No /odom data! Is origincar_base running?", file=sys.stderr)
        sys.exit(1)

    if args.mode == 'linear':
        do_linear(args, cal)
    else:
        do_angular(args, cal)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
