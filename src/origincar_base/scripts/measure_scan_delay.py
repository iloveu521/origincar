#!/usr/bin/env python3
"""Measure LaserScan header timestamp age without manual/SSH timing error.

It subscribes to a LaserScan topic and prints local ROS time minus
msg.header.stamp. Run it on the machine where AMCL/Nav2 runs. If clocks between
robots are synchronized, this is the end-to-end scan age seen by Nav2.
"""

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ScanDelayProbe(Node):
    def __init__(self, topic: str, window: int, best_effort: bool):
        super().__init__('scan_delay_probe')
        self.topic = topic
        self.window = max(1, window)
        self.delays = []
        self.periods = []
        self.last_stamp = None
        reliability = (QoSReliabilityPolicy.BEST_EFFORT
                       if best_effort else QoSReliabilityPolicy.RELIABLE)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, topic, self.on_scan, qos)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'Measuring {topic} age with subscriber QoS reliability={reliability.name}. '
            'Delay = local node clock now - LaserScan.header.stamp')

    def on_scan(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = stamp_to_sec(msg.header.stamp)
        delay = now - stamp
        if math.isfinite(delay):
            self.delays.append(delay)
            if len(self.delays) > self.window:
                self.delays.pop(0)
        if self.last_stamp is not None:
            period = stamp - self.last_stamp
            if math.isfinite(period) and period > 0:
                self.periods.append(period)
                if len(self.periods) > self.window:
                    self.periods.pop(0)
        self.last_stamp = stamp

    def report(self):
        if not self.delays:
            self.get_logger().warn(f'No scans received on {self.topic} yet')
            return
        ds = list(self.delays)
        ds_sorted = sorted(ds)
        p95 = ds_sorted[int(0.95 * (len(ds_sorted) - 1))]
        msg = (
            f'samples={len(ds)} delay_ms: '
            f'latest={ds[-1]*1000:.1f} avg={statistics.mean(ds)*1000:.1f} '
            f'min={min(ds)*1000:.1f} max={max(ds)*1000:.1f} p95={p95*1000:.1f}'
        )
        if self.periods:
            ps = list(self.periods)
            hz = 1.0 / statistics.mean(ps) if statistics.mean(ps) > 0 else 0.0
            msg += f' | stamp_period_ms_avg={statistics.mean(ps)*1000:.1f} hz_by_stamp={hz:.2f}'
        self.get_logger().info(msg)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/scan')
    parser.add_argument('--window', type=int, default=100)
    parser.add_argument('--best-effort', action='store_true',
                        help='Subscribe with sensor-data style best effort QoS')
    args = parser.parse_args(argv)

    rclpy.init()
    node = ScanDelayProbe(args.topic, args.window, args.best_effort)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
