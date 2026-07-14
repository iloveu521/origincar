#!/usr/bin/env python3
"""Measure /tf transform timestamp age for one frame pair.

Delay = local node clock now - TransformStamped.header.stamp.
Run this on the Nav2/AMCL machine to see whether TF from the base RDK is old
(positive delay) or future-dated (negative delay) relative to scan timestamps.
"""

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_msgs.msg import TFMessage


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TfDelayProbe(Node):
    def __init__(self, parent: str, child: str, window: int):
        super().__init__('tf_delay_probe')
        self.parent = parent.lstrip('/')
        self.child = child.lstrip('/')
        self.window = max(1, window)
        self.delays = []
        self.periods = []
        self.last_stamp = None
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'Measuring /tf age for {self.parent} -> {self.child}. '
            'Delay = local node clock now - transform.header.stamp')

    def on_tf(self, msg: TFMessage):
        now = self.get_clock().now().nanoseconds * 1e-9
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') != self.parent:
                continue
            if t.child_frame_id.lstrip('/') != self.child:
                continue
            stamp = stamp_to_sec(t.header.stamp)
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
            self.get_logger().warn(
                f'No matching TF received on /tf for {self.parent} -> {self.child} yet')
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
            avg_period = statistics.mean(ps)
            hz = 1.0 / avg_period if avg_period > 0 else 0.0
            msg += f' | stamp_period_ms_avg={avg_period*1000:.1f} hz_by_stamp={hz:.2f}'
        self.get_logger().info(msg)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('parent', nargs='?', default='odom_combined')
    parser.add_argument('child', nargs='?', default='base_footprint')
    parser.add_argument('--window', type=int, default=200)
    args = parser.parse_args(argv)

    rclpy.init()
    node = TfDelayProbe(args.parent, args.child, args.window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
