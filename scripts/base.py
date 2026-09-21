#!/usr/bin/env python3
import math
import time

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from common import config, limit, run


class Base(Node):
    def __init__(self):
        super().__init__('base')
        robot = config('robot.json')
        self.drive = robot['drive']
        self.blocked = robot['sensors']['lidar_blocked_deg']
        self.cmd = Twist()
        self.stamp = 0.0
        self.mask = None
        self.drive_pub = self.create_publisher(Twist, 'drive/cmd_vel', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick)

    def on_cmd(self, msg):
        self.cmd = Twist()
        self.cmd.linear.x, self.cmd.angular.z = limit(msg.linear.x, msg.angular.z, self.drive)
        self.stamp = time.monotonic()

    def tick(self):
        if time.monotonic() - self.stamp > self.drive['command_timeout']:
            self.cmd = Twist()
        self.drive_pub.publish(self.cmd)

    def on_scan(self, scan):
        if self.mask is None or len(self.mask) != len(scan.ranges):
            deg = np.degrees(scan.angle_min + scan.angle_increment * np.arange(len(scan.ranges))) % 360
            lo, hi = self.blocked
            self.mask = (deg >= lo) & (deg <= hi)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        ranges[self.mask] = math.nan
        scan.ranges = ranges.tolist()
        scan.intensities = []
        self.scan_pub.publish(scan)


def main():
    run(Base)


if __name__ == '__main__':
    main()
