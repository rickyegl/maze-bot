#!/usr/bin/env python3
import math
import time

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from common import config, limit, run


def blocked(bearings, arc):
    lo, hi = arc
    deg = np.degrees(bearings) % 360
    return (deg >= lo) & (deg <= hi) if lo <= hi else (deg >= lo) | (deg <= hi)


class Base(Node):
    def __init__(self):
        super().__init__('base')
        robot = config('robot.json')
        self.drive = robot['drive']
        self.arc = [float(v) for v in self.declare_parameter(
            'blocked_deg', [float(v) for v in robot['sensors']['lidar_blocked_deg']]).value]
        self.lidar_yaw = robot['sensors'].get('lidar_yaw', 0.0)
        self.cmd = Twist()
        self.stamp = 0.0
        self.mask = self.shape = None
        self.drive_pub = self.create_publisher(Twist, 'drive/cmd_vel', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_cmd(self, msg):
        self.cmd = Twist()
        self.cmd.linear.x, self.cmd.angular.z = limit(msg.linear.x, msg.angular.z, self.drive)
        self.stamp = time.monotonic()

    def tick(self):
        if time.monotonic() - self.stamp > self.drive['command_timeout']:
            self.cmd = Twist()
        self.drive_pub.publish(self.cmd)

    def on_scan(self, scan):
        shape = (len(scan.ranges), scan.angle_min, scan.angle_increment)
        if shape != self.shape:
            a = self.lidar_yaw + scan.angle_min + scan.angle_increment * np.arange(len(scan.ranges))
            self.mask, self.shape = blocked(a, self.arc), shape
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        ranges[self.mask] = math.nan
        scan.ranges = ranges.tolist()
        if len(scan.intensities) == len(ranges):
            values = np.asarray(scan.intensities, dtype=np.float32)
            values[self.mask] = 0.0
            scan.intensities = values.tolist()
        self.scan_pub.publish(scan)

    def destroy_node(self):
        self.drive_pub.publish(Twist())
        super().destroy_node()


def main():
    run(Base)


if __name__ == '__main__':
    main()
