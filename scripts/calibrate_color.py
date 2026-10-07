#!/usr/bin/env python3
import argparse
import json
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger

from color_sensor import rgb_hex, track_names
from common import config_path


def main():
    path = config_path('colors.json')
    colors = json.loads(path.read_text())
    names = track_names(colors, 'all')
    parser = argparse.ArgumentParser(description='park the colour sensor on a tile and record what it reads')
    parser.add_argument('name', nargs='?', choices=names)
    parser.add_argument('--profile', default='robot')
    parser.add_argument('--samples', type=int, default=20)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not args.name:
        for profile, readings in colors['calibration'].items():
            print(f'{profile}:')
            for n in names:
                print(f'  {n:12} {readings.get(n, "-")}')
        return

    rclpy.init()
    node = Node('calibrate_color')
    samples = []
    node.create_subscription(ColorRGBA, 'color/rgb', lambda m: samples.append((m.r, m.g, m.b)), 10)
    while rclpy.ok() and len(samples) < args.samples:
        rclpy.spin_once(node, timeout_sec=2.0)
        if not samples:
            node.get_logger().warn('nothing on /color/rgb, is color_sensor running?', throttle_duration_sec=5.0)
    rgb = np.array(samples) * 255
    spread = float(np.linalg.norm(rgb - rgb.mean(0), axis=1).max())
    colors = json.loads(path.read_text())
    colors['calibration'].setdefault(args.profile, {})[args.name] = rgb_hex(rgb.mean(0))
    path.write_text(json.dumps(colors, indent=2) + '\n')
    print(f'{args.profile} {args.name} = {rgb_hex(rgb.mean(0))} (spread {spread:.1f})')

    reload = node.create_client(Trigger, 'color_sensor/reload')
    if reload.wait_for_service(timeout_sec=2.0):
        rclpy.spin_until_future_complete(node, reload.call_async(Trigger.Request()), timeout_sec=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
