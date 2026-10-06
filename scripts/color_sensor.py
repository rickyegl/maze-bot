#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger

from common import config, run


def hex_rgb(code):
    return tuple(int(code[i:i + 2], 16) for i in (1, 3, 5))


def rgb_hex(rgb):
    return '#' + ''.join('%02X' % int(np.clip(round(float(v)), 0, 255)) for v in rgb)


def mean_rgb(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        return None
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    rgb = rows[:, :msg.width * 3].reshape(-1, 3).mean(0)
    return rgb[::-1] if msg.encoding == 'bgr8' else rgb


def classify(rgb, palette):
    return min(palette, key=lambda name: np.linalg.norm(np.subtract(rgb, palette[name])), default=None)


def palette_for(colors, profile):
    names = ['white', *colors['pista_a']]
    readings = colors['calibration'].get(profile, {})
    return {n: hex_rgb(readings[n]) for n in names if n in readings}, [n for n in names if n not in readings]


class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        self.profile = self.declare_parameter('profile', 'sim').value
        self.confirm = self.declare_parameter('confirm', 3).value
        self.reload()
        self.name_pub = self.create_publisher(String, 'color', 10)
        self.hex_pub = self.create_publisher(String, 'color/hex', 10)
        self.rgb_pub = self.create_publisher(ColorRGBA, 'color/rgb', 10)
        self.create_subscription(Image, 'color_sensor/image', self.on_image, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reload', self.on_reload)

    def reload(self):
        self.palette, missing = palette_for(config('colors.json'), self.profile)
        self.current = self.candidate = None
        self.count = 0
        text = f'{self.profile}: ' + (', '.join(f'{n} {rgb_hex(v)}' for n, v in self.palette.items()) or 'nothing')
        if missing:
            text += '; not calibrated: ' + ', '.join(missing)
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        return text

    def on_reload(self, _, response):
        response.message = self.reload()
        response.success = bool(self.palette)
        return response

    def on_image(self, msg):
        rgb = mean_rgb(msg)
        if rgb is None:
            self.get_logger().warn(f'cannot read {msg.encoding} images', once=True)
            return
        self.rgb_pub.publish(ColorRGBA(r=rgb[0] / 255, g=rgb[1] / 255, b=rgb[2] / 255, a=1.0))
        self.hex_pub.publish(String(data=rgb_hex(rgb)))
        if not self.palette:
            return
        name = classify(rgb, self.palette)
        self.count = self.count + 1 if name == self.candidate else 1
        self.candidate = name
        if self.count >= self.confirm and name != self.current:
            self.current = name
            self.get_logger().info(f'colour: {name} ({rgb_hex(rgb)})')
        if self.current:
            self.name_pub.publish(String(data=self.current))


def main():
    run(ColorSensor)


if __name__ == '__main__':
    main()
