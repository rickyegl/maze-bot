#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from common import config, run


def hex_rgb(code):
    return tuple(int(code[i:i + 2], 16) for i in (1, 3, 5))


def rgb_hex(rgb):
    return '#' + ''.join('%02X' % int(np.clip(round(float(v)), 0, 255)) for v in rgb)


def mean_rgb(msg):
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    rgb = rows[:, :msg.width * 3].reshape(-1, 3).mean(0)
    return rgb[::-1] if msg.encoding == 'bgr8' else rgb


def classify(rgb, palette):
    return min(palette, key=lambda name: np.linalg.norm(np.subtract(rgb, palette[name])), default=None)


class ColorSensor(Node):
    def __init__(self):
        super().__init__('color_sensor')
        profile = self.declare_parameter('profile', 'sim').value
        self.confirm = self.declare_parameter('confirm', 3).value
        readings = config('colors.json')['calibration'][profile]
        self.palette = {name: hex_rgb(code) for name, code in readings.items()}
        self.current = self.candidate = None
        self.count = 0
        self.name_pub = self.create_publisher(String, 'color', 10)
        self.hex_pub = self.create_publisher(String, 'color/hex', 10)
        self.create_subscription(Image, 'color_sensor/image', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        rgb = mean_rgb(msg)
        self.hex_pub.publish(String(data=rgb_hex(rgb)))
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
