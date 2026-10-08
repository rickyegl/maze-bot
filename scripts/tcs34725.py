#!/usr/bin/env python3
import fcntl
import os

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from common import run

I2C_SLAVE = 0x0703
GAINS = {1: 0, 4: 1, 16: 2, 60: 3}


def to_rgb(r, g, b, level=551.0):
    total = r + g + b
    if total <= 0:
        return 0, 0, 0
    return tuple(min(255, round(level * v / total)) for v in (r, g, b))


class Tcs34725:
    def __init__(self, bus=1, address=0x29, gain=4, integration_ms=24.0):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        self.cycles = max(1, min(256, round(integration_ms / 2.4)))
        self.write(0x01, 256 - self.cycles)
        self.write(0x0F, GAINS[gain])
        self.write(0x00, 0x01)
        self.write(0x00, 0x03)

    def write(self, reg, value):
        os.write(self.fd, bytes((0x80 | reg, value)))

    def sample(self):
        os.write(self.fd, bytes((0xA0 | 0x14,)))
        d = os.read(self.fd, 8)
        return tuple(d[i] | d[i + 1] << 8 for i in range(0, 8, 2))


class ColourSensor(Node):
    def __init__(self):
        super().__init__('tcs34725')
        p = self.declare_parameter
        self.chip = Tcs34725(p('bus', 1).value, p('address', 0x29).value, p('gain', 4).value,
                             p('integration_ms', 24.0).value)
        self.level = p('level', 551.0).value
        self.full = min(65535, 1024 * self.chip.cycles)
        self.pub = self.create_publisher(Image, 'color_sensor/image', qos_profile_sensor_data)
        self.create_timer(1.0 / p('rate', 20.0).value, self.tick)

    def tick(self):
        try:
            clear, r, g, b = self.chip.sample()
        except OSError:
            return
        if clear >= 0.95 * self.full:
            self.get_logger().warn('saturated, lower gain', throttle_duration_sec=5.0)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'color_sensor_link'
        msg.height = msg.width = 1
        msg.encoding, msg.step = 'rgb8', 3
        msg.data = bytes(to_rgb(r, g, b, self.level))
        self.pub.publish(msg)


def main():
    run(ColourSensor)


if __name__ == '__main__':
    main()
