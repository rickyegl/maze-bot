#!/usr/bin/env python3
import fcntl
import os
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String

from common import run

I2C_SLAVE = 0x0703


def fit(label, value, cols=16):
    text = f'{label} {value}'
    return (text if len(text) <= cols else value)[:cols].ljust(cols)


class Screen:
    def __init__(self, cols=16, hold=3.0, background=('white',)):
        self.cols, self.hold, self.background = cols, hold, set(background)
        self.latest = self.shown = self.marker = None
        self.since = 0.0

    def colour(self, name, now):
        self.latest = name
        self.update(now)

    def update(self, now):
        if self.latest in (None, self.shown):
            return
        held = (self.shown not in (None, *self.background) and self.latest in self.background
                and now - self.since < self.hold)
        if not held:
            self.shown, self.since = self.latest, now

    def rows(self, now):
        self.update(now)
        colour = self.shown.replace('_', ' ') if self.shown else '--'
        marker = '--' if self.marker is None else str(self.marker)
        return fit('Color', colour, self.cols), fit('ArUco', marker, self.cols)


class Lcd1602:
    RS, EN, LIGHT = 0x01, 0x04, 0x08

    def __init__(self, bus=1, address=0x27, cols=16):
        self.cols = cols
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        for nibble in (0x30, 0x30, 0x30, 0x20):
            os.write(self.fd, self.pulse(nibble, 0))
            time.sleep(0.005)
        for command in (0x28, 0x08, 0x01, 0x06, 0x0C):
            os.write(self.fd, self.byte(command, 0))
            time.sleep(0.002)

    def pulse(self, nibble, mode):
        b = (nibble & 0xF0) | mode | self.LIGHT
        return bytes((b | self.EN, b))

    def byte(self, value, mode):
        return self.pulse(value & 0xF0, mode) + self.pulse(value << 4 & 0xF0, mode)

    def show(self, row, text):
        data = self.byte(0x80 | 0x40 * row, 0)
        for ch in text.ljust(self.cols)[:self.cols]:
            data += self.byte(ord(ch) if 32 <= ord(ch) < 127 else ord('?'), self.RS)
        os.write(self.fd, data)


class Display(Node):
    def __init__(self):
        super().__init__('display')
        p = self.declare_parameter
        backend = p('backend', 'auto').value
        self.screen = Screen(hold=p('hold', 3.0).value)
        self.lcd = None
        if backend != 'log':
            try:
                self.lcd = Lcd1602(p('bus', 1).value, p('address', 0x27).value)
            except OSError:
                self.get_logger().info('no LCD found, logging only')
        self.rows = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'display', latched)
        self.create_subscription(String, 'color', lambda m: self.screen.colour(m.data, time.monotonic()), 10)
        self.create_subscription(Int32, 'aruco/id', self.on_marker, latched)
        self.create_timer(0.2, self.refresh)

    def on_marker(self, msg):
        self.screen.marker = msg.data

    def refresh(self):
        rows = self.screen.rows(time.monotonic())
        if rows == self.rows:
            return
        if self.lcd:
            for i, text in enumerate(rows):
                self.lcd.show(i, text)
        self.rows = rows
        self.get_logger().info('[%s] [%s]' % rows)
        self.pub.publish(String(data='\n'.join(rows)))


def main():
    run(Display)


if __name__ == '__main__':
    main()
