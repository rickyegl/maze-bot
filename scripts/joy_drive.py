#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from common import config, limit, run


def shape(x, dead=0.1):
    if abs(x) < dead:
        return 0.0
    m = (abs(x) - dead) / (1 - dead)
    return math.copysign(m * m, x)


def pull(trigger):
    return min(1.0, max(0.0, (1.0 - trigger) / 2))


def arcade(drive, turn, boost, slow, speed, turn_speed, turbo):
    k = 1.0 + (turbo - 1.0) * boost - 0.7 * slow
    return speed * k * drive, turn_speed * k * turn


class JoyDrive(Node):
    def __init__(self):
        super().__init__('joy_drive')
        p = self.declare_parameter
        self.speed = p('speed', 0.15).value
        self.turn_speed = p('turn', 1.2).value
        self.turbo = p('turbo', 2.0).value
        self.axes = {k: p(f'axis_{k}', v).value for k, v in
                     {'drive': 1, 'turn': 0, 'slow': 4, 'boost': 5}.items()}
        self.stop_button = p('button_stop', 1).value
        self.timeout = p('timeout', 0.5).value
        self.drive = config('robot.json')['drive']
        self.moving = False
        self.stamp = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(0.1, self.watchdog)

    def axis(self, msg, name):
        i = self.axes[name]
        return msg.axes[i] if i < len(msg.axes) else 0.0

    def on_joy(self, msg):
        self.stamp = time.monotonic()
        stop = self.stop_button < len(msg.buttons) and msg.buttons[self.stop_button]
        drive, turn = shape(self.axis(msg, 'drive')), shape(self.axis(msg, 'turn'))
        if stop or not (drive or turn):
            return self.halt()
        v, w = arcade(drive, turn, pull(self.axis(msg, 'boost')), pull(self.axis(msg, 'slow')),
                      self.speed, self.turn_speed, self.turbo)
        self.send(v, w)
        self.moving = True

    def watchdog(self):
        if self.moving and time.monotonic() - self.stamp > self.timeout:
            self.get_logger().warn('pad went quiet, stopping')
            self.halt()

    def halt(self):
        if self.moving:
            self.send(0.0, 0.0)
        self.moving = False

    def send(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = limit(v, w, self.drive)
        self.pub.publish(msg)


def main():
    run(JoyDrive)


if __name__ == '__main__':
    main()
