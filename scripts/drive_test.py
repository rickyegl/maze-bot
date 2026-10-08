#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def steps(square, seconds, side, speed, spin=0.0):
    if spin:
        return [('spin', 0.0, spin)]
    if not square:
        return [('drive', 0.0, seconds)]
    out = []
    for i in range(4):
        out += [('drive', i * math.pi / 2, side / speed), ('stop', i * math.pi / 2, 0.5),
                ('turn', (i + 1) * math.pi / 2, 4.0), ('stop', (i + 1) * math.pi / 2, 0.5)]
    return out


class DriveTest(Node):
    def __init__(self):
        super().__init__('drive_test')
        p = self.declare_parameter
        self.speed = p('speed', 0.15).value
        self.turn_speed = p('turn_speed', 1.2).value
        self.kp, self.ki = p('kp', 4.0).value, p('ki', 2.0).value
        self.hold = p('hold', True).value
        self.plan = steps(p('square', False).value, p('seconds', 6.0).value, p('side', 0.5).value,
                          self.speed, p('spin', 0.0).value)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.heading = self.integral = 0.0
        self.last = self.ready = self.since = None
        self.step = 0
        self.done = False
        self.create_timer(0.02, self.tick)

    def on_imu(self, msg):
        now = time.monotonic()
        if self.last is not None and self.since is not None:
            self.heading += msg.angular_velocity.z * min(now - self.last, 0.1)
        self.last = now
        self.ready = self.ready or now

    def tick(self):
        now = time.monotonic()
        if self.done or self.ready is None or now < self.ready + 2:
            return
        if self.since is None:
            self.since = now
        what, target, length = self.plan[self.step]
        err = self.heading - target
        t = now - self.since
        msg = Twist()
        if what == 'turn' and abs(err) < math.radians(2) or what != 'turn' and t >= length:
            self.get_logger().info(f'{what} done, heading {math.degrees(self.heading):+.1f} deg')
            self.step += 1
            self.since, self.integral = now, 0.0
            if self.step == len(self.plan):
                self.finish()
            return
        if what == 'drive':
            if abs(err) > math.radians(30):
                return self.finish()
            if self.hold:
                self.integral = max(-0.5, min(0.5, self.integral + err * 0.02))
                msg.angular.z = max(-1.0, min(1.0, -self.kp * err - self.ki * self.integral))
            msg.linear.x = self.speed
        elif what == 'turn':
            if t > length:
                return self.finish()
            msg.angular.z = -math.copysign(max(0.4, min(self.turn_speed, 3 * abs(err))), err)
        elif what == 'spin':
            msg.angular.z = self.turn_speed
        self.pub.publish(msg)

    def finish(self):
        self.done = True
        self.pub.publish(Twist())
        self.get_logger().info(f'stopped, heading {math.degrees(self.heading):+.1f} deg')


def main():
    rclpy.init()
    node = DriveTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.pub.publish(Twist())
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
