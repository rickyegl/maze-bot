#!/usr/bin/env python3
import math
import sys
import time

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from common import config, run

MOTORS = {1: (17, 22, 27), 2: (11, 10, 9), 3: (5, 19, 6), 4: (21, 16, 20)}
ENCODERS = {1: 14, 2: 18, 3: 24, 4: 8}
STOP_GPIO = 26


def sign(x):
    return (x > 0) - (x < 0)


def ramp(current, target, step):
    return current + max(-step, min(step, target - current))


def full_speed(drive):
    return drive['max_wheel_rpm'] * math.pi / 30 * drive['wheel_radius']


def wheel_speeds(v, w, drive):
    half = w * drive['wheel_separation'] / 2
    full = full_speed(drive)
    return tuple(max(-full, min(full, s)) for s in (v - half, v + half))


def motor_speeds(left, right, layout):
    out = {}
    for side, v in (('left', left), ('right', right)):
        for m in layout[side]:
            out[m] = -v if m in layout['reversed'] else v
    return out


class SpeedLoop:
    def __init__(self, drive, layout):
        self.per_metre = layout['edges_per_rev'] / (2 * math.pi * drive['wheel_radius'])
        self.feed = 1 / full_speed(drive)
        self.kp, self.ki = layout['speed_kp'], layout['speed_ki']
        self.top, self.window = layout['max_duty'], layout['speed_window']
        self.history = []
        self.integral = 0.0
        self.direction = 0
        self.last = None

    def measure(self, now, edges):
        self.history.append((now, edges))
        while len(self.history) > 2 and now - self.history[1][0] >= self.window:
            del self.history[0]
        then, before = self.history[0]
        return (edges - before) / (now - then) / self.per_metre if now > then else 0.0

    def update(self, now, target, edges):
        dt = 0.0 if self.last is None else min(now - self.last, 0.1)
        self.last = now
        speed = self.measure(now, edges)
        if sign(target) != self.direction:
            self.integral, self.direction = 0.0, sign(target)
        if target == 0:
            return 0.0
        error = target - speed * self.direction
        self.integral = max(-self.top, min(self.top, self.integral + self.ki * error * dt))
        duty = self.feed * target + self.kp * error + self.integral
        return math.copysign(min(max(duty * sign(target), 0.0), self.top), target)


def header_chip(lgpio):
    for n in range(16):
        try:
            h = lgpio.gpiochip_open(n)
        except lgpio.error:
            continue
        label = lgpio.gpio_get_chip_info(h)[3]
        lgpio.gpiochip_close(h)
        if label.startswith('pinctrl-'):
            return n
    raise RuntimeError('no pinctrl gpiochip')


class Drivers:
    def __init__(self, frequency, stop_gpio=STOP_GPIO):
        import lgpio
        self.lg, self.frequency, self.stop_gpio = lgpio, frequency, stop_gpio
        self.h = lgpio.gpiochip_open(header_chip(lgpio))
        self.duty = {}
        for m, pins in MOTORS.items():
            for pin in pins:
                lgpio.gpio_claim_output(self.h, pin, 0)
            self.brake(m)
        if stop_gpio >= 0:
            lgpio.gpio_claim_input(self.h, stop_gpio, lgpio.SET_PULL_UP)
        self.encoders = {}
        for m, a in ENCODERS.items():
            lgpio.gpio_claim_alert(self.h, a, lgpio.RISING_EDGE, lgpio.SET_PULL_UP)
            self.encoders[m] = lgpio.callback(self.h, a, lgpio.RISING_EDGE)

    def edges(self):
        return {m: cb.tally() for m, cb in self.encoders.items()}

    def stop_pressed(self):
        return self.stop_gpio >= 0 and self.lg.gpio_read(self.h, self.stop_gpio) == 0

    def brake(self, m):
        pwm, in1, in2 = MOTORS[m]
        self.lg.tx_pwm(self.h, pwm, self.frequency, 0)
        self.lg.gpio_write(self.h, in1, 1)
        self.lg.gpio_write(self.h, in2, 1)
        self.duty[m] = 0.0

    def set(self, m, duty):
        duty = max(-1.0, min(1.0, duty))
        if duty == self.duty[m]:
            return
        if duty == 0:
            return self.brake(m)
        pwm, in1, in2 = MOTORS[m]
        if sign(duty) != sign(self.duty[m]):
            self.lg.tx_pwm(self.h, pwm, self.frequency, 0)
            self.lg.gpio_write(self.h, in1, int(duty > 0))
            self.lg.gpio_write(self.h, in2, int(duty < 0))
        self.lg.tx_pwm(self.h, pwm, self.frequency, abs(duty) * 100)
        self.duty[m] = duty

    def close(self):
        try:
            for m in MOTORS:
                self.brake(m)
            for cb in self.encoders.values():
                cb.cancel()
        finally:
            self.lg.gpiochip_close(self.h)


class Motors(Node):
    def __init__(self):
        super().__init__('motors')
        robot = config('robot.json')
        self.drive, self.layout = robot['drive'], robot['motors']
        stop_gpio = self.declare_parameter('stop_gpio', STOP_GPIO).value
        self.loops = {m: SpeedLoop(self.drive, self.layout) for m in MOTORS}
        self.drivers = Drivers(self.layout['pwm_frequency'], stop_gpio)
        self.target = (0.0, 0.0)
        self.rates = [0.0, 0.0]
        self.stamp = -math.inf
        self.last = None
        self.stopped = False
        self.create_subscription(Twist, 'drive/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'left {self.layout["left"]}, right {self.layout["right"]}, STOP on GPIO {stop_gpio}')

    def on_cmd(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.target, self.stamp = (msg.linear.x, msg.angular.z), time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = 0.0 if self.last is None else min(now - self.last, 0.1)
        self.last = now
        if not self.stopped and self.drivers.stop_pressed():
            self.stopped = True
            self.get_logger().error('STOP pressed, braked until restart')
        if self.stopped or now - self.stamp > self.drive['command_timeout']:
            self.rates = [0.0, 0.0]
        else:
            self.rates[0] = ramp(self.rates[0], self.target[0], self.drive['linear_acceleration'] * dt)
            self.rates[1] = ramp(self.rates[1], self.target[1], self.drive['angular_acceleration'] * dt)
        speeds = motor_speeds(*wheel_speeds(*self.rates, self.drive), self.layout)
        edges = self.drivers.edges()
        for m in MOTORS:
            self.drivers.set(m, self.loops[m].update(now, speeds[m], edges[m]))

    def destroy_node(self):
        self.drivers.close()
        super().destroy_node()


def spin_each(motors, duty=0.45, seconds=2.0):
    drivers = Drivers(config('robot.json')['motors']['pwm_frequency'])
    try:
        print('wheels off the ground, 12 V on')
        time.sleep(3)
        for m in motors:
            print(f'M{m} IN1 high at {duty:.0%}', flush=True)
            before = drivers.edges()[m]
            drivers.set(m, duty)
            time.sleep(seconds)
            drivers.brake(m)
            rate = (drivers.edges()[m] - before) / seconds
            print(f'  {rate:.0f} edges/s' + ('  not turning?' if rate < 20 else ''), flush=True)
            time.sleep(2)
    finally:
        drivers.close()


def main():
    if '--test' in sys.argv:
        picked = [int(a) for a in sys.argv[sys.argv.index('--test') + 1:] if a.isdigit()]
        return spin_each([m for m in MOTORS if m in picked] or list(MOTORS))
    run(Motors)


if __name__ == '__main__':
    main()
