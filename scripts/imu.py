#!/usr/bin/env python3
import fcntl
import math
import os
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from common import run

I2C_SLAVE = 0x0703
G = 9.80665


def decode(raw):
    v = np.frombuffer(bytes(raw), dtype='>i2').astype(float)
    return v[0:3] / 8192.0, np.radians(v[4:7] / 65.5)


def tilt_of(accel):
    ax, ay, az = accel
    return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))


def quaternion(roll, pitch):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return sr * cp, cr * sp, -sr * sp, cr * cp


class Tilt:
    def __init__(self, tau=1.0, band=0.15):
        self.tau, self.band = tau, band
        self.roll = self.pitch = None

    def update(self, gyro, accel, dt):
        if self.roll is None:
            self.roll, self.pitch = tilt_of(accel)
            return self.roll, self.pitch
        self.roll += gyro[0] * dt
        self.pitch += gyro[1] * dt
        if abs(np.linalg.norm(accel) - 1.0) < self.band:
            k = min(1.0, dt / self.tau)
            roll, pitch = tilt_of(accel)
            self.roll += k * (roll - self.roll)
            self.pitch += k * (pitch - self.pitch)
        return self.roll, self.pitch


class Mpu6050:
    def __init__(self, bus=1, address=0x68):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        self.write(0x6B, 0x80)
        time.sleep(0.1)
        for reg, value in ((0x6B, 0x01), (0x1A, 0x03), (0x19, 4), (0x1B, 0x08), (0x1C, 0x08)):
            self.write(reg, value)

    def write(self, reg, value):
        os.write(self.fd, bytes((reg, value)))

    def sample(self):
        os.write(self.fd, bytes((0x3B,)))
        return decode(os.read(self.fd, 14))


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu')
        p = self.declare_parameter
        self.chip = Mpu6050(p('bus', 1).value, p('address', 0x68).value)
        self.still_time = p('still_time', 1.0).value
        self.tilt = Tilt(p('tilt_tau', 1.0).value)
        self.bias = None
        self.g = 1.0
        self.still = []
        self.stamp = None
        self.pub = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)
        self.get_logger().info('hold the car still')

    def tick(self):
        try:
            accel, gyro = self.chip.sample()
        except OSError:
            return
        now = time.monotonic()
        if self.bias is None:
            return self.calibrate(now, accel, gyro)
        accel, gyro = accel / self.g, gyro - self.bias
        dt = 0.0 if self.stamp is None else min(now - self.stamp, 0.1)
        self.stamp = now
        roll, pitch = self.tilt.update(gyro, accel, dt)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        q = msg.orientation
        q.x, q.y, q.z, q.w = quaternion(roll, pitch)
        w, a = msg.angular_velocity, msg.linear_acceleration
        w.x, w.y, w.z = (float(v) for v in gyro)
        a.x, a.y, a.z = (float(v) * G for v in accel)
        self.pub.publish(msg)

    def calibrate(self, now, accel, gyro):
        self.still.append((now, accel, gyro))
        if now - self.still[0][0] < self.still_time:
            return
        gyros = np.array([s[2] for s in self.still])
        self.g = float(np.linalg.norm(np.mean([s[1] for s in self.still], 0)))
        self.still = []
        if np.abs(gyros - gyros.mean(0)).max() > 0.05:
            return self.get_logger().warn('moved, measuring again')
        self.bias = gyros.mean(0)
        self.get_logger().info(f'gyro bias {np.degrees(self.bias).round(2)} deg/s, {self.g:.2f} g at rest')


def main():
    run(ImuNode)


if __name__ == '__main__':
    main()
