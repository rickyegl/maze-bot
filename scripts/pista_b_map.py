#!/usr/bin/env python3
import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import pista_b_track as track
from common import run, wrap


class Likelihood:
    def __init__(self, segments, res=0.01, sigma=0.02, margin=0.5):
        pts = np.array([p for s in segments for p in s])
        self.x0, self.y0 = pts.min(0) - margin
        x1, y1 = pts.max(0) + margin
        self.res = res
        gx = self.x0 + (np.arange(int((x1 - self.x0) / res) + 1) + 0.5) * res
        gy = self.y0 + (np.arange(int((y1 - self.y0) / res) + 1) + 0.5) * res
        X, Y = np.meshgrid(gx, gy, indexing='ij')
        d = np.full(X.shape, np.inf)
        for (ax, ay), (bx, by) in segments:
            vx, vy = bx - ax, by - ay
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / (vx * vx + vy * vy), 0, 1)
            d = np.minimum(d, np.hypot(X - ax - t * vx, Y - ay - t * vy))
        self.grid = np.exp(-0.5 * (d / sigma) ** 2).astype(np.float32)

    def scores(self, poses, pts):
        c, s = np.cos(poses[:, 2:3]), np.sin(poses[:, 2:3])
        ix = ((poses[:, 0:1] + c * pts[:, 0] - s * pts[:, 1] - self.x0) / self.res).astype(int)
        iy = ((poses[:, 1:2] + s * pts[:, 0] + c * pts[:, 1] - self.y0) / self.res).astype(int)
        nx, ny = self.grid.shape
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        return np.where(ok, self.grid[ix.clip(0, nx - 1), iy.clip(0, ny - 1)], 0).mean(1)


def search(field, pts, centre, span, yaw_span, n=9):
    d, a = np.linspace(-span, span, n), np.linspace(-yaw_span, yaw_span, n)
    X, Y, A = np.meshgrid(d, d, a, indexing='ij')
    poses = np.stack([centre[0] + X.ravel(), centre[1] + Y.ravel(), centre[2] + A.ravel()], 1)
    sc = field.scores(poses, pts)
    k = int(np.argmax(sc))
    return poses[k], float(sc[k])


def refine(field, pts, centre, span, yaw_span):
    pose, _ = search(field, pts, centre, span, yaw_span)
    return search(field, pts, pose, span / 4, yaw_span / 4)


def known_walls(start):
    ball = [track.edge(track.BALL, s) for s in track.SIDES]
    return track.outline(start=start) + ball + track.fin_pockets()


def locate(pts, fields):
    best = None
    for cell in track.START_CELLS:
        x, y = track.centre(cell)
        for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
            pose, sc = refine(fields[cell], pts, (x, y, yaw), 0.08, math.radians(20))
            if best is None or sc > best[2]:
                best = (cell, pose, sc)
    return best


def scan_points(scan, lidar_x, most=200):
    r = np.asarray(scan.ranges, float)
    a = scan.angle_min + scan.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.08) & (r < 6.0)
    r, a = r[ok], a[ok]
    if len(r) > most:
        keep = np.linspace(0, len(r) - 1, most).astype(int)
        r, a = r[keep], a[keep]
    return np.stack([lidar_x + r * np.cos(a), r * np.sin(a)], 1)


class PistaBMap(Node):
    def __init__(self):
        super().__init__('pista_b_map')
        p = self.declare_parameter
        self.span = p('search_xy', 0.04).value
        self.yaw_span = math.radians(p('search_yaw_deg', 5.0).value)
        self.min_fit = p('min_fit', 0.3).value
        self.lidar_x = p('lidar_x', 0.066).value
        self.fields = self.field = self.pose = self.stamp = self.gyro_t = None
        self.vel = np.zeros(2)
        self.fails = 0
        self.gyro = self.gyro_fix = 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom = self.create_publisher(Odometry, 'maze/odom', 10)
        self.start_pub = self.create_publisher(String, '~/start', latched)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.gyro_t is not None and 0 < t - self.gyro_t < 0.2:
            self.gyro += msg.angular_velocity.z * (t - self.gyro_t)
        self.gyro_t = t

    def on_scan(self, scan):
        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        pts = scan_points(scan, self.lidar_x)
        if len(pts) < 30:
            return
        if self.pose is None:
            self.start(pts)
        else:
            self.track(pts, t)
        self.stamp, self.gyro_fix = t, self.gyro
        self.publish(scan.header.stamp)

    def start(self, pts):
        if self.fields is None:
            self.fields = {c: Likelihood(known_walls(c)) for c in track.START_CELLS}
        cell, pose, sc = locate(pts, self.fields)
        self.field, self.pose = self.fields[cell], pose
        self.get_logger().info(f'start unit {cell}, facing {math.degrees(pose[2]):.0f}, fit {sc:.2f}')
        self.start_pub.publish(String(data=f'{cell[0]},{cell[1]}'))

    def track(self, pts, t):
        dt = min(max(t - self.stamp, 0.0), 0.5)
        guess = self.pose.copy()
        guess[:2] += self.vel * dt
        guess[2] += self.gyro - self.gyro_fix
        lost = self.fails >= 3
        pose, sc = refine(self.field, pts, guess, 0.12 if lost else self.span,
                          math.radians(15) if lost else self.yaw_span)
        if sc < self.min_fit:
            self.fails += 1
            self.pose, self.vel = guess, self.vel * 0.3
            return
        if dt > 0:
            self.vel = 0.5 * self.vel + 0.5 * (pose[:2] - self.pose[:2]) / dt
        pose[2] = wrap(pose[2])
        self.pose, self.fails = pose, 0

    def publish(self, stamp):
        x, y, yaw = self.pose
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'maze', 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header, odom.child_frame_id = tf.header, 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
        odom.pose.pose.orientation = tf.transform.rotation
        self.odom.publish(odom)


def main():
    run(PistaBMap)


if __name__ == '__main__':
    main()
