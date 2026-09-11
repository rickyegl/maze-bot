#!/usr/bin/env python3
import json
import math

import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from common import run, wrap, yaw_of

R = 6
N = 2 * R + 1
UNKNOWN, OPEN, WALL = -1, 0, 1


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def add(grid, k, s):
    k = k.astype(int) + R
    s = s.astype(int) + R
    keep = (k >= 0) & (k <= N) & (s >= 0) & (s < N)
    np.add.at(grid, (k[keep], s[keep]), 1)


class WallMap(Node):
    def __init__(self):
        super().__init__('wall_map')
        p = self.declare_parameter
        self.cell = p('cell', 0.30).value
        self.wall = p('wall_thickness', 0.012).value
        self.tol = p('hit_tol', 0.03).value
        self.size = p('track_size', 5).value
        self.lidar = np.array([p('lidar_x', 0.066).value, 0.0])
        self.frame = p('frame', 'maze').value
        self.reset()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.walls_pub = self.create_publisher(String, 'maze/walls', latched)
        self.markers_pub = self.create_publisher(MarkerArray, 'maze/wall_markers', latched)
        self.odom_pub = self.create_publisher(Odometry, 'maze/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset', self.on_reset)

    def reset(self):
        self.v = np.zeros((N + 1, N))
        self.h = np.zeros((N + 1, N))
        self.pose = np.zeros(3)
        self.odom = self.last = None
        self.sent = None

    def on_reset(self, _, response):
        self.reset()
        response.success = True
        return response

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = np.array([p.position.x, p.position.y, yaw_of(p.orientation)])

    def predict(self):
        if self.last is not None:
            d = self.odom - self.last
            local = rot(-self.last[2]) @ d[:2]
            self.pose[:2] += rot(self.pose[2]) @ local
            self.pose[2] = wrap(self.pose[2] + wrap(d[2]))
        self.last = self.odom.copy()

    def on_scan(self, scan):
        if self.odom is None:
            return
        self.predict()
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + scan.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > scan.range_min) & (r < 1.5)
        local = np.stack([r * np.cos(a), r * np.sin(a)], 1)[ok] + self.lidar
        fix = self.match(local)
        if fix is not None:
            self.pose += fix
            origin = self.pose[:2] + rot(self.pose[2]) @ self.lidar
            self.integrate(origin, self.pose[:2] + local @ rot(self.pose[2]).T)
        self.publish_pose(scan.header.stamp)
        self.report()

    def states(self):
        out = []
        for grid in (self.v, self.h):
            s = np.full(grid.shape, UNKNOWN)
            s[grid >= 1.5] = WALL
            s[grid <= -1.5] = OPEN
            out.append(s)
        return out

    def residuals(self, pts):
        u = pts[..., 0] / self.cell + 0.5
        w = pts[..., 1] / self.cell + 0.5
        return (u - np.round(u)) * self.cell, (w - np.round(w)) * self.cell, np.round(u), np.round(w)

    def weight(self, state, k, s):
        k = k.astype(int) + R
        s = s.astype(int) + R
        inside = (k >= 0) & (k <= N) & (s >= 0) & (s < N)
        table = np.select([state == WALL, state == OPEN], [1.0, 0.0], 0.5)
        return np.where(inside, table[np.clip(k, 0, N), np.clip(s, 0, N - 1)], 0.0)

    def score(self, pts, sigma=0.015):
        sv, sh = self.states()
        rx, ry, ku, kw = self.residuals(pts)
        rx = np.abs(rx) - self.wall / 2
        ry = np.abs(ry) - self.wall / 2
        wv = self.weight(sv, ku, np.round(pts[..., 1] / self.cell))
        wh = self.weight(sh, kw, np.round(pts[..., 0] / self.cell))
        g = -0.5 / sigma ** 2
        return np.maximum(wv * np.exp(g * rx * rx), wh * np.exp(g * ry * ry))

    def match(self, local, span=0.04, yaw_span=math.radians(5)):
        if len(local) < 30:
            return None
        pts = local[::max(1, len(local) // 150)]
        offsets = np.arange(-span, span + 1e-9, 0.01)
        gx, gy = np.meshgrid(offsets, offsets, indexing='ij')
        shifts = np.stack([gx.ravel(), gy.ravel()], 1)
        best, fix = -1.0, None
        for dyaw in np.linspace(-yaw_span, yaw_span, 11):
            world = self.pose[:2] + pts @ rot(self.pose[2] + dyaw).T
            scores = self.score(world[None] + shifts[:, None]).sum(1)
            i = int(scores.argmax())
            if scores[i] > best:
                best, fix = scores[i], np.array([*shifts[i], dyaw])
        return fix if best / len(pts) > 0.35 else None

    def integrate(self, origin, pts):
        hit_v, hit_h = np.zeros_like(self.v), np.zeros_like(self.h)
        miss_v, miss_h = np.zeros_like(self.v), np.zeros_like(self.h)
        rx, ry, _, _ = self.residuals(pts)
        on_v = np.abs(np.abs(rx) - self.wall / 2) < self.tol
        on_h = ~on_v & (np.abs(np.abs(ry) - self.wall / 2) < self.tol)
        add(hit_v, np.round(pts[on_v, 0] / self.cell + 0.5), np.round(pts[on_v, 1] / self.cell))
        add(hit_h, np.round(pts[on_h, 1] / self.cell + 0.5), np.round(pts[on_h, 0] / self.cell))
        ray = pts - origin
        length = np.linalg.norm(ray, axis=1, keepdims=True)
        stop = origin + ray * np.clip(1 - 2 * self.tol / length, 0, 1)
        t = np.linspace(0, 1, 40)[:, None, None]
        cells = np.round((origin + (stop - origin) * t) / self.cell).astype(int)
        a, b = cells[:-1], cells[1:]
        cross_v = (a[..., 0] != b[..., 0]) & (a[..., 1] == b[..., 1])
        cross_h = (a[..., 1] != b[..., 1]) & (a[..., 0] == b[..., 0])
        add(miss_v, np.maximum(a[..., 0], b[..., 0])[cross_v], a[..., 1][cross_v])
        add(miss_h, np.maximum(a[..., 1], b[..., 1])[cross_h], a[..., 0][cross_h])
        for grid, hit, miss in ((self.v, hit_v, miss_v), (self.h, hit_h, miss_h)):
            wall = hit >= 2
            grid[wall] += 0.6
            grid[~wall & (miss >= 2)] -= 0.3
            np.clip(grid, -5, 5, out=grid)

    def locate(self, sv, sh):
        n, best, where = self.size, -1, None
        for ox in range(n):
            for oy in range(n):
                cols, rows = slice(R - ox, R - ox + n), slice(R - oy, R - oy + n)
                edges = np.concatenate([sv[R - ox, rows], sv[R - ox + n, rows],
                                        sh[R - oy, cols], sh[R - oy + n, cols]])
                score = int((edges == WALL).sum()) - 2 * int((edges == OPEN).sum())
                if score > best:
                    best, where = score, (ox, oy)
        return where if best >= 2 * n + 2 else None

    def report(self):
        sv, sh = self.states()
        loc = self.locate(sv, sh)
        key = (sv.tobytes(), sh.tobytes(), loc)
        if key == self.sent:
            return
        self.sent = key
        i0, j0 = (-loc[0], -loc[1]) if loc else (-R, -R)
        w = h = self.size if loc else N
        vwall = [[int(sv[b + i0 + R, j0 + h - 1 - r + R]) for b in range(w + 1)] for r in range(h)]
        hwall = [[int(sh[j0 + h - b + R, c + i0 + R]) for c in range(w)] for b in range(h + 1)]
        doc = {'located': loc is not None, 'cols': w, 'rows': h,
               'start_cell': {'col': -i0, 'row': j0 + h - 1}, 'vwall': vwall, 'hwall': hwall}
        self.walls_pub.publish(String(data=json.dumps(doc)))
        self.publish_markers(sv, sh)

    def publish_markers(self, sv, sh):
        out = MarkerArray()
        for i, (state, rgba) in enumerate(((WALL, (0.95, 0.2, 0.15, 0.9)), (OPEN, (0.2, 0.8, 0.3, 0.5)))):
            for axis, grid in enumerate((sv, sh)):
                m = Marker()
                m.header.frame_id = self.frame
                m.ns, m.id, m.type = 'walls', 2 * i + axis, Marker.CUBE_LIST
                m.pose.orientation.w = 1.0
                thick, tall = (0.02, 0.15) if state == WALL else (0.01, 0.01)
                m.scale.x, m.scale.y = (thick, self.cell * 0.9) if axis == 0 else (self.cell * 0.9, thick)
                m.scale.z = tall
                m.color.r, m.color.g, m.color.b, m.color.a = rgba
                for k, s in zip(*np.nonzero(grid == state)):
                    line, along = (k - R - 0.5) * self.cell, (s - R) * self.cell
                    x, y = (line, along) if axis == 0 else (along, line)
                    m.points.append(Point(x=float(x), y=float(y), z=tall / 2))
                out.markers.append(m)
        self.markers_pub.publish(out)

    def publish_pose(self, stamp):
        x, y, a = (float(v) for v in self.pose)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, self.frame, 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(a / 2), math.cos(a / 2)
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)


def main():
    run(WallMap)


if __name__ == '__main__':
    main()
