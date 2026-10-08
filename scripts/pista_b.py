#!/usr/bin/env python3
import math
from collections import deque

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

import pista_b_lines as lines
import pista_b_track as track
from common import config, limit, run, wrap, yaw_of

C = track.CELL


class EdgeVotes:
    def __init__(self, trim=0.03, near=0.025, beyond=0.05, least=3):
        self.trim, self.near, self.beyond, self.least = trim, near, beyond, least
        self.wall = dict.fromkeys(track.SIDES, 0)
        self.open = dict.fromkeys(track.SIDES, 0)

    def add(self, origin, pts):
        ox, oy = origin
        for side in track.SIDES:
            (ax, ay), (bx, by) = track.edge(track.BALL, side)
            ex, ey = bx - ax, by - ay
            ux, uy = ex / C, ey / C
            along = (pts[:, 0] - ax) * ux + (pts[:, 1] - ay) * uy
            off = np.abs((pts[:, 0] - ax) * uy - (pts[:, 1] - ay) * ux)
            on = (off < self.near) & (along > self.trim) & (along < C - self.trim)
            dx, dy = pts[:, 0] - ox, pts[:, 1] - oy
            den = dx * ey - dy * ex
            with np.errstate(divide='ignore', invalid='ignore'):
                t = ((ax - ox) * ey - (ay - oy) * ex) / den
                u = ((ax - ox) * dy - (ay - oy) * dx) / den
            reach = np.hypot(dx, dy)
            through = (t > 0) & (t * reach < reach - self.beyond) & (u > self.trim / C) & (u < 1 - self.trim / C)
            self.wall[side] += on.sum() >= self.least
            self.open[side] += through.sum() >= self.least

    def state(self, side, votes=3):
        w, o = self.wall[side], self.open[side]
        if o >= votes and o > 2 * w:
            return 'open'
        if w >= votes and w > 2 * o:
            return 'wall'

    def open_side(self):
        states = {s: self.state(s) for s in track.SIDES}
        opened = [s for s, v in states.items() if v == 'open']
        walled = [s for s, v in states.items() if v == 'wall']
        if len(opened) == 1:
            return opened[0]
        if len(walled) == 3 and not opened:
            return next(s for s in track.SIDES if s not in walled)


def room_path(start, goal, entry=None):
    free = set(track.ROOM1) - {track.BALL} | ({entry} if entry else set())
    prev, todo = {start: None}, deque([start])
    while todo:
        here = todo.popleft()
        for d in track.SIDES.values():
            nxt = track.step(here, d)
            if nxt not in free or nxt in prev:
                continue
            if entry in (here, nxt) and track.into_room(entry) not in (here, nxt):
                continue
            prev[nxt] = here
            todo.append(nxt)
    path, here = [], goal
    while here is not None:
        path.append(here)
        here = prev.get(here)
    return path[::-1]


def unit_colour(readings, cell, inset=0.04, least=5, share=0.75):
    x0, y0 = cell[0] * C + inset, cell[1] * C + inset
    names = [n for n, x, y in readings if x0 < x < x0 + C - 2 * inset and y0 < y < y0 + C - 2 * inset]
    if len(names) < least:
        return None
    best = max(set(names), key=names.count)
    return best if names.count(best) >= share * len(names) else None


def next_unit(cell, colour, been):
    d = track.COLOUR_STEP.get(colour)
    if d is None:
        return None
    nxt = track.step(cell, d)
    if nxt in been or nxt not in track.ROOM3 and nxt not in track.FIN_CELLS:
        return None
    return nxt


class PistaB(Node):
    def __init__(self):
        super().__init__('pista_b')
        p = self.declare_parameter
        robot = config('robot.json')
        self.drive, self.grip, sensors = robot['drive'], robot['gripper'], robot['sensors']
        self.speed = p('speed', 0.15).value
        self.slow = p('slow_speed', 0.05).value
        self.turn_speed = p('turn_speed', 1.0).value
        self.stall_time = p('stall_time', 2.0).value
        self.carry_arm = p('carry_arm', -0.68).value
        self.pan = math.radians(p('pan_deg', 12.0).value)
        self.cam_x, self.cam_h = sensors['camera_x'], sensors['camera_height']
        self.sensor_x, self.lidar_x = sensors['color_sensor_x'], sensors['lidar_x']
        self.go = p('autostart', False).value
        pin = p('button_gpio', -1).value

        self.pose = self.rays = self.start_cell = None
        self.turning = self.looking = self.done = False
        self.arm = self.claw = 0.0
        self.votes = EdgeVotes()
        self.linemap = lines.LineMap()
        self.colours = deque(maxlen=100)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_pub = self.create_publisher(Float64, 'gripper/arm_cmd', 10)
        self.claw_pub = self.create_publisher(Float64, 'gripper/claw_cmd', 10)
        self.status = self.create_publisher(String, '~/status', latched)
        self.create_subscription(Odometry, 'maze/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, 'color', self.on_colour, 10)
        self.create_service(Trigger, '~/start', self.on_start)
        self.button = self.watch(pin)
        self.steps = self.mission()
        self.create_timer(0.05, self.tick)
        self.say('waiting for ~/start')

    def watch(self, pin):
        if pin < 0:
            return None
        try:
            from gpiozero import Button
        except ImportError:
            return None
        button = Button(pin, pull_up=True, bounce_time=0.05)
        button.when_pressed = self.start
        return button

    def start(self):
        self.go = True

    def on_start(self, _, response):
        self.start()
        response.success = True
        return response

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_scan(self, msg):
        if self.pose is None or self.turning:
            return
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.08) & (r < 3.0)
        x, y, yaw = self.pose
        lx, ly = x + math.cos(yaw) * self.lidar_x, y + math.sin(yaw) * self.lidar_x
        pts = np.stack([lx + r[ok] * np.cos(a[ok] + yaw), ly + r[ok] * np.sin(a[ok] + yaw)], 1)
        self.votes.add((lx, ly), pts)

    def on_info(self, msg):
        if self.rays is None:
            self.rays = lines.floor_rays(*lines.intrinsics(list(msg.k), msg.width, msg.height), msg.width, msg.height)

    def on_image(self, msg):
        if not self.looking or self.turning or self.rays is None or self.pose is None:
            return
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :3 * msg.width]
        rgb = rgb.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            rgb = rgb[..., ::-1]
        V, U, fwd, left = self.rays
        px, py = lines.to_track(fwd, left, self.cam_h, self.cam_x, self.pose)
        self.linemap.add(px, py, *lines.classify(rgb[V, U]))

    def on_colour(self, msg):
        if self.pose is not None:
            x, y, yaw = self.pose
            self.colours.append((msg.data, x + math.cos(yaw) * self.sensor_x, y + math.sin(yaw) * self.sensor_x))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def send(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = limit(v, w, self.drive)
        self.turning = abs(w) > 0.3
        self.cmd.publish(msg)

    def here(self):
        return track.cell_of(self.pose[0], self.pose[1])

    def tick(self):
        if self.done:
            return
        try:
            next(self.steps)
        except StopIteration:
            self.done = True
            self.send(0.0, 0.0)

    def mission(self):
        while not self.go or self.pose is None:
            yield
        self.set_gripper(self.grip['arm_stowed'], self.grip['claw_open'])
        ok = (yield from self.section1()) and (yield from self.section2()) and (yield from self.section3())
        self.send(0.0, 0.0)
        self.say('done, on FIN' if ok else 'gave up')

    def wait(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            self.send(0.0, 0.0)
            yield

    def turn_to(self, heading, tol=math.radians(2)):
        end = self.now() + 5.0
        while self.now() < end:
            err = wrap(heading - self.pose[2])
            if abs(err) < tol:
                break
            self.send(0.0, math.copysign(max(0.5, min(self.turn_speed, 3 * abs(err))), err))
            yield
        yield from self.wait(0.2)

    def go_to(self, x, y, reverse=None, speed=None, tol=0.02):
        speed = speed or self.speed
        best, since = math.inf, self.now()
        while True:
            px, py, a = self.pose
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            if dist < best - 0.003:
                best, since = dist, self.now()
            elif self.now() - since > self.stall_time:
                self.get_logger().warn(f'stuck at ({px:.2f}, {py:.2f})')
                break
            if dist < 0.05:
                along = dx * math.cos(a) + dy * math.sin(a)
                if abs(along) < tol:
                    break
                self.send(math.copysign(max(0.03, min(speed, 1.5 * abs(along))), along), 0.0)
                yield
                continue
            err = wrap(math.atan2(dy, dx) - a)
            if reverse is None:
                reverse = abs(err) > math.pi / 2
            if reverse:
                err = wrap(err + math.pi)
            if abs(err) > 0.3:
                since = self.now()
                self.send(0.0, math.copysign(self.turn_speed, err))
            else:
                self.send((-1 if reverse else 1) * min(speed, 1.5 * dist), 3.0 * err)
            yield
        yield from self.wait(0.2)

    def set_gripper(self, arm=None, claw=None):
        if arm is not None:
            self.arm_pub.publish(Float64(data=float(arm)))
        if claw is not None:
            self.claw_pub.publish(Float64(data=float(claw)))

    def move_gripper(self, arm=None, claw=None):
        t = 0.5
        if arm is not None:
            t, self.arm = t + abs(arm - self.arm) / self.grip['arm_speed'], arm
        if claw is not None:
            t, self.claw = t + abs(claw - self.claw) / self.grip['claw_speed'], claw
        end = self.now() + t
        while self.now() < end:
            self.set_gripper(arm, claw)
            self.send(0.0, 0.0)
            yield

    def look(self, seconds):
        end = self.now() + seconds
        while self.votes.open_side() is None and self.now() < end:
            self.send(0.0, 0.0)
            yield
        return self.votes.open_side()

    def walk(self, path):
        for cell in path[1:]:
            yield from self.go_to(*track.centre(cell))

    def section1(self):
        self.start_cell = self.here()
        self.say(f'section 1: start unit {self.start_cell}, looking for the ball')
        side = yield from self.look(1.5)
        if side is None:
            first = track.into_room(self.start_cell)
            yield from self.go_to(*track.centre(first))
            k = track.RING.index(first) if first in track.RING else 0
            for i in range(1, len(track.RING) + 1):
                side = yield from self.look(1.0)
                if side:
                    break
                yield from self.go_to(*track.centre(track.RING[(k + i) % len(track.RING)]))
        if side is None:
            self.say('section 1: no open side found')
            return False
        self.say(f'section 1: the ball is open to the {side}')
        d = track.SIDES[side]
        front = track.step(track.BALL, d)
        entry = self.start_cell if self.here() == self.start_cell else None
        yield from self.walk(room_path(self.here(), front, entry))
        yield from self.go_to(*track.centre(front))
        yield from self.turn_to(track.HEADING[side])
        yield from self.move_gripper(arm=self.grip['arm_lowered'], claw=self.grip['claw_open'])
        bx, by = track.centre(track.BALL)
        reach = -self.grip['ball_in_jaws_x']
        yield from self.go_to(bx + d[0] * reach, by + d[1] * reach, reverse=True, speed=self.slow, tol=0.005)
        yield from self.move_gripper(claw=self.grip['claw_closed'])
        yield from self.move_gripper(arm=self.carry_arm)
        self.say('section 1: got the ball')
        yield from self.walk(room_path(front, track.step(track.CP1, (-1, 0))))
        yield from self.go_to(*track.centre(track.CP1), reverse=True)
        return self.here() == track.CP1

    def section2(self):
        self.say('section 2: looking at the lines')
        self.looking = True
        for yaw in (math.pi - self.pan, math.pi + self.pan, math.pi):
            yield from self.turn_to(yaw)
            yield from self.wait(1.2)
        cy = track.centre(track.CP1)[1]
        yield from self.go_to(track.FIELD[0] * C + 0.12, cy, reverse=True)
        for x in track.LINE_X:
            lo, hi, seen = self.linemap.gap(x * C)
            self.say(f'section 2: gap at x {x * C:.2f} from {lo:.2f} to {hi:.2f}, {seen:.0%} seen')
            yield from self.go_to(x * C, (lo + hi) / 2, reverse=True)
        yield from self.go_to(track.FIELD[1] * C - 0.08, cy, reverse=True)
        yield from self.go_to(*track.centre(track.CP2), reverse=True)
        self.looking = False
        return self.here() == track.CP2

    def read(self, cell):
        end = self.now() + 2.0
        while self.now() < end:
            colour = unit_colour(self.colours, cell)
            if colour:
                return colour
            self.send(0.0, 0.0)
            yield
        return None

    def section3(self):
        self.say('section 3: following the colours')
        cell, been = track.ENTRY3, [track.CP2]
        while True:
            yield from self.go_to(*track.centre(cell))
            colour = yield from self.read(cell)
            nxt = next_unit(cell, colour, been)
            if nxt is None:
                self.say(f'section 3: {cell} reads {colour}, lost')
                return False
            self.say(f'section 3: {cell} is {colour}, on to {nxt}')
            been.append(cell)
            if nxt not in track.ROOM3:
                break
            cell = nxt
        yield from self.go_to(*track.centre(nxt))
        return self.here() == nxt


def main():
    run(PistaB)


if __name__ == '__main__':
    main()
