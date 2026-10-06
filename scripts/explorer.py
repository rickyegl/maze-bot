#!/usr/bin/env python3
import json
import math
from collections import deque

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

import strategies
from common import DIRS, config, limit, run, step, wrap, yaw_of


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        p = self.declare_parameter
        self.cell = p('cell', 0.30).value
        self.size = p('track_size', 5).value
        self.speed = p('speed', 0.15).value
        self.turn_speed = p('turn_speed', 1.0).value
        self.run_time = p('run_time', 360.0).value
        self.stall_time = p('stall_time', 2.0).value
        self.goal_color = p('goal_color', 'goal_red').value
        self.strategy = strategies.make(p('strategy', 'flood').value, p('strategy_seed', 0).value)
        self.drive = config('robot.json')['drive']
        pin = p('button_gpio', -1).value
        pull_up = p('button_active_low', True).value

        self.pose = self.doc = None
        self.here, self.head, self.travel = (0, 0), None, None
        self.visited = {(0, 0)}
        self.blocked = set()
        self.path = []
        self.target = self.goal = None
        self.finishing = self.returning = self.done = False
        self.started = None
        self.best, self.stuck = math.inf, 0.0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker = self.create_publisher(Marker, '~/goal', latched)
        self.create_subscription(String, 'maze/walls', self.on_walls, latched)
        self.create_subscription(Odometry, 'maze/odom', self.on_odom, 10)
        self.create_subscription(String, 'color', self.on_color, 10)
        self.create_subscription(Bool, '~/button', lambda m: m.data and self.go(), 10)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_timer(0.05, self.tick)
        self.button = self.watch(pin, pull_up)
        self.get_logger().info(f'exploring by {self.strategy.name}, waiting for ~/start')

    def watch(self, pin, pull_up):
        if pin < 0:
            return None
        try:
            from gpiozero import Button
        except ImportError:
            self.get_logger().warn(f'no gpiozero, GPIO {pin} is not read')
            return None
        button = Button(pin, pull_up=pull_up, bounce_time=0.05)
        button.when_pressed = self.go
        return button

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def go(self):
        if self.started is None:
            self.started = self.now()
            self.get_logger().info('go')

    def on_start(self, _, response):
        response.success = self.started is None
        self.go()
        return response

    def on_walls(self, msg):
        self.doc = json.loads(msg.data)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_color(self, msg):
        if msg.data != self.goal_color or self.goal or self.finishing or self.returning or not self.path:
            return
        self.goal = self.target or self.here
        self.visited.discard(self.goal)
        self.here = self.target = self.path.pop()
        self.get_logger().info(f'casilla final at {self.goal}, saving it for last')

    def open_edge(self, cell, d):
        if (cell, d) in self.blocked:
            return False
        if self.doc is None:
            return True
        s = self.doc['start_cell']
        col, row = cell[0] + s['col'], s['row'] - cell[1]
        grid, r, c = {(1, 0): ('vwall', row, col + 1), (-1, 0): ('vwall', row, col),
                      (0, 1): ('hwall', row, col), (0, -1): ('hwall', row + 1, col)}[d]
        grid = self.doc[grid]
        return not (0 <= r < len(grid) and 0 <= c < len(grid[r])) or grid[r][c] != 1

    def on_track(self, cell):
        box = self.visited | {self.here, cell}
        if max(c[0] for c in box) - min(c[0] for c in box) >= self.size:
            return False
        if max(c[1] for c in box) - min(c[1] for c in box) >= self.size:
            return False
        if self.doc is None or not self.doc['located']:
            return True
        s = self.doc['start_cell']
        return 0 <= cell[0] + s['col'] < self.doc['cols'] and 0 <= s['row'] - cell[1] < self.doc['rows']

    def route(self, wanted):
        queue, seen = deque([(self.here, None, 0)]), {self.here}
        while queue:
            cell, first, n = queue.popleft()
            if cell != self.here and wanted(cell):
                return first, n
            for d in DIRS:
                nxt = step(cell, d)
                if nxt in seen or not self.on_track(nxt) or not self.open_edge(cell, d):
                    continue
                if nxt == self.goal and not wanted(nxt):
                    continue
                seen.add(nxt)
                queue.append((nxt, first or d, n + 1))
        return None, None

    def plan(self):
        if self.returning:
            if not self.path:
                return self.stop(f'back at the start, {len(self.visited)} cells crossed')
            self.here = self.target = self.path.pop()
            return
        if not self.finishing:
            self.strategy.note(self)
            d = self.strategy.choose(self)
            if d is None or self.now() - self.started > self.run_time / 2:
                self.finishing = True
                self.get_logger().info(f'coverage {len(self.visited)} cells, heading for the goal')
        if self.finishing:
            d = self.route(lambda c: c == self.goal)[0] if self.goal else None
            if d is None:
                self.returning = True
                self.get_logger().info('walking the path back')
                return
        self.path.append(self.here)
        self.here = self.target = step(self.here, d)
        self.head = self.travel = d
        self.visited.add(self.here)
        if self.here == self.goal:
            self.returning = True
        self.show()

    def follow(self):
        x, y, a = self.pose
        dx, dy = self.target[0] * self.cell - x, self.target[1] * self.cell - y
        dist = math.hypot(dx, dy)
        if dist < 0.02:
            self.target = None
            return self.send(0.0, 0.0)
        err = wrap(math.atan2(dy, dx) - a)
        if abs(err) > 0.3:
            return self.send(0.0, math.copysign(self.turn_speed, err))
        if dist < self.best - 0.003:
            self.best, self.stuck = dist, 0.0
        else:
            self.stuck += 0.05
        if self.stuck > self.stall_time:
            return self.jammed()
        self.send(min(self.speed, 1.5 * dist), 3.0 * err)

    def jammed(self):
        back = self.path[-1] if self.path else self.here
        d = (self.here[0] - back[0], self.here[1] - back[1])
        self.blocked |= {(back, d), (self.here, (-d[0], -d[1]))}
        self.visited.discard(self.here)
        self.here = self.target = self.path.pop() if self.path else back
        self.best, self.stuck = math.inf, 0.0
        self.get_logger().warn(f'stuck going {d}, edge written off')

    def tick(self):
        if self.done or self.started is None or self.pose is None:
            return
        if self.head is None:
            self.head = DIRS[round(self.pose[2] / (math.pi / 2)) % 4]
        if self.target is None:
            self.best, self.stuck = math.inf, 0.0
            self.plan()
        if self.target is not None and not self.done:
            self.follow()

    def send(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = limit(v, w, self.drive)
        self.cmd.publish(msg)

    def stop(self, why):
        self.done = True
        self.send(0.0, 0.0)
        self.get_logger().info(why)

    def show(self):
        m = Marker()
        m.header.frame_id = 'maze'
        m.type = Marker.CUBE
        m.pose.position.x, m.pose.position.y = self.here[0] * self.cell, self.here[1] * self.cell
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = self.cell * 0.8
        m.scale.z = 0.004
        m.color.g, m.color.b, m.color.a = 0.7, 1.0, 0.65
        self.marker.publish(m)


def main():
    run(Explorer)


if __name__ == '__main__':
    main()
