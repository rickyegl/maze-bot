#!/usr/bin/env python3
import time
from collections import deque

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from common import run

GRAY = {'rgb8': (3, cv2.COLOR_RGB2GRAY), 'bgr8': (3, cv2.COLOR_BGR2GRAY),
        'rgba8': (4, cv2.COLOR_RGBA2GRAY), 'bgra8': (4, cv2.COLOR_BGRA2GRAY)}


YUV = {'yuv422_yuy2': 0, 'yuyv': 0, 'yuv422': 1, 'uyvy': 1}


def gray_of(msg):
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    enc = msg.encoding.lower()
    if enc in ('mono8', '8uc1'):
        return rows[:, :msg.width]
    if enc in YUV:
        return rows[:, YUV[enc]:2 * msg.width:2]
    if enc not in GRAY:
        return None
    n, code = GRAY[enc]
    return cv2.cvtColor(np.ascontiguousarray(rows[:, :n * msg.width].reshape(msg.height, msg.width, n)), code)


def shrink(gray, width):
    if width <= 0 or gray.shape[1] <= width:
        return gray
    f = width / gray.shape[1]
    return cv2.resize(gray, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)


def make_detector(name='DICT_4X4_50'):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, name))
    if hasattr(aruco, 'ArucoDetector'):
        find = aruco.ArucoDetector(dictionary, aruco.DetectorParameters()).detectMarkers
    else:
        params = aruco.DetectorParameters_create()
        find = lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)

    def detect(gray):
        corners, ids, _ = find(gray)
        if ids is None:
            return []
        sizes = [cv2.contourArea(c.reshape(4, 2).astype(np.float32)) for c in corners]
        return [int(i) for _, i in sorted(zip(sizes, ids.flatten()), reverse=True)]
    return detect


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        p = self.declare_parameter
        self.detect = make_detector(p('dictionary', 'DICT_4X4_50').value)
        self.period = 1.0 / p('rate', 3.0).value
        self.width = p('max_width', 640).value
        self.need = p('confirm', 2).value
        self.window = p('window', 30.0).value
        self.seen = deque()
        self.due = 0.0
        self.current = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Int32, 'aruco/id', latched)
        frames = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, frames)

    def on_image(self, msg):
        now = time.monotonic()
        if now < self.due:
            return
        self.due = max(self.due + self.period, now)
        gray = gray_of(msg)
        if gray is None:
            self.get_logger().warn(f'cannot read {msg.encoding} frames', once=True)
            return
        found = self.detect(shrink(gray, self.width))
        if not found:
            return
        self.seen.append((now, found[0]))
        while now - self.seen[0][0] > self.window:
            self.seen.popleft()
        if sum(i == found[0] for _, i in self.seen) < self.need:
            return
        if found[0] != self.current:
            self.current = found[0]
            self.get_logger().info(f'aruco: id {self.current}')
        self.pub.publish(Int32(data=self.current))


def main():
    run(ArucoDetector)


if __name__ == '__main__':
    main()
