import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from common import config
from display import Screen, layout, render
from imu import Tilt, decode, quaternion
from motors import SpeedLoop, full_speed, motor_speeds, ramp, wheel_speeds
from tcs34725 import to_rgb

ROBOT = config('robot.json')


def test_wheels_split_and_flip():
    left, right = wheel_speeds(0.1, 1.0, ROBOT['drive'])
    assert math.isclose(left, 0.1 - 0.0759) and math.isclose(right, 0.1 + 0.0759)
    speeds = motor_speeds(-0.1, 0.2, ROBOT['motors'])
    assert speeds[3] == speeds[4] == 0.1 and speeds[1] == speeds[2] == 0.2


def test_wheels_cap_at_full_speed():
    left, right = wheel_speeds(5.0, 0.0, ROBOT['drive'])
    assert left == right == full_speed(ROBOT['drive'])


def test_ramp():
    assert ramp(0.0, 1.0, 0.1) == 0.1
    assert ramp(0.0, -0.05, 0.1) == -0.05


def test_speed_loop_pushes_a_slow_wheel():
    loop = SpeedLoop(ROBOT['drive'], ROBOT['motors'])
    first = loop.update(0.0, 0.1, 0)
    assert 0 < first <= 1.0
    later = loop.update(0.2, 0.1, 0)
    assert later > first
    assert loop.update(0.4, 0.0, 0) == 0.0
    assert loop.update(0.6, -0.1, 0) < 0


def test_imu_decode_and_level():
    raw = (8192).to_bytes(2, 'big', signed=True) + bytes(4) + bytes(2) + (655).to_bytes(2, 'big', signed=True) + bytes(4)
    accel, gyro = decode(raw)
    assert accel.tolist() == [1.0, 0.0, 0.0]
    assert math.isclose(gyro[0], math.radians(10.0))
    assert quaternion(0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)


def test_tilt_settles_on_gravity():
    tilt = Tilt(tau=0.5)
    tilt.update(np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.0)
    nose_up = np.array([-math.sin(0.2), 0.0, math.cos(0.2)])
    for _ in range(500):
        roll, pitch = tilt.update(np.zeros(3), nose_up, 0.01)
    assert abs(pitch - 0.2) < 0.01 and abs(roll) < 1e-6


def test_colour_drops_brightness():
    assert to_rgb(0, 0, 0) == (0, 0, 0)
    assert to_rgb(100, 100, 100) == to_rgb(1000, 1000, 1000)
    assert sum(to_rgb(30, 50, 20, level=300)) == 300


def test_oled_big_letters():
    assert layout('13')[1:] == (11, 9)
    assert layout('START GREEN')[0] == ['START', 'GREEN']
    pages = render('X')
    assert len(pages) == 8 and all(len(p) == 128 for p in pages)
    assert any(any(p) for p in pages)


def test_pages_take_turns():
    screen = Screen(page_time=3.5)
    screen.colour('orange', 0.0)
    assert screen.page(0.0) == 'ORANGE'
    screen.mark(13, 1.0)
    assert screen.page(1.0) == '13'
    assert screen.page(3.0) == '13'
    assert screen.page(4.6) == 'ORANGE'
    assert screen.page(8.2) == '13'
