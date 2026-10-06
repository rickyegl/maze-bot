import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from aruco_detector import gray_of, shrink
from base import blocked
from color_sensor import palette_for
from wall_map import deskew


def test_mask_wraps_through_zero():
    a = np.radians([0.0, 10.0, 90.0, 350.0])
    assert blocked(a, [340.0, 20.0]).tolist() == [True, True, False, True]
    assert blocked(a, [80.0, 100.0]).tolist() == [False, False, True, False]


def test_deskew_turns_early_beams_back():
    a = np.zeros(5)
    out = deskew(a, 1.0, 0.01)
    assert math.isclose(out[0], -0.04) and out[-1] == 0.0
    assert deskew(a, 0.0, 0.01) is a


def test_palette_lists_missing_colours():
    colors = {'pista_a': {'cyan': '#5CE1E6', 'goal_red': '#FF5757'},
              'calibration': {'robot': {'white': '#FFFFFF', 'cyan': '#00FFFF'}}}
    palette, missing = palette_for(colors, 'robot')
    assert palette == {'white': (255, 255, 255), 'cyan': (0, 255, 255)}
    assert missing == ['goal_red']


def test_yuyv_is_read_off_the_luma():
    y = np.arange(8, dtype=np.uint8).reshape(2, 4)
    data = np.stack([y, np.full_like(y, 128)], -1).reshape(2, 8)
    msg = SimpleNamespace(height=2, width=4, step=8, encoding='yuyv', data=data.tobytes())
    assert (gray_of(msg) == y).all()
    msg.encoding = 'nv12'
    assert gray_of(msg) is None


def test_shrink_keeps_small_frames():
    gray = np.zeros((480, 640), np.uint8)
    assert shrink(gray, 640) is gray
    assert shrink(np.zeros((960, 1280), np.uint8), 640).shape == (480, 640)
