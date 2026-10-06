import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from joy_drive import arcade, pull, shape


def test_shape_has_a_deadzone_and_keeps_sign():
    assert shape(0.05) == 0.0
    assert shape(1.0) == 1.0 and shape(-1.0) == -1.0
    assert 0 < shape(0.5) < 0.5


def test_triggers():
    assert pull(1.0) == 0.0 and pull(-1.0) == 1.0


def test_arcade_turbo_and_precision():
    assert arcade(1.0, 0.0, 0.0, 0.0, 0.15, 1.2, 2.0) == (0.15, 0.0)
    assert math.isclose(arcade(1.0, 0.0, 1.0, 0.0, 0.15, 1.2, 2.0)[0], 0.30)
    assert math.isclose(arcade(1.0, 0.0, 0.0, 1.0, 0.15, 1.2, 2.0)[0], 0.045)
