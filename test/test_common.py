import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from common import config, limit, wrap

DRIVE = config('robot.json')['drive']


def test_wrap():
    assert wrap(3 * math.pi) == -math.pi
    assert abs(wrap(-0.5)) == 0.5


def test_limit_keeps_curvature():
    v, w = limit(1.0, 4.0, DRIVE)
    assert v <= DRIVE['max_linear'] and w <= DRIVE['max_angular']
    assert math.isclose(w / v, 4.0)


def test_limit_rejects_nan():
    assert limit(math.nan, 0.0, DRIVE) == (0.0, 0.0)
