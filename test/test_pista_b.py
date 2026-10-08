import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import pista_b_track as track
from pista_b import EdgeVotes, next_unit, room_path, unit_colour
from pista_b_lines import LineMap
from pista_b_map import Likelihood, known_walls, locate, scan_points


def cast(origin, segs, n=360):
    ox, oy = origin
    out = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (ax, ay), (bx, by) in segs:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((ax - ox) * ey - (ay - oy) * ex) / den
            u = ((ax - ox) * dy - (ay - oy) * dx) / den
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        out.append(best)
    return out


def walls(start, open_side):
    return track.outline(start=start) + [track.edge(track.BALL, s) for s in track.SIDES if s != open_side]


def on_wall(segs, x, y):
    return any(min(a[0], b[0]) <= x <= max(a[0], b[0]) and min(a[1], b[1]) <= y <= max(a[1], b[1])
               for a, b in segs)


def test_outline_opens_for_the_start():
    assert on_wall(track.outline(), 0.45, 0.0)
    assert not on_wall(track.outline(start=(1, -1)), 0.45, 0.0)
    assert on_wall(track.outline(start=(1, -1)), 0.45, -0.3)


def test_votes_find_the_open_side():
    for side, spot in (('S', (0.45, 0.15)), ('E', (0.75, 0.75)), ('W', (0.15, 0.15))):
        votes = EdgeVotes()
        ranges = cast(spot, walls((1, -1), side))
        a = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)
        r = np.array(ranges)
        ok = r < 3
        pts = np.stack([spot[0] + r[ok] * np.cos(a[ok]), spot[1] + r[ok] * np.sin(a[ok])], 1)
        for _ in range(3):
            votes.add(spot, pts)
        assert votes.open_side() == side


def test_room_path_goes_round_the_ball():
    assert room_path((0, 0), (2, 1)) == [(0, 0), (1, 0), (2, 0), (2, 1)]
    assert room_path((1, -1), (1, 2), (1, -1))[:2] == [(1, -1), (1, 0)]
    assert track.BALL not in room_path((1, 0), (1, 2))


def test_colours_lead_to_fin():
    assert unit_colour([('orange', 2.85, 0.45)] * 6, (9, 1)) == 'orange'
    assert unit_colour([('orange', 2.85, 0.45)] * 3, (9, 1)) is None
    assert next_unit((9, 1), 'orange', []) == (10, 1)
    assert next_unit((9, 1), 'magenta', [(8, 1)]) is None
    assert next_unit((11, 2), 'yellow', []) == (11, 3)


def test_gap_from_the_floor():
    m = LineMap()
    x = 5 * track.CELL
    ys = np.arange(m.y0 + 0.005, m.y0 + 0.6, 0.01)
    gap = (ys > 0.30) & (ys < 0.60)
    px = np.full(len(ys), x)
    for _ in range(3):
        m.add(px, ys, ~gap, gap)
    lo, hi, seen = m.gap(x)
    assert abs(lo - 0.30) <= 0.011 and abs(hi - 0.60) <= 0.011 and seen > 0.9


def test_locate_the_start():
    truth = (0.75, -0.13, math.pi / 2)
    lidar = (truth[0], truth[1] + 0.066)
    ranges = cast(lidar, walls((2, -1), 'N'), 360)
    scan = SimpleNamespace(ranges=ranges, angle_min=-math.pi - truth[2], angle_increment=2 * math.pi / 360)
    pts = scan_points(scan, 0.066)
    fields = {c: Likelihood(known_walls(c)) for c in track.START_CELLS}
    cell, pose, score = locate(pts, fields)
    assert cell == (2, -1)
    assert abs(pose[0] - truth[0]) < 0.02 and abs(pose[1] - truth[1]) < 0.02
    assert abs(pose[2] - truth[2]) < 0.05
