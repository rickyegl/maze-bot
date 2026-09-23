import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from display import Screen, fit


def test_fit_pads_and_trims():
    assert fit('Color', 'cyan') == 'Color cyan      '
    assert fit('Color', 'a very long colour') == 'a very long colo'


def test_colour_is_held_over_background():
    screen = Screen(hold=3.0)
    screen.colour('cyan', 0.0)
    screen.colour('white', 1.0)
    assert screen.rows(2.0)[0].strip() == 'Color cyan'
    assert screen.rows(3.5)[0].strip() == 'Color white'


def test_marker_row():
    screen = Screen()
    assert screen.rows(0.0)[1].strip() == 'ArUco --'
    screen.marker = 13
    assert screen.rows(0.0)[1].strip() == 'ArUco 13'
