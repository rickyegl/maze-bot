import sys
from collections import deque
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

import strategies
from common import DIRS, step

MAZE = [
    '+---+---+---+',
    '| .   . | . |',
    '+   +   +   +',
    '| . | .   . |',
    '+   +---+   +',
    '| .   .   . |',
    '+---+---+---+',
]


class Grid:
    def __init__(self, lines):
        self.lines = lines
        self.rows, self.cols = len(lines) // 2, (len(lines[0]) - 1) // 4
        self.here, self.head, self.travel, self.goal = (0, 0), DIRS[0], None, None
        self.visited = {self.here}

    def open_edge(self, cell, d):
        col, row = cell[0], self.rows - 1 - cell[1]
        if d[0]:
            return self.lines[2 * row + 1][4 * (col + (d[0] > 0))] != '|'
        return self.lines[2 * (row + (d[1] < 0))][4 * col + 1:4 * col + 4] != '---'

    def on_track(self, cell):
        return 0 <= cell[0] < self.cols and 0 <= cell[1] < self.rows

    def route(self, wanted):
        queue, seen = deque([(self.here, None, 0)]), {self.here}
        while queue:
            cell, first, n = queue.popleft()
            if cell != self.here and wanted(cell):
                return first, n
            for d in DIRS:
                nxt = step(cell, d)
                if nxt not in seen and self.on_track(nxt) and self.open_edge(cell, d):
                    seen.add(nxt)
                    queue.append((nxt, first or d, n + 1))
        return None, None


def explore(name, limit=200):
    grid, strategy = Grid(MAZE), strategies.make(name, seed=3)
    for _ in range(limit):
        strategy.note(grid)
        d = strategy.choose(grid)
        if d is None:
            break
        assert grid.open_edge(grid.here, d)
        grid.here, grid.travel = step(grid.here, d), d
        grid.visited.add(grid.here)
    return grid.visited


@pytest.mark.parametrize('name', ['flood', 'dijkstra', 'dfs', 'right_hand', 'left_hand', 'random'])
def test_covers_every_cell(name):
    assert len(explore(name)) == 9


def test_unknown_strategy():
    with pytest.raises(ValueError):
        strategies.make('teleport')
