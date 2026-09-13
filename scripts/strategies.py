import heapq
import random

from common import DIRS, step


def turn(d, quarters):
    return DIRS[(DIRS.index(d) + quarters) % 4]


def heading(nav):
    return nav.travel or nav.head or DIRS[0]


def allowed(nav, cell, d):
    nxt = step(cell, d)
    return nav.open_edge(cell, d) and nav.on_track(nxt) and nxt != nav.goal


def unexplored(nav):
    return lambda c: c not in nav.visited and c != nav.goal


class Strategy:
    name = ''

    def __init__(self, seed=0):
        self.seed = seed

    def note(self, nav):
        pass

    def choose(self, nav):
        raise NotImplementedError


class Flood(Strategy):
    name = 'flood'

    def choose(self, nav):
        return nav.route(unexplored(nav))[0]


STRATEGIES = {
    'flood': Flood,
}


def make(name, seed=0):
    if name not in STRATEGIES:
        raise ValueError(f'unknown strategy {name!r}, pick one of {", ".join(STRATEGIES)}')
    return STRATEGIES[name](seed)
