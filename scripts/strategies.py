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


class Dijkstra(Strategy):
    name = 'dijkstra'
    CELL, TURN, BACK = 2.0, 1.3, 1.0

    def choose(self, nav):
        start = (nav.here, heading(nav))
        best = {start: 0.0}
        todo = [(0.0, 0, nav.here, start[1], None)]
        n = 0
        while todo:
            cost, _, cell, h, first = heapq.heappop(todo)
            if cost > best.get((cell, h), float('inf')):
                continue
            if cell != nav.here and unexplored(nav)(cell):
                return first
            for d in DIRS:
                if not allowed(nav, cell, d):
                    continue
                extra = 0.0 if d == h else self.BACK if d == turn(h, 2) else self.TURN
                key, c = (step(cell, d), d), cost + self.CELL + extra
                if c < best.get(key, float('inf')):
                    best[key] = c
                    n += 1
                    heapq.heappush(todo, (c, n, key[0], d, first or d))
        return None


class DepthFirst(Strategy):
    name = 'dfs'

    def __init__(self, seed=0):
        super().__init__(seed)
        self.stack = []

    def note(self, nav):
        if nav.here in self.stack:
            del self.stack[self.stack.index(nav.here) + 1:]
        else:
            self.stack.append(nav.here)

    def fresh(self, nav, cell, h):
        return [d for d in (h, turn(h, -1), turn(h, 1), turn(h, 2))
                if allowed(nav, cell, d) and step(cell, d) not in nav.visited]

    def choose(self, nav):
        ways = self.fresh(nav, nav.here, heading(nav))
        if ways:
            return ways[0]
        for cell in reversed(self.stack):
            if cell != nav.here and self.fresh(nav, cell, DIRS[0]):
                first = nav.route(lambda c, target=cell: c == target)[0]
                if first is not None:
                    return first
        return Flood().choose(nav)


class WallFollower(Strategy):
    def __init__(self, seed=0, hand=-1):
        super().__init__(seed)
        self.hand = hand
        self.name = 'right_hand' if hand < 0 else 'left_hand'
        self.seen = {}
        self.crossed = 0

    def note(self, nav):
        if len(nav.visited) != self.crossed:
            self.seen.clear()
            self.crossed = len(nav.visited)
        state = (nav.here, heading(nav))
        self.seen[state] = self.seen.get(state, 0) + 1

    def choose(self, nav):
        h = heading(nav)
        if self.seen.get((nav.here, h), 0) > 1 or nav.route(unexplored(nav))[0] is None:
            return None
        for q in (self.hand, 0, -self.hand, 2):
            if allowed(nav, nav.here, turn(h, q)):
                return turn(h, q)
        return None


class RandomMouse(Strategy):
    name = 'random'

    def __init__(self, seed=0):
        super().__init__(seed)
        self.rng = random.Random(seed)

    def choose(self, nav):
        if nav.route(unexplored(nav))[0] is None:
            return None
        h = heading(nav)
        ways = [d for d in DIRS if allowed(nav, nav.here, d)]
        onward = [d for d in ways if d != turn(h, 2)] or ways
        return self.rng.choice(onward) if onward else None


STRATEGIES = {
    'flood': Flood,
    'dijkstra': Dijkstra,
    'dfs': DepthFirst,
    'right_hand': lambda seed=0: WallFollower(seed, -1),
    'left_hand': lambda seed=0: WallFollower(seed, 1),
    'random': RandomMouse,
}


def make(name, seed=0):
    if name not in STRATEGIES:
        raise ValueError(f'unknown strategy {name!r}, pick one of {", ".join(STRATEGIES)}')
    return STRATEGIES[name](seed)
