import math

CELL = 0.30
GAP = 0.30
LINE_W = 0.02

ROOM1 = [(i, j) for i in range(3) for j in range(3)]
BALL = (1, 1)
CP1, CP2 = (3, 1), (8, 1)
ROOM3 = [(i, j) for i in range(9, 12) for j in range(3)]
ENTRY3 = (9, 1)
FIELD = (4.0, 8.0, 0.5, 2.5)
LINE_X = (5.0, 6.0, 7.0)

SIDES = {'E': (1, 0), 'N': (0, 1), 'W': (-1, 0), 'S': (0, -1)}
HEADING = {'E': 0.0, 'N': math.pi / 2, 'W': math.pi, 'S': -math.pi / 2}
START_CELLS = [(i, -1) for i in range(3)] + [(i, 3) for i in range(3)] + [(-1, j) for j in range(3)]
FIN_CELLS = [(i, -1) for i in range(9, 12)] + [(i, 3) for i in range(9, 12)] + [(12, j) for j in range(3)]
COLOUR_STEP = {'orange': (1, 0), 'magenta': (-1, 0), 'yellow': (0, 1), 'cyan': (0, -1)}
RING = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2), (0, 1)]


def centre(cell):
    return (cell[0] + 0.5) * CELL, (cell[1] + 0.5) * CELL


def cell_of(x, y):
    return math.floor(x / CELL), math.floor(y / CELL)


def step(cell, d):
    return cell[0] + d[0], cell[1] + d[1]


def into_room(start):
    return min(max(start[0], 0), 2), min(max(start[1], 0), 2)


def edge(cell, side):
    x0, y0 = cell[0] * CELL, cell[1] * CELL
    x1, y1 = x0 + CELL, y0 + CELL
    return {'E': ((x1, y0), (x1, y1)), 'W': ((x0, y0), (x0, y1)),
            'N': ((x0, y1), (x1, y1)), 'S': ((x0, y0), (x1, y0))}[side]


def outline(start=None, fin=None):
    rects = [(0, 0, 3, 3), (3, 1, 4, 2), (FIELD[0], FIELD[2], FIELD[1], FIELD[3]), (8, 1, 9, 2), (9, 0, 12, 3)]
    rects += [(c[0], c[1], c[0] + 1, c[1] + 1) for c in (start, fin) if c is not None]

    def inside(a, b):
        x, y = (a + 0.5) / 2, (b + 0.5) / 2
        return any(r[0] <= x <= r[2] and r[1] <= y <= r[3] for r in rects)

    h = CELL / 2
    segs = []
    for a in range(-3, 28):
        run = None
        for b in range(-4, 10):
            on = inside(a - 1, b) != inside(a, b)
            if on and run is None:
                run = b
            elif not on and run is not None:
                segs.append(((a * h, run * h), (a * h, b * h)))
                run = None
    for b in range(-3, 10):
        run = None
        for a in range(-4, 28):
            on = inside(a, b - 1) != inside(a, b)
            if on and run is None:
                run = a
            elif not on and run is not None:
                segs.append(((run * h, b * h), (a * h, b * h)))
                run = None
    return segs


def fin_pockets():
    return [edge(c, s) for c in FIN_CELLS for s, d in SIDES.items() if step(c, d) not in ROOM3]
