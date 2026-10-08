import math

import numpy as np

import pista_b_track as track

RES = 0.01


def classify(rgb):
    c = rgb.astype(np.int16)
    hi, lo = c.max(-1), c.min(-1)
    white = (lo >= 150) & (hi - lo <= 60)
    green = (c[..., 1] >= 80) & (c[..., 1] - c[..., 0] >= 50) & (c[..., 1] - c[..., 2] >= 20)
    return white, green


def intrinsics(k, width, height, fov=1.047):
    if k[0] > 0:
        return k[0], k[4], k[2], k[5]
    f = width / 2 / math.tan(fov / 2)
    return f, f, width / 2, height / 2


def floor_rays(fx, fy, cx, cy, width, height, stride=4):
    V, U = np.meshgrid(np.arange(int(cy) + 8, height, stride), np.arange(0, width, stride), indexing='ij')
    fwd = fy / (V - cy)
    return V, U, fwd, -(U - cx) / fx * fwd


def to_track(fwd, left, height, cam_x, pose):
    bx, by = cam_x - fwd * height, -left * height
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * bx - s * by, y + s * bx + c * by


class LineMap:
    def __init__(self):
        c = track.CELL
        self.x0, self.y0 = track.FIELD[0] * c, track.FIELD[2] * c
        self.nx = round((track.FIELD[1] - track.FIELD[0]) * c / RES)
        self.ny = round((track.FIELD[3] - track.FIELD[2]) * c / RES)
        self.white = np.zeros((self.nx, self.ny), int)
        self.green = np.zeros((self.nx, self.ny), int)

    def add(self, px, py, white, green):
        ix, iy = ((px - self.x0) / RES).astype(int), ((py - self.y0) / RES).astype(int)
        keep = (ix > 0) & (ix < self.nx - 1) & (iy > 0) & (iy < self.ny - 1) & (white | green)
        np.add.at(self.white, (ix[keep], iy[keep]), white[keep])
        np.add.at(self.green, (ix[keep], iy[keep]), green[keep])

    def gap(self, x, half=0.015):
        lo, hi = max(int((x - half - self.x0) / RES), 0), int((x + half - self.x0) / RES) + 1
        w, g = self.white[lo:hi].sum(0), self.green[lo:hi].sum(0)
        n = round(track.GAP / RES)
        cost = [w[s:s + n].sum() + g[:s].sum() + g[s + n:].sum() for s in range(self.ny - n + 1)]
        s = int(np.argmin(cost))
        return self.y0 + s * RES, self.y0 + (s + n) * RES, float(((w + g) > 0).mean())
