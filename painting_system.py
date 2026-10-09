"""Painting logic + surface coverage.

Each panel is a rectangle (origin, u, v, normal) divided into a CELL x CELL grid. Every control cycle,
if (a) the planned trajectory says 'spray on', (b) the real TCP is inside the spray distance window and
(c) the gun axis is within MAX_INCIDENCE of the panel normal, the spray ray is intersected with the
panel plane; every grid cell whose centre lies inside the spray footprint (circle of radius
t*tan(half_angle)) is marked as painted and a thin coloured tile is spawned on it.
Coverage = painted cells / total cells. It is a visual/coverage model, NOT a paint-thickness simulation.
"""
import math
import numpy as np
import pybullet as p
import config as C
from trajectories import Surface


class PaintingSystem:
    def __init__(self, surfaces):
        self.S = surfaces
        self.grid, self.tiles, self.vis = {}, {}, {}
        for n, s in surfaces.items():
            nu, nv = math.ceil(s.size[0] / C.CELL), math.ceil(s.size[1] / C.CELL)
            self.grid[n] = np.zeros((nu, nv), bool)
            self.tiles[n] = []
            h = C.CELL / 2 * 0.98
            self.vis[n] = p.createVisualShape(p.GEOM_BOX, halfExtents=[h, h, 0.002], rgbaColor=list(s.color))

    def reset(self):
        for n in self.S:
            for b in self.tiles[n]:
                p.removeBody(b)
            self.tiles[n] = []
            self.grid[n][:] = False

    def _spawn(self, s, i, j):
        pos = s.origin + s.u * (i + 0.5) * C.CELL + s.v * (j + 0.5) * C.CELL + s.n * 0.004
        R = np.column_stack([s.u, np.cross(s.n, s.u), s.n])
        quat = _mat_to_quat(R)
        self.tiles[s.name].append(p.createMultiBody(0, -1, self.vis[s.name], list(pos), list(quat)))

    def update(self, name, tcp, quat, spray_cmd):
        """Returns (active, spray_distance_m, incidence_deg)."""
        if not spray_cmd or name is None:
            return False
        s = self.S[name]
        d = np.array(p.getMatrixFromQuaternion(list(quat))).reshape(3, 3)[:, 2]
        den = float(np.dot(s.n, d))
        if den > -1e-6:
            return False
        t = float(np.dot(s.n, s.origin - tcp)) / den
        inc = math.acos(min(1.0, -den))
        if not (C.SPRAY_RANGE[0] <= t <= C.SPRAY_RANGE[1]) or inc > C.MAX_INCIDENCE:
            return False
        hit = tcp + d * t - s.origin
        a, b, r = float(np.dot(hit, s.u)), float(np.dot(hit, s.v)), t * math.tan(C.SPRAY_HALF_ANGLE)
        g = self.grid[name]
        nu, nv = g.shape
        if a < -r or a > s.size[0] + r or b < -r or b > s.size[1] + r:
            return False
        for i in range(max(0, int((a - r) // C.CELL)), min(nu - 1, int((a + r) // C.CELL)) + 1):
            for j in range(max(0, int((b - r) // C.CELL)), min(nv - 1, int((b + r) // C.CELL)) + 1):
                if not g[i, j] and ((i + .5) * C.CELL - a) ** 2 + ((j + .5) * C.CELL - b) ** 2 <= r * r:
                    g[i, j] = True
                    self._spawn(s, i, j)
        return True

    def progress(self, name):
        g = self.grid[name]
        return float(g.sum()) / g.size

    def total_progress(self):
        return sum(g.sum() for g in self.grid.values()) / sum(g.size for g in self.grid.values())


def _mat_to_quat(R):
    from trajectories import rot_to_quat
    return rot_to_quat(R)
