"""Spray visualisation (cheap): one translucent cone body, one TCP marker, one replaceable direction line,
plus capped debug-line paths (planned = thin, executed = thick).  No particles."""
import math
import os
import numpy as np
import pybullet as p
import config as C


def _write_cone_obj(path, half_angle, n=18):
    r = math.tan(half_angle)
    L = ["o cone", "v 0 0 0"]
    for i in range(n):
        a = 2 * math.pi * i / n
        L.append(f"v {r * math.cos(a):.5f} {r * math.sin(a):.5f} 1")
    L.append("v 0 0 1")
    for i in range(n):
        a, b = 2 + i, 2 + (i + 1) % n
        L.append(f"f 1 {a} {b}")
        L.append(f"f {n + 2} {b} {a}")
    with open(path, "w") as f:
        f.write("\n".join(L))


class SprayVisual:
    LENGTH = 0.32

    def __init__(self, asset_dir):
        path = os.path.join(asset_dir, "spray_cone.obj")
        _write_cone_obj(path, C.SPRAY_HALF_ANGLE)
        vs = p.createVisualShape(p.GEOM_MESH, fileName=path, meshScale=[self.LENGTH] * 3,
                                 rgbaColor=[0.35, 0.75, 1.0, 0.35])
        self.cone = p.createMultiBody(0, -1, vs, [0, 0, -5])
        p.changeVisualShape(self.cone, -1, rgbaColor=[0.35, 0.75, 1.0, 0.35])
        ms = p.createVisualShape(p.GEOM_SPHERE, radius=0.022, rgbaColor=[1, 0.1, 0.1, 1])
        self.marker = p.createMultiBody(0, -1, ms, [0, 0, -5])
        self.line = None

    def update(self, pos, quat, spraying):
        p.resetBasePositionAndOrientation(self.marker, list(pos), [0, 0, 0, 1])
        if spraying:
            p.resetBasePositionAndOrientation(self.cone, list(pos), list(quat))
        else:
            p.resetBasePositionAndOrientation(self.cone, [0, 0, -5], [0, 0, 0, 1])
        d = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)[:, 2]
        kw = dict(lineColorRGB=[1, 0.2, 0.2], lineWidth=2, lifeTime=0)
        if self.line is None:
            self.line = p.addUserDebugLine(list(pos), list(pos + 0.6 * d), **kw)
        else:
            p.addUserDebugLine(list(pos), list(pos + 0.6 * d), replaceItemUniqueId=self.line, **kw)

    def hide(self):
        p.resetBasePositionAndOrientation(self.cone, [0, 0, -5], [0, 0, 0, 1])


class PathVisualizer:
    MAX_EXEC = 700

    def __init__(self):
        self.planned, self.executed, self.last = [], [], None

    def show_planned(self, legs):
        self.clear_planned()
        for a, b, spray, _ in legs:
            col, w = ([0.0, 0.55, 1.0], 1.5) if spray else ([0.55, 0.55, 0.55], 1.0)
            self.planned.append(p.addUserDebugLine(list(a), list(b), col, w))

    def add_executed(self, pt):
        if self.last is None or np.linalg.norm(pt - self.last) > 0.10:
            if self.last is not None:
                self.executed.append(p.addUserDebugLine(list(self.last), list(pt), [1.0, 0.55, 0.0], 4))
                if len(self.executed) > self.MAX_EXEC:
                    p.removeUserDebugItem(self.executed.pop(0))
            self.last = pt.copy()

    def clear_planned(self):
        for i in self.planned:
            p.removeUserDebugItem(i)
        self.planned = []

    def clear_executed(self):
        for i in self.executed:
            p.removeUserDebugItem(i)
        self.executed, self.last = [], None

    def clear(self):
        self.clear_planned()
        self.clear_executed()
