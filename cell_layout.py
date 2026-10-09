"""Painting cell: floor, car stand, rail loop (robot 7th axis), safety fence and virtual safety walls."""
import numpy as np
import pybullet as p
import config as C


def _vbox(center, half, rgba):
    vs = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half), rgbaColor=list(rgba))
    return p.createMultiBody(0, -1, vs, list(center))


def build_cell():
    ids = []
    fx, fy = C.FENCE_HALF
    ids.append(_vbox((0, 0, -0.01), (fx + 2.0, fy + 2.0, 0.01), (0.56, 0.58, 0.61, 1)))       # floor
    ids.append(_vbox((0, 0, 0.004), (3.0, 1.55, 0.004), (0.40, 0.42, 0.46, 1)))               # car stand
    rx, ry = C.RING_HALF
    rail = (0.22, 0.24, 0.28, 1)
    for sy in (1, -1):
        ids.append(_vbox((0, sy * ry, 0.006), (rx + 0.2, 0.12, 0.006), rail))                 # rail loop
    for sx in (1, -1):
        ids.append(_vbox((sx * rx, 0, 0.006), (0.12, ry + 0.2, 0.006), rail))
    ids.append(_vbox((0, -(ry + 2.1) / 2, 0.006), (0.10, (ry - 2.1) / 2, 0.006), rail))      # ROOF spur
    yellow, black = (0.95, 0.78, 0.10, 1), (0.10, 0.10, 0.10, 1)
    for sy in (1, -1):                                                                        # fence
        for x in np.linspace(-fx, fx, 5):
            ids.append(_vbox((x, sy * fy, 0.6), (0.04, 0.04, 0.6), yellow))
        for z in (0.5, 1.0):
            ids.append(_vbox((0, sy * fy, z), (fx, 0.015, 0.03), black))
    for sx in (1, -1):
        for y in np.linspace(-fy, fy, 5):
            ids.append(_vbox((sx * fx, y, 0.6), (0.04, 0.04, 0.6), yellow))
        for z in (0.5, 1.0):
            ids.append(_vbox((sx * fx, 0, z), (0.015, fy, 0.03), black))
    # virtual safety walls (full height light-curtain style) = ONE compound collision body
    h = 1.75
    parts = [((0, fy, h), (fx, 0.05, h)), ((0, -fy, h), (fx, 0.05, h)),
             ((fx, 0, h), (0.05, fy, h)), ((-fx, 0, h), (0.05, fy, h))]
    col = p.createCollisionShapeArray(
        shapeTypes=[p.GEOM_BOX] * 4, radii=[0.0] * 4, halfExtents=[list(b) for _, b in parts],
        lengths=[0.0] * 4, collisionFramePositions=[list(c) for c, _ in parts],
        collisionFrameOrientations=[[0, 0, 0, 1]] * 4)
    ghost = p.createVisualShape(p.GEOM_BOX, halfExtents=[1e-4] * 3, rgbaColor=[0, 0, 0, 0])   # hide default visual
    fence = p.createMultiBody(0, col, ghost, [0, 0, 0])
    return dict(visual=ids, collision=fence)


def add_labels():
    """Static 3-D text: warning sign + station names (a handful of debug items only)."""
    fx, fy = C.FENCE_HALF
    p.addUserDebugText("DANGER - ROBOT PAINTING CELL", [-1.6, fy + 0.05, 1.45], [0.8, 0.1, 0.0], 1.4)
    p.addUserDebugText("DANGER - ROBOT PAINTING CELL", [1.6, -fy - 0.05, 1.45], [0.8, 0.1, 0.0], 1.4)
    for name, (x, y) in C.STATIONS.items():
        p.addUserDebugText(f"[{name}]", [x, y, 0.05], [0.1, 0.1, 0.1], 1.0)
