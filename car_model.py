"""Procedural passenger car (4.8 x 1.9 x 1.6 m) built only from primitives + one tiny generated mesh.

VISUAL and COLLISION geometry are separate: the many small visual parts are collision-free bodies,
while ONE invisible compound body (a few boxes + 4 wheel cylinders) is used for collision checking.
That keeps contact queries cheap (one body to test) and is conservative (the cabin hull is a box).
"""
import os
import numpy as np
import pybullet as p

PRIMER = (0.74, 0.76, 0.79, 1.0)
GLASS = (0.06, 0.10, 0.16, 1.0)
DARK = (0.08, 0.08, 0.09, 1.0)
TYRE = (0.05, 0.05, 0.05, 1.0)
HUB = (0.65, 0.66, 0.70, 1.0)


def _vbox(center, half, rgba, rpy=(0, 0, 0)):
    vs = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half), rgbaColor=list(rgba))
    return p.createMultiBody(0, -1, vs, list(center), p.getQuaternionFromEuler(rpy))


def _vcyl(center, radius, length, rgba, rpy):
    vs = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=list(rgba))
    return p.createMultiBody(0, -1, vs, list(center), p.getQuaternionFromEuler(rpy))


def _write_cabin_obj(path):
    """Trapezoid prism (windshield / rear-window slopes) in the x-z plane, extruded over y = +-0.8."""
    prof = [(-1.25, 0.95), (0.90, 0.95), (0.55, 1.60), (-0.95, 1.60)]
    w = 0.80
    v = [(x, -w, z) for x, z in prof] + [(x, w, z) for x, z in prof]
    quads = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    lines = ["o cabin"] + [f"v {x} {y} {z}" for x, y, z in v]
    nid = 0
    for qd in quads:
        a, b, c = (np.array(v[i]) for i in qd[:3])
        n = np.cross(b - a, c - a); n /= np.linalg.norm(n)
        lines.append(f"vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}")
        nid += 1
        i0, i1, i2, i3 = (i + 1 for i in qd)
        lines.append(f"f {i0}//{nid} {i1}//{nid} {i2}//{nid}")
        lines.append(f"f {i0}//{nid} {i2}//{nid} {i3}//{nid}")
    with open(path, "w") as f:
        f.write("\n".join(lines))


def build_car(asset_dir):
    names = []
    # ---- visual parts -------------------------------------------------------------------------
    names.append(_vbox((0, 0, 0.635), (2.30, 0.95, 0.315), PRIMER))              # lower body
    names.append(_vbox((0, 0, 0.36), (2.25, 0.955, 0.04), DARK))                  # sill
    for sx in (1, -1):
        names.append(_vbox((sx * 2.35, 0, 0.46), (0.05, 0.90, 0.16), DARK))       # bumpers
    names.append(_vbox((2.305, 0, 0.62), (0.012, 0.45, 0.09), DARK))              # grille
    for sy in (1, -1):
        names.append(_vbox((2.305, sy * 0.68, 0.80), (0.012, 0.18, 0.06), (1.0, 0.95, 0.70, 1)))   # headlights
        names.append(_vbox((-2.305, sy * 0.68, 0.80), (0.012, 0.18, 0.06), (0.85, 0.05, 0.05, 1)))  # tail lights
    cabin_obj = os.path.join(asset_dir, "cabin.obj")
    _write_cabin_obj(cabin_obj)
    vs = p.createVisualShape(p.GEOM_MESH, fileName=cabin_obj, rgbaColor=list(GLASS))
    cab = p.createMultiBody(0, -1, vs, [0, 0, 0])                                 # glass greenhouse
    p.changeVisualShape(cab, -1, rgbaColor=list(GLASS))
    names.append(cab)
    names.append(_vbox((-0.20, 0, 1.592), (0.75, 0.80, 0.008), PRIMER))           # roof panel (top z = 1.6)
    for sx in (1.45, -1.45):
        for sy in (0.80, -0.80):                                                  # wheels
            names.append(_vcyl((sx, sy, 0.34), 0.34, 0.26, TYRE, (np.pi / 2, 0, 0)))
            names.append(_vcyl((sx, sy * 1.17, 0.34), 0.17, 0.02, HUB, (np.pi / 2, 0, 0)))
    # ---- ONE compound collision body (invisible) ----------------------------------------------
    boxes = [((0, 0, 0.635), (2.30, 0.95, 0.315)),
             ((2.35, 0, 0.46), (0.05, 0.90, 0.16)), ((-2.35, 0, 0.46), (0.05, 0.90, 0.16)),
             ((-0.175, 0, 1.275), (1.075, 0.82, 0.325))]                          # cabin hull
    types, half, rad, ln, pos, orn = [], [], [], [], [], []
    for c, h in boxes:
        types.append(p.GEOM_BOX); half.append(list(h)); rad.append(0.0); ln.append(0.0)
        pos.append(list(c)); orn.append([0, 0, 0, 1])
    for sx in (1.45, -1.45):
        for sy in (0.80, -0.80):
            types.append(p.GEOM_CYLINDER); half.append([0, 0, 0]); rad.append(0.34); ln.append(0.26)
            pos.append([sx, sy, 0.34]); orn.append(p.getQuaternionFromEuler([np.pi / 2, 0, 0]))
    col = p.createCollisionShapeArray(shapeTypes=types, radii=rad, halfExtents=half, lengths=ln,
                                      collisionFramePositions=pos, collisionFrameOrientations=orn)
    ghost = p.createVisualShape(p.GEOM_BOX, halfExtents=[1e-4] * 3, rgbaColor=[0, 0, 0, 0])   # hide default visual
    body = p.createMultiBody(0, col, ghost, [0, 0, 0])
    return dict(visual=names, collision=body)
