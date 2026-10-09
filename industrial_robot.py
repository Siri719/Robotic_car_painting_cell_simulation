"""Original 6-DOF industrial painting robot, generated as a URDF at start-up (no external assets).

Kinematic chain (all joints revolute, zero pose = arm straight up):
  J1 turret yaw (z) -> J2 shoulder pitch (y) -> J3 elbow pitch (y) -> J4 forearm roll (z)
  -> J5 wrist pitch (y) -> J6 tool roll (z) -> fixed spray gun -> fixed TCP (nozzle tip).
Positive pitch rotates +z toward +x (the arm leans forward).

TCP = Tool Centre Point: the nozzle tip. Its z axis is the spray direction. IK is solved for the
pose (position + orientation) of the TCP link, so the gun always points at the panel.
"""
import math
import os
import numpy as np
import pybullet as p
import config as C

_mat = [0]


def _shape(tag, kind, size, xyz, rpy, rgba):
    if kind == "box":
        g = f'<box size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"/>'
    elif kind == "cyl":
        g = f'<cylinder radius="{size[0]:.4f}" length="{size[1]:.4f}"/>'
    else:
        g = f'<sphere radius="{size[0]:.4f}"/>'
    mat = ""
    if tag == "visual":
        _mat[0] += 1
        mat = f'<material name="m{_mat[0]}"><color rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"/></material>'
    return (f'<{tag}><origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>'
            f'<geometry>{g}</geometry>{mat}</{tag}>')


def _link(name, mass, dims, com, shapes):
    dx, dy, dz = dims
    ixx, iyy, izz = mass / 12 * (dy * dy + dz * dz), mass / 12 * (dx * dx + dz * dz), mass / 12 * (dx * dx + dy * dy)
    s = (f'<link name="{name}"><inertial><origin xyz="{com[0]} {com[1]} {com[2]}"/><mass value="{mass}"/>'
         f'<inertia ixx="{ixx}" iyy="{iyy}" izz="{izz}" ixy="0" ixz="0" iyz="0"/></inertial>')
    for kind, size, xyz, rpy, rgba, collide in shapes:
        s += _shape("visual", kind, size, xyz, rpy, rgba)
        if collide:
            s += _shape("collision", kind, size, xyz, rpy, None)
    return s + "</link>"


def _joint(name, parent, child, xyz, axis=None, j=None):
    if axis is None:
        return (f'<joint name="{name}" type="fixed"><parent link="{parent}"/><child link="{child}"/>'
                f'<origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}"/></joint>')
    return (f'<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}"/><axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>'
            f'<limit lower="{C.J_LOWER[j]}" upper="{C.J_UPPER[j]}" effort="{C.J_FORCE[j]}" velocity="{C.J_VEL[j]}"/>'
            f'<dynamics damping="2.0" friction="0.5"/></joint>')


def write_urdf(path):
    A = C.ARM
    OR, DK, YE, SI, BR = (0.95, 0.42, 0.05, 1), (0.16, 0.17, 0.19, 1), (0.92, 0.76, 0.10, 1), (0.75, 0.77, 0.80, 1), (0.85, 0.65, 0.20, 1)
    Y = (math.pi / 2, 0, 0)
    Z = (0, 0, 0)
    x = ['<?xml version="1.0"?><robot name="painter6">']
    x.append(_link("base", 300, (1, 1, 0.5), (0, 0, 0.25), [
        ("cyl", (0.38, A["ped_h"]), (0, 0, 0.25), Z, DK, True),
        ("cyl", (0.50, 0.04), (0, 0, 0.02), Z, YE, True)]))
    x.append(_link("link1", 150, (0.6, 0.6, 0.9), (0.1, 0, 0.4), [
        ("cyl", (0.30, 0.55), (0, 0, 0.275), Z, OR, True),
        ("cyl", (0.22, 0.48), (A["shoulder"][0], 0, A["shoulder"][2]), Y, DK, True)]))
    x.append(_link("link2", 80, (0.3, 0.3, A["upper"]), (0, 0, A["upper"] / 2), [
        ("box", (0.28, 0.30, A["upper"]), (0, 0, A["upper"] / 2), Z, OR, True),
        ("cyl", (0.19, 0.42), (0, 0, A["upper"]), Y, DK, True)]))
    x.append(_link("link3", 40, (0.25, 0.25, A["fore"]), (0, 0, A["fore"] / 2), [
        ("box", (0.20, 0.24, A["fore"]), (0, 0, A["fore"] / 2), Z, OR, True)]))
    x.append(_link("link4", 12, (0.22, 0.22, A["w1"]), (0, 0, A["w1"] / 2), [
        ("cyl", (0.11, A["w1"]), (0, 0, A["w1"] / 2), Z, OR, True)]))
    x.append(_link("link5", 8, (0.2, 0.26, A["w2"]), (0, 0, A["w2"] / 2), [
        ("cyl", (0.10, 0.26), (0, 0, 0), Y, DK, True),
        ("cyl", (0.08, A["w2"]), (0, 0, A["w2"] / 2), Z, OR, True)]))
    x.append(_link("link6", 3, (0.12, 0.12, 0.04), (0, 0, 0.02), [
        ("cyl", (0.06, 0.04), (0, 0, 0.02), Z, SI, True)]))
    x.append(_link("tool", 2, (0.1, 0.12, 0.40), (0, 0, 0.2), [          # spray gun
        ("cyl", (0.05, 0.05), (0, 0, 0.025), Z, DK, True),
        ("box", (0.07, 0.10, 0.20), (0, 0, 0.15), Z, (0.25, 0.26, 0.28, 1), True),
        ("cyl", (0.035, 0.10), (0, 0.085, 0.17), Z, SI, False),
        ("cyl", (0.018, 0.15), (0, 0, 0.325), Z, BR, True)]))
    x.append(_link("tcp", 0.01, (0.01, 0.01, 0.01), (0, 0, 0), [
        ("sph", (0.004,), (0, 0, 0), Z, (1, 0, 0, 1), True)]))
    x.append(_joint("j1", "base", "link1", (0, 0, A["ped_h"]), (0, 0, 1), 0))
    x.append(_joint("j2", "link1", "link2", A["shoulder"], (0, 1, 0), 1))
    x.append(_joint("j3", "link2", "link3", (0, 0, A["upper"]), (0, 1, 0), 2))
    x.append(_joint("j4", "link3", "link4", (0, 0, A["fore"]), (0, 0, 1), 3))
    x.append(_joint("j5", "link4", "link5", (0, 0, A["w1"]), (0, 1, 0), 4))
    x.append(_joint("j6", "link5", "link6", (0, 0, A["w2"]), (0, 0, 1), 5))
    x.append(_joint("tool_fix", "link6", "tool", (0, 0, 0)))
    x.append(_joint("tcp_fix", "tool", "tcp", (0, 0, C.TOOL_LEN)))
    x.append("</robot>")
    with open(path, "w") as f:
        f.write("".join(x))


def quat_angle(q1, q2):
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q1, q2)))))


class IndustrialRobot:
    ARM = [0, 1, 2, 3, 4, 5]
    TOOL, TCP = 6, 7
    LINK_NAMES = {-1: "pedestal", 0: "turret", 1: "upper arm", 2: "forearm", 3: "wrist1",
                  4: "wrist2", 5: "flange", 6: "SPRAY GUN", 7: "TCP"}

    def __init__(self, asset_dir):
        path = os.path.join(asset_dir, "painter6.urdf")
        write_urdf(path)
        self.id = p.loadURDF(path, [0, 0, 0], useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.base = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ state
    def set_base(self, x, y, yaw):
        """Carriage (7th axis): kinematic translation/rotation of the whole robot on its rail."""
        self.base = (x, y, yaw)
        p.resetBasePositionAndOrientation(self.id, [x, y, 0], p.getQuaternionFromEuler([0, 0, yaw]))

    def reset_q(self, q):
        for j, v in zip(self.ARM, q):
            p.resetJointState(self.id, j, float(v), 0.0)

    def q(self):
        return np.array([s[0] for s in p.getJointStates(self.id, self.ARM)])

    def command(self, q, qd):
        """Joint position control with velocity feed-forward (smooth, no teleporting)."""
        p.setJointMotorControlArray(self.id, self.ARM, p.POSITION_CONTROL,
                                    targetPositions=[float(v) for v in q],
                                    targetVelocities=[float(v) for v in qd],
                                    forces=C.J_FORCE.tolist(),
                                    positionGains=[0.25] * 6, velocityGains=[1.0] * 6)

    def tcp_pose(self):
        s = p.getLinkState(self.id, self.TCP, computeForwardKinematics=1)
        return np.array(s[4]), np.array(s[5])         # link-frame position / orientation (world)

    def snapshot(self):
        return self.base, self.q()

    def restore(self, snap):
        self.set_base(*snap[0])
        self.reset_q(snap[1])

    # ------------------------------------------------------------------ inverse kinematics
    def pose_error(self, pos, quat):
        cp, cq = self.tcp_pose()
        return float(np.linalg.norm(cp - pos)), quat_angle(cq, quat)

    def solve_ik(self, pos, quat, seed, tries=5):
        """Damped-least-squares IK (PyBullet) on the TCP pose, iterated until FK confirms the result.
        Returns (q, ok, pos_err, ang_err). Joint limits are passed to the solver."""
        q = np.asarray(seed, float)
        lo, hi = C.J_LOWER.tolist(), C.J_UPPER.tolist()
        rng = (C.J_UPPER - C.J_LOWER).tolist()
        pe = ae = 1e9
        for _ in range(tries):
            self.reset_q(q)
            sol = p.calculateInverseKinematics(self.id, self.TCP, list(pos), list(quat), lowerLimits=lo,
                                               upperLimits=hi, jointRanges=rng, restPoses=list(q),
                                               maxNumIterations=200, residualThreshold=1e-6)
            q = np.clip(np.array(sol[:6]), C.J_LOWER, C.J_UPPER)
            self.reset_q(q)
            pe, ae = self.pose_error(pos, quat)
            if pe < C.IK_TOL_POS and ae < C.IK_TOL_ANG:
                return q, True, pe, ae
        return q, False, pe, ae

    def multi_seed_ik(self, pos, quat, ref=None):
        """Try several elbow/wrist seeds (IK is non-unique); keep the sound solution nearest to `ref`
        (or nearest to a comfortable posture). Rejects near-singular wrists (|J5| small)."""
        bx, by, yaw = self.base
        d = np.array(pos[:2]) - np.array([bx, by])
        c, s = math.cos(-yaw), math.sin(-yaw)
        j1 = math.atan2(s * d[0] + c * d[1], c * d[0] - s * d[1])
        pref = np.array([j1, 0.8, 1.2, 0.0, 0.8, 0.0]) if ref is None else np.asarray(ref)
        best, best_c = None, 1e9
        for a2 in (0.4, 0.9, 1.4):
            for a3 in (0.5, 1.2, 1.9):
                for a5 in (-0.8, 0.8, 1.3):
                    seed = np.array([j1, a2, a3, 0.0, a5, 0.0])
                    q, ok, pe, ae = self.solve_ik(pos, quat, seed, tries=3)
                    if not ok or abs(q[4]) < 0.15:
                        continue
                    cost = float(np.sum((q - pref) ** 2))
                    if cost < best_c:
                        best, best_c = (q, ok, pe, ae), cost
        if best is None:
            return seed, False, pe, ae
        return best
