"""Painting trajectories, reachability planning and motion segments.

PIPELINE per panel
 1. generate_cartesian(): boustrophedon (zig-zag) passes over the panel -> TCP positions + a CONSTANT tool
    orientation (gun axis = -normal, optionally tilted). Spray flag per segment (on in passes, off in
    transitions / lead-in / lead-out).
 2. Planner.plan_surface(): inverse kinematics at every node (seeded from the previous node for branch
    continuity), FK verification, joint-jump check, clearance check -> REACHABILITY REPORT.
 3. time parameterisation (speed + accel + joint-velocity limits) -> cubic Hermite spline in joint space
    (C1 continuous, zero velocity at pass ends) -> sampled at 240 Hz by the executor.
"""
import math
import numpy as np
import config as C


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s])


def tool_quat(d, x_hint):
    """Tool frame: z = spray direction d, x = pass direction (orthogonalised), y = z cross x."""
    z = unit(d)
    x = unit(np.asarray(x_hint) - np.dot(x_hint, z) * z)
    return rot_to_quat(np.column_stack([x, np.cross(z, x), z]))


class Surface:
    def __init__(self, name, cfg):
        self.name, self.station = name, cfg["station"]
        self.origin, self.u, self.v, self.n = (np.array(cfg[k], float) for k in ("origin", "u", "v", "normal"))
        self.size, self.color = cfg["size"], cfg["color"]
        th = math.radians(cfg["tilt_deg"])
        w = np.array(cfg["tilt_dir"], float)
        w = unit(w - np.dot(w, self.n) * self.n)
        self.d = unit(-self.n * math.cos(th) + w * math.sin(th))       # spray direction


def load_surfaces():
    return {n: Surface(n, c) for n, c in C.SURFACES.items()}


def station_pose(name):
    x, y = C.STATIONS[name]
    return (x, y, math.atan2(-y, -x))                                    # base x axis faces the car centre


# ===================================================================== Cartesian generation
def _sample_leg(a, b, step):
    n = max(1, math.ceil(np.linalg.norm(b - a) / step))
    return [a + (b - a) * k / n for k in range(n)]


def generate_legs(surf):
    """List of (A, B, spray, pass_index) TCP legs: lead-in, zig-zag passes, transitions, lead-out."""
    su, sv = surf.size
    m = C.EDGE_INSET
    npass = max(2, math.ceil((sv - 2 * m) / C.PASS_SPACING) + 1)
    vs = np.linspace(m, sv - m, npass)
    tcp = lambda a, b: surf.origin + surf.u * a + surf.v * b - surf.d * C.STANDOFF
    legs = []
    first = tcp(m, vs[0])
    legs.append((first - surf.d * C.APPROACH, first, False, 1))
    for k, b in enumerate(vs):
        a0, a1 = (m, su - m) if k % 2 == 0 else (su - m, m)
        legs.append((tcp(a0, b), tcp(a1, b), True, k + 1))
        if k < npass - 1:
            legs.append((tcp(a1, b), tcp(a1, vs[k + 1]), False, k + 1))
    last = legs[-1][1]
    legs.append((last, last - surf.d * C.APPROACH, False, npass))
    return legs, npass


def legs_to_waypoints(legs, step=None):
    step = step or C.SAMPLE_STEP
    pos, spray, pid = [], [], []
    for a, b, sp, k in legs:
        for pt in _sample_leg(np.asarray(a, float), np.asarray(b, float), step):
            pos.append(pt); spray.append(sp); pid.append(k)
    pos.append(np.asarray(legs[-1][1], float)); pid.append(legs[-1][3])
    return np.array(pos), np.array(spray[:], bool), np.array(pid)


# ===================================================================== motion segments
class Wait:
    def __init__(self, duration, q):
        self.duration, self.q = duration, np.asarray(q, float)

    def sample(self, t):
        return self.q, np.zeros(6), None, False, 0


class JointMove:
    """Rest-to-rest quintic (minimum-jerk) joint move: zero velocity and acceleration at both ends."""
    def __init__(self, q0, q1):
        self.q0, self.q1 = np.asarray(q0, float), np.asarray(q1, float)
        dq = np.abs(self.q1 - self.q0)
        tv = 1.875 * dq / (C.J_VEL * C.VEL_SCALE)
        ta = np.sqrt(5.77 * dq / (C.J_ACC * C.ACC_SCALE))
        self.duration = float(max(0.8, tv.max(), ta.max()))

    def sample(self, t):
        T = self.duration
        s = min(max(t / T, 0.0), 1.0)
        pos = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        vel = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T
        dq = self.q1 - self.q0
        return self.q0 + dq * pos, dq * vel, None, False, 0


class JointTrajectory:
    """Time-stamped joint nodes + cubic Hermite interpolation. sample() returns q, qdot, spray flag, pass."""
    def __init__(self, q, t, seg_spray, pass_idx, tang):
        self.q, self.t, self.seg_spray, self.pass_idx, self.m = q, t, seg_spray, pass_idx, tang
        self.duration = float(t[-1])

    def sample(self, tt):
        tt = min(max(tt, 0.0), self.duration)
        i = int(np.clip(np.searchsorted(self.t, tt, side="right") - 1, 0, len(self.t) - 2))
        h = self.t[i + 1] - self.t[i]
        s = (tt - self.t[i]) / h
        q0, q1, m0, m1 = self.q[i], self.q[i + 1], self.m[i] * h, self.m[i + 1] * h
        q = (2 * s ** 3 - 3 * s ** 2 + 1) * q0 + (s ** 3 - 2 * s ** 2 + s) * m0 + (-2 * s ** 3 + 3 * s ** 2) * q1 + (s ** 3 - s ** 2) * m1
        qd = ((6 * s ** 2 - 6 * s) * q0 + (3 * s ** 2 - 4 * s + 1) * m0 + (-6 * s ** 2 + 6 * s) * q1 + (3 * s ** 2 - 2 * s) * m1) / h
        return q, qd, None, bool(self.seg_spray[i]), int(self.pass_idx[i])


def time_parameterise(pos, q, spray):
    """Segment times from Cartesian speed, rest-to-rest acceleration and joint-velocity limits.
    Nodes at sharp corners (pass ends) are 'rest' nodes: tangent 0, so the arm decelerates smoothly."""
    N = len(pos)
    d = np.diff(pos, axis=0)
    L = np.linalg.norm(d, axis=1)
    rest = np.zeros(N, bool)
    rest[0] = rest[-1] = True
    for i in range(1, N - 1):
        a, b = d[i - 1], d[i]
        c = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)
        rest[i] = c < math.cos(math.radians(30))
    dt = np.zeros(N - 1)
    for i in range(N - 1):
        v = C.PAINT_SPEED if spray[i] else C.TRANSIT_SPEED
        t_c = L[i] / v
        t_a = math.sqrt(6 * L[i] / C.CART_ACC) if (rest[i] and rest[i + 1]) else (
            math.sqrt(2 * L[i] / C.CART_ACC) if (rest[i] or rest[i + 1]) else 0.0)
        t_j = float(np.max(np.abs(q[i + 1] - q[i]) / (C.J_VEL * C.VEL_SCALE))) * 1.5
        dt[i] = max(t_c, t_a, t_j, 0.05)
    t = np.concatenate([[0], np.cumsum(dt)])
    m = np.zeros_like(q)
    for i in range(1, N - 1):
        if not rest[i]:
            m[i] = (q[i + 1] - q[i - 1]) / (t[i + 1] - t[i - 1])
    return t, m


# ===================================================================== rail loop route for the carriage
class Ring:
    def __init__(self):
        rx, ry = C.RING_HALF
        self.p = np.array([[rx, -ry], [rx, ry], [-rx, ry], [-rx, -ry], [rx, -ry]])
        self.l = np.linalg.norm(np.diff(self.p, axis=0), axis=1)
        self.cum = np.concatenate([[0], np.cumsum(self.l)])
        self.L = self.cum[-1]

    def point(self, s):
        s = s % self.L
        i = min(int(np.searchsorted(self.cum, s, side="right") - 1), 3)
        return self.p[i] + (self.p[i + 1] - self.p[i]) * (s - self.cum[i]) / self.l[i]

    def nearest(self, xy):
        best = (1e9, 0.0)
        for i in range(4):
            a, b = self.p[i], self.p[i + 1]
            t = np.clip(np.dot(xy - a, b - a) / np.dot(b - a, b - a), 0, 1)
            dist = np.linalg.norm(a + (b - a) * t - xy)
            if dist < best[0]:
                best = (dist, self.cum[i] + t * self.l[i])
        return best[1]

    def arc(self, s0, s1):
        ds = (s1 - s0) % self.L
        if ds > self.L / 2:
            ds -= self.L
        n = max(2, int(abs(ds) / 0.1))
        return [self.point(s0 + ds * k / n) for k in range(n + 1)]


def route(xy_from, xy_to):
    ring = Ring()
    xy_from, xy_to = np.asarray(xy_from, float), np.asarray(xy_to, float)
    s0, s1 = ring.nearest(xy_from), ring.nearest(xy_to)
    p0, p1 = ring.point(s0), ring.point(s1)
    pts = [xy_from]
    if np.linalg.norm(xy_from - p0) > 1e-6:
        pts.append(p0)
    pts += ring.arc(s0, s1)[1:]
    if np.linalg.norm(p1 - xy_to) > 1e-6:
        pts.append(xy_to)
    return np.array(pts)


class CarriageMove:
    """Robot pedestal rides along the rail route (minimum-jerk along arc length), tucked arm held still."""
    def __init__(self, pts, q_hold, yaw0):
        self.q = np.asarray(q_hold, float)
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.cum = np.concatenate([[0], np.cumsum(seg)])
        self.pts = pts
        yaw = np.unwrap(np.arctan2(-pts[:, 1], -pts[:, 0]))
        self.yaw = yaw + round((yaw0 - yaw[0]) / (2 * math.pi)) * 2 * math.pi
        self.len = float(self.cum[-1])
        self.duration = max(1.0, 1.875 * self.len / C.CARRIAGE_SPEED)

    def sample(self, t):
        s = min(max(t / self.duration, 0.0), 1.0)
        a = self.len * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
        x, y, yaw = (np.interp(a, self.cum, c) for c in (self.pts[:, 0], self.pts[:, 1], self.yaw))
        return self.q, np.zeros(6), (float(x), float(y), float(yaw)), False, 0


# ===================================================================== planner / reachability
class SurfacePlan:
    def __init__(self, surf, legs, npass):
        self.surf, self.legs, self.npass = surf, legs, npass
        self.jt, self.ok, self.report = None, False, {}
        self.approach_q = self.retreat_q = None


def staged(q0, q1):
    """Safe joint-space route HOME <-> work pose: (1) turret J1, (2) shoulder+elbow J2,J3, (3) wrist J4-J6.
    Avoids sweeping the unfurling arm through the car / fence (a straight joint interpolation does)."""
    a = np.array(q0, float); a[0] = q1[0]
    b = a.copy(); b[1:3] = q1[1:3]
    return [np.array(q0, float), a, b, np.array(q1, float)]


class Planner:
    def __init__(self, robot, checker):
        self.robot, self.checker = robot, checker

    def validate_joint_move(self, q0, q1, n=30):
        """Sample a rest-to-rest joint move and check clearance at every sample (robot at its station)."""
        mv, worst = JointMove(q0, q1), math.inf
        for k in range(n + 1):
            self.robot.reset_q(mv.sample(mv.duration * k / n)[0])
            worst = min(worst, self.checker.clearance())
        return worst >= C.PLAN_MIN_CLEAR, worst

    def validate_route(self, xy_from, xy_to, q_hold, step=0.25):
        pts = route(xy_from, xy_to)
        mv = CarriageMove(pts, q_hold, 0.0)
        self.robot.reset_q(q_hold)
        worst = math.inf
        for a in np.arange(0, mv.duration, step / max(mv.len, 1e-6) * mv.duration):
            self.robot.set_base(*mv.sample(a)[2])
            worst = min(worst, self.checker.clearance())
        return worst >= C.PLAN_MIN_CLEAR, worst

    def plan_surface(self, surf):
        rb, plan_snap = self.robot, self.robot.snapshot()
        rb.set_base(*station_pose(surf.station))
        legs, npass = generate_legs(surf)
        plan = SurfacePlan(surf, legs, npass)
        pos, spray, pid = legs_to_waypoints(legs)
        N = len(pos)
        quat = tool_quat(surf.d, surf.u)
        q = np.zeros((N, 6)); ok = np.zeros(N, bool); clr = np.full(N, np.inf)
        jumps, unreachable = [], []
        for i in range(N):
            if i == 0:
                qi, oki, pe, ae = rb.multi_seed_ik(pos[0], quat)
            else:
                qi, oki, pe, ae = rb.solve_ik(pos[i], quat, q[i - 1])
                if not oki:                                                         # fall back to multi-seed IK
                    qi, oki, pe, ae = rb.multi_seed_ik(pos[i], quat, ref=q[i - 1])
                if oki and np.max(np.abs(qi - q[i - 1])) > C.JUMP_LIMIT:            # IK branch flip -> retry
                    q2, ok2, pe2, ae2 = rb.multi_seed_ik(pos[i], quat, ref=q[i - 1])
                    if ok2 and np.max(np.abs(q2 - q[i - 1])) <= C.JUMP_LIMIT:
                        qi, oki = q2, True
                    else:
                        jumps.append(i)
            q[i], ok[i] = (qi if oki else (q[i - 1] if i else qi)), oki
            if not oki:
                unreachable.append(i)
            rb.reset_q(q[i])
            clr[i] = self.checker.clearance()
        n_bad_clear = int(np.sum(clr < C.PLAN_MIN_CLEAR))
        ok_all = (len(unreachable) == 0 and len(jumps) == 0 and n_bad_clear == 0)
        in_limits = bool(np.all(q >= C.J_LOWER) and np.all(q <= C.J_UPPER))
        t_tot = 0.0
        if ok_all:
            plan.approach_q, plan.retreat_q = q[0].copy(), q[-1].copy()
            plan.in_path = staged(C.Q_HOME, q[0])
            plan.out_path = staged(C.Q_HOME, q[-1])[::-1]
            for pth in (plan.in_path, plan.out_path):
                for a_, b_ in zip(pth[:-1], pth[1:]):
                    okm, _ = self.validate_joint_move(a_, b_)
                    ok_all &= okm
                    n_bad_clear += int(not okm)
            t, tang = time_parameterise(pos, q, spray[:N - 1] if len(spray) >= N else spray)
            plan.jt = JointTrajectory(q, t, spray[:N - 1], pid[:N - 1], tang)
            t_tot = plan.jt.duration
        plan.ok = ok_all
        plan.report = dict(surface=surf.name, station=surf.station, passes=npass, nodes=N,
                           reachable=int(ok.sum()), unreachable=len(unreachable), joint_jumps=len(jumps),
                           min_clearance_m=float(clr.min()), poses_below_clearance=n_bad_clear,
                           within_joint_limits=in_limits, path_length_m=float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))),
                           planned_duration_s=float(t_tot), success=bool(ok_all),
                           max_joint_abs=np.abs(q).max(axis=0).round(2).tolist())
        rb.restore(plan_snap)
        return plan

    def plan_collision_test(self):
        """Deliberate unsafe straight-line move toward the car side (demo of runtime collision stop)."""
        rb, snap = self.robot, self.robot.snapshot()
        rb.set_base(*station_pose("LEFT"))
        surf = load_surfaces()["LEFT"]
        quat = tool_quat(surf.d, surf.u)
        a, b = np.array([0.0, 1.75, 0.70]), np.array([0.0, 0.55, 0.70])
        pos = np.array(_sample_leg(a, b, 0.08) + [b])
        q = np.zeros((len(pos), 6)); good = True
        for i, pt in enumerate(pos):
            qi, oki, _, _ = rb.multi_seed_ik(pt, quat) if i == 0 else rb.solve_ik(pt, quat, q[i - 1])
            q[i] = qi; good &= oki
        spray = np.zeros(len(pos) - 1, bool)
        t, tang = time_parameterise(pos, q, spray)
        t = t * 2.0; tang = tang / 2.0                       # slow (about 0.3 m/s) so the stop is gentle
        jt = JointTrajectory(q, t, spray, np.ones(len(pos) - 1, int), tang)
        rb.restore(snap)
        return q[0].copy(), jt, good
