"""Robotic Car Painting Cell - PyBullet.   python main.py [--auto] [--headless] [--shots]

Control loop (240 Hz):  sample active motion segment -> joint motor commands -> stepSimulation ->
measure real TCP -> paint logic -> collision check (60 Hz) -> HUD (12 Hz).
"""
import argparse
import json
import math
import os
import time
from collections import deque

import numpy as np
import pybullet as p

import config as C
from camera_manager import CameraManager, save_png
from car_model import build_car
from cell_layout import add_labels, build_cell
from collision_checker import COLLISION, SAFE, WARNING, CollisionChecker
from industrial_robot import IndustrialRobot
from painting_system import PaintingSystem
from painting_tool import PathVisualizer, SprayVisual
from trajectories import (staged, CarriageMove, JointMove, Planner, Wait, load_surfaces, route, station_pose)
from ui import COLORS, HUD, LEGEND, poll_keys

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
for _d in ("assets", "screenshots", "report"):
    os.makedirs(os.path.join(HERE, _d), exist_ok=True)
TOTAL_PHASES = 14


class App:
    def __init__(self, gui=True, verbose=True, hud=True):
        self.gui, self.use_hud = gui, hud
        p.connect(p.GUI, options="--width=1500 --height=900") if gui else p.connect(p.DIRECT)
        p.setAdditionalSearchPath(ASSETS)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(C.DT)
        if gui:
            for f, v in ((p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0), (p.COV_ENABLE_GUI, 0), (p.COV_ENABLE_SHADOWS, int(C.SHADOWS))):
                p.configureDebugVisualizer(f, v)
            p.configureDebugVisualizer(rgbBackground=[0.86, 0.89, 0.93], lightPosition=[6, -8, 12])
        self.cell, self.car = build_cell(), build_car(ASSETS)
        if gui:
            add_labels()
        self.robot = IndustrialRobot(ASSETS)
        self.checker = CollisionChecker(self.robot, self.car["collision"], self.cell["collision"])
        self.surfaces = load_surfaces()
        self.painter = PaintingSystem(self.surfaces)
        self.spray, self.paths = SprayVisual(ASSETS), PathVisualizer()
        self.cams, self.hud = CameraManager(gui), HUD(gui and hud)
        self.planner = Planner(self.robot, self.checker)
        self.plans, self.plan_report = {}, {}
        self.results = dict(surfaces={}, collisions=0, warnings=0)
        self.verbose = verbose
        self.reset(first=True)
        self.plan_all()
        self.reset()
        self.cams.set(1)

    # ------------------------------------------------------------------ planning / reachability report
    def plan_all(self):
        t0 = time.time()
        # carriage-route clearance (arm tucked at HOME)
        self.route_report = {}
        for a, b in (("LEFT", "RIGHT"), ("RIGHT", "ROOF"), ("ROOF", "FRONT"), ("FRONT", "REAR"), ("REAR", "LEFT"), ("LEFT", "ROOF")):
            snap = self.robot.snapshot()
            ok, clr = self.planner.validate_route(C.STATIONS[a], C.STATIONS[b], C.Q_HOME)
            self.robot.restore(snap)
            self.route_report[f"{a}->{b}"] = dict(ok=bool(ok), min_clearance_m=float(clr))
        for name, surf in self.surfaces.items():
            plan = self.planner.plan_surface(surf)
            self.plans[name], self.plan_report[name] = plan, plan.report
        self.plan_time = time.time() - t0
        if self.verbose:
            print("\n=== REACHABILITY / PLANNING REPORT ===")
            for n, r in self.plan_report.items():
                print(f"{n:6s} station={r['station']:5s} passes={r['passes']:2d} nodes={r['nodes']:3d} reachable={r['reachable']}/{r['nodes']} "
                      f"jumps={r['joint_jumps']} min_clear={r['min_clearance_m']:.3f} m  planned={r['planned_duration_s']:.1f} s  -> {'OK' if r['success'] else 'FAILED'}")
            for k, r in self.route_report.items():
                print(f"rail route {k:12s} min clearance {r['min_clearance_m']:.3f} m -> {'OK' if r['ok'] else 'FAILED'}")
            print(f"planning time {self.plan_time:.1f} s\n")

    # ------------------------------------------------------------------ state
    def reset(self, first=False):
        self.queue, self.cur, self.seg_t = deque(), None, 0.0
        self.mode, self.status, self.paused, self.fast = "MANUAL", "IDLE", False, False
        self.col = dict(status=SAFE, min_dist=math.inf, detail="", tcp_dist=math.inf)
        self.col_lock = False
        self.step_n, self.sim_t = 0, 0.0
        self.active_surface, self.pass_idx, self.spraying, self.phase = None, 0, False, (0, "READY")
        self.hold_q = C.Q_ZERO.copy()
        self.state = dict(xy=np.array(C.STATIONS["LEFT"], float), yaw=station_pose("LEFT")[2], q=C.Q_ZERO.copy())
        self.robot.set_base(*station_pose("LEFT"))
        self.robot.reset_q(C.Q_ZERO)
        self.robot.command(C.Q_ZERO, np.zeros(6))
        if not first:
            self.painter.reset()
            self.paths.clear()
            self.spray.hide()
        self.min_clear_run = math.inf
        self.last_tcp = None

    # ------------------------------------------------------------------ action building
    def _act(self, phase, label, seg, surface=None, kind="move"):
        return dict(phase=(phase, label), seg=seg, surface=surface, kind=kind)

    def _moves(self, phase, label, path):
        return [self._act(phase, label, JointMove(a, b)) for a, b in zip(path[:-1], path[1:])
                if np.linalg.norm(a - b) > 1e-6]

    def _home_and_station(self, v, st, phase, label, acts):
        """From virtual state v: tuck arm, ride rail to station `st` (no-ops when already there)."""
        if np.linalg.norm(v["q"] - C.Q_HOME) > 1e-6:
            acts.append(self._act(phase, label, JointMove(v["q"], C.Q_HOME)))
            v["q"] = C.Q_HOME.copy()
        tgt = np.array(C.STATIONS[st], float)
        if np.linalg.norm(v["xy"] - tgt) > 1e-6:
            mv = CarriageMove(route(v["xy"], tgt), C.Q_HOME, v["yaw"])
            acts.append(self._act(phase, label, mv))
            v["xy"], v["yaw"] = tgt, float(mv.yaw[-1])

    def build_surface(self, v, name, ph, labels, acts):
        plan = self.plans[name]
        self._home_and_station(v, plan.surf.station, ph[0], labels[0], acts)
        acts += self._moves(ph[0], labels[0], plan.in_path)
        acts.append(self._act(ph[1], f"PAINT {name}", plan.jt, surface=name, kind="paint"))
        v["q"] = plan.retreat_q.copy()

    def build_demo(self):
        v, acts = dict(self.state), []
        acts.append(self._act(1, "SYSTEM INITIALIZATION", Wait(2.0, v["q"]), kind="wait"))
        acts.append(self._act(2, "MOVE TO HOME", JointMove(v["q"], C.Q_HOME)))
        v["q"] = C.Q_HOME.copy()
        phases = {"LEFT": (3, 4), "RIGHT": (6, 7), "ROOF": (8, 9), "HOOD": (10, 11), "TRUNK": (12, 13)}
        order = [n for n in C.DEMO_ORDER if self.plans[n].ok]
        for i, n in enumerate(order):
            ph = phases[n]
            plan = self.plans[n]
            self._home_and_station(v, plan.surf.station, ph[0], f"MOVE TO {n} SIDE" if n in ("LEFT", "RIGHT") else f"MOVE TO {n}", acts)
            acts += self._moves(ph[0], f"MOVE TO {n} START", plan.in_path)
            acts.append(self._act(ph[1], f"PAINT {n}", plan.jt, surface=n, kind="paint"))
            # retreat to HOME: LEFT gets its own phase 5 ("move safely away"), others merge into next phase
            nxt = phases[order[i + 1]][0] if i + 1 < len(order) else 14
            acts += self._moves(5 if n == "LEFT" else nxt, "MOVE SAFELY AWAY" if n == "LEFT" else
                                (f"MOVE TO {order[i + 1]}" if i + 1 < len(order) else "RETURN TO HOME"), plan.out_path)
            v["q"] = C.Q_HOME.copy()
        self._home_and_station(v, "LEFT", 14, "RETURN TO HOME", acts)
        return acts

    def build_manual(self, cmd):
        v, acts = dict(self.state), []
        if cmd == "HOME":
            acts.append(self._act(0, "MOVE TO HOME", JointMove(v["q"], C.Q_HOME)) if np.linalg.norm(v["q"] - C.Q_HOME) > 1e-6 else None)
            v["q"] = C.Q_HOME.copy()
            self._home_and_station(v, "LEFT", 0, "MOVE TO HOME", acts)
            acts = [a for a in acts if a]
        elif cmd in self.plans:
            if not self.plans[cmd].ok:
                print(f"[PLAN] {cmd} is not reachable/safe - refusing to execute")
                return []
            plan = self.plans[cmd]
            self._home_and_station(v, plan.surf.station, 0, f"MOVE TO {cmd}", acts)
            acts += self._moves(0, f"MOVE TO {cmd} START", plan.in_path)
            acts.append(self._act(0, f"PAINT {cmd}", plan.jt, surface=cmd, kind="paint"))
            acts += self._moves(0, "MOVE SAFELY AWAY", plan.out_path)
        elif cmd == "COLTEST":
            q0, jt, ok = self.planner.plan_collision_test()
            self._home_and_station(v, "LEFT", 0, "COLLISION TEST: APPROACH", acts)
            acts += self._moves(0, "COLLISION TEST: APPROACH", staged(C.Q_HOME, q0))
            acts.append(self._act(0, "COLLISION TEST: UNSAFE MOVE", jt, kind="test"))
        return acts

    # ------------------------------------------------------------------ executor
    def start(self, acts, mode):
        if self.col_lock or not acts or self.queue or self.cur:
            return
        self.queue, self.mode, self.paused = deque(acts), mode, False
        self.next_action()

    def next_action(self):
        if self.cur and self.cur["kind"] == "paint":
            self.finish_paint(self.cur)
        if self.cur:
            self.state["q"] = self.cur["seg"].sample(self.cur["seg"].duration)[0].copy() if not isinstance(self.cur["seg"], CarriageMove) else self.cur["seg"].q.copy()
            if isinstance(self.cur["seg"], CarriageMove):
                s = self.cur["seg"].sample(self.cur["seg"].duration)[2]
                self.state["xy"], self.state["yaw"] = np.array(s[:2]), s[2]
        if not self.queue:
            self.cur, self.status = None, "IDLE"
            self.active_surface, self.spraying = None, False
            self.spray.hide()
            if self.mode == "AUTOMATIC":
                self.finish_demo()
            return
        self.cur, self.seg_t = self.queue.popleft(), 0.0
        self.phase = self.cur["phase"]
        a = self.cur
        if a["kind"] == "paint":
            self.active_surface = a["surface"]
            self.paths.clear_executed()
            self.paths.show_planned(self.plans[a["surface"]].legs)
            self.paint_t0, self.paint_clear = self.sim_t, math.inf
            self.n_warn0 = self.results["warnings"]
            if self.cams.auto:
                self.cams.set({"LEFT": 2, "RIGHT": 3, "ROOF": 4}.get(a["surface"], 1))
        else:
            if a["kind"] != "test":
                self.paths.clear_planned()
            self.active_surface = None

    def finish_paint(self, a):
        n = a["surface"]
        plan = self.plans[n]
        self.results["surfaces"][n] = dict(
            execution_time_s=round(self.sim_t - self.paint_t0, 2), trajectory_nodes=plan.report["nodes"],
            passes=plan.report["passes"], reachable_nodes=plan.report["reachable"],
            planned_min_clearance_m=round(plan.report["min_clearance_m"], 3),
            runtime_min_clearance_m=round(self.paint_clear, 3), warnings=self.results["warnings"] - self.n_warn0,
            coverage_percent=round(100 * self.painter.progress(n), 1))
        self.paths.clear_planned()

    def finish_demo(self):
        if self.cams.auto:
            self.cams.set(6)
        self.results["total_coverage_percent"] = round(100 * self.painter.total_progress(), 1)
        self.results["total_sim_time_s"] = round(self.sim_t, 1)
        self.results["planning_reports"] = self.plan_report
        self.results["rail_route_reports"] = self.route_report
        path = os.path.join(HERE, "report", "results.json")
        with open(path, "w") as f:
            json.dump(self.results, f, indent=2)
        if self.verbose:
            print(f"[DEMO COMPLETE] results written to {path}")

    def stop(self):
        self.queue, self.cur, self.mode, self.status = deque(), None, "STOPPED", "STOPPED (press R)"
        self.spray.hide()
        self.spraying = False

    def step(self):
        rb = self.robot
        q_cmd, qd = self.hold_q, np.zeros(6)
        spray_cmd = False
        if self.cur and not self.paused and not self.col_lock:
            seg = self.cur["seg"]
            q_cmd, qd, base, spray_cmd, pid = seg.sample(self.seg_t)
            if base is not None:
                rb.set_base(*base)
            self.hold_q = np.array(q_cmd)
            self.pass_idx = pid
            self.seg_t += C.DT
            self.status = "PAINTING" if self.cur["kind"] == "paint" and spray_cmd else "MOVING"
            if self.seg_t >= seg.duration:
                self.next_action()
        rb.command(q_cmd, qd)
        p.stepSimulation()
        self.step_n += 1
        self.sim_t += C.DT
        if self.step_n % 2 == 0 or True:
            tcp, quat = rb.tcp_pose()
            active = self.painter.update(self.active_surface, tcp, quat, spray_cmd and not self.col_lock)
            self.spraying = active
            if active:
                self.paths.add_executed(tcp)
            self.last_tcp = tcp
            if self.step_n % C.VIS_EVERY == 0:
                self.spray.update(tcp, quat, active)
                self.cams.follow(tcp)
        if self.step_n % C.CHECK_EVERY == 0:
            prev = self.col["status"]
            self.col = self.checker.check()
            self.min_clear_run = min(self.min_clear_run, self.col["min_dist"])
            if self.cur and self.cur["kind"] == "paint":
                self.paint_clear = min(self.paint_clear, self.col["min_dist"])
            if self.col["status"] == WARNING and prev != WARNING:
                self.results["warnings"] += 1
            if self.col["status"] == COLLISION and not self.col_lock:
                self.col_lock, self.status, self.mode = True, "HALTED - COLLISION", "COLLISION STOP"
                self.results["collisions"] += 1
                self.spray.hide()
                print(f"[COLLISION] {self.col['detail']}  (min distance {self.col['min_dist']*1000:.1f} mm) - motion halted, press R to reset")

    # ------------------------------------------------------------------ HUD
    def draw_hud(self):
        s = self.active_surface or "-"
        prog = 100 * (self.painter.progress(self.active_surface) if self.active_surface else self.painter.total_progress())
        n = self.plans[self.active_surface].npass if self.active_surface in self.plans else 0
        td = self.col["tcp_dist"]
        c = COLORS[self.col["status"]]
        L = [("ROBOTIC CAR PAINTING CELL", [0.05, 0.1, 0.3], 1.6),
             (f"Mode: {self.mode}{'  (PAUSED)' if self.paused else ''}", [0, 0, 0], 1.2),
             (f"Phase: {self.phase[0]}/{TOTAL_PHASES}  {self.phase[1]}" if self.phase[0] else f"Phase: {self.phase[1]}", [0, 0, 0], 1.2),
             (f"Current Surface: {s}", [0, 0, 0], 1.2),
             (f"Paint Progress: {prog:.0f}%" + ("" if self.active_surface else "  (total)"), [0, 0, 0], 1.2),
             (f"Robot Status: {self.status}", [0, 0, 0], 1.2),
             (f"Collision: {self.col['status']}" + (f"  [{self.col['detail']}]" if self.col["detail"] else ""), c, 1.3),
             (f"TCP Distance: {td:.2f} m" if td < 1.5 else "TCP Distance: > 1.50 m", [0, 0, 0], 1.2),
             (f"Trajectory: PASS {self.pass_idx} / {n}" if n and self.active_surface else "Trajectory: -", [0, 0, 0], 1.2),
             ("", [0, 0, 0], 1.0)] + [(t, [0.1, 0.1, 0.1], 1.0) for t in LEGEND]
        self.hud.draw(L)

    # ------------------------------------------------------------------ keys
    def handle_key(self, k):
        if k == "RESET":
            self.reset(); return
        if k == "STOP":
            self.stop(); return
        if k == "PAUSE":
            self.paused = not self.paused; return
        if k == "FAST":
            self.fast = not self.fast; return
        if k == "AUTOCAM":
            self.cams.auto = not self.cams.auto; return
        if k == "SHOT":
            self.screenshot(); return
        if k == "CAMCYCLE":
            self.cams.cycle(); return
        if k.startswith("CAM"):
            self.cams.set(int(k[3])); return
        if self.cur or self.queue or self.col_lock or self.mode == "STOPPED":
            return                                               # ignore commands while busy / halted
        if k == "AUTO":
            self.reset()
            self.start(self.build_demo(), "AUTOMATIC")
        else:
            self.start(self.build_manual(k), "MANUAL")

    def screenshot(self):
        os.makedirs(os.path.join(HERE, "screenshots"), exist_ok=True)
        path = os.path.join(HERE, "screenshots", f"shot_{int(time.time())}.png")
        try:
            w, h, vm, pm = p.getDebugVisualizerCamera()[:4]
            img = p.getCameraImage(w, h, vm, pm, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            save_png(path, np.array(img[2], dtype=np.uint8).reshape(h, w, 4)[:, :, :3])
            print("saved", path)
        except Exception as e:
            print("screenshot failed:", e)

    # ------------------------------------------------------------------ main loops
    def run(self, auto=False):
        if auto:
            self.handle_key("AUTO")
        hud_dt, last_hud = 1.0 / C.HUD_HZ, 0.0
        try:
            while p.isConnected():
                t0 = time.time()
                for k in poll_keys():
                    self.handle_key(k)
                self.step()
                if self.use_hud and t0 - last_hud > hud_dt:
                    self.draw_hud()
                    last_hud = t0
                if not self.fast:
                    dt = C.DT - (time.time() - t0)
                    if dt > 0:
                        time.sleep(dt)
        except p.error as e:
            print(f"[INFO] PyBullet window closed or crashed: {e}")
            print("       If it closed by itself on start-up, run:  python main.py --no-hud   (and tell me what happens)")

    def run_headless(self, shots=False, max_s=900):
        os.makedirs(os.path.join(HERE, "screenshots"), exist_ok=True)
        self.handle_key("AUTO")
        last_surface, t_wall = None, time.time()
        while (self.cur or self.queue) and self.sim_t < max_s and not self.col_lock:
            self.step()
            if shots and self.cur and self.cur["kind"] == "paint" and self.cur["seg"].duration - self.seg_t < C.DT * 1.5:
                self._shot(f"painted_{self.cur['surface'].lower()}.png", {"LEFT": 2, "RIGHT": 3, "ROOF": 4}.get(self.cur["surface"], 1))
        print(f"headless run finished: sim {self.sim_t:.1f} s in {time.time()-t_wall:.1f} s wall, collisions={self.results['collisions']}")
        if shots:
            self._shot("final_result.png", 6)
        return self.results

    def _shot(self, name, view):
        save_png(os.path.join(HERE, "screenshots", name), CameraManager.render(view, tcp=self.last_tcp))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true", help="start the automatic demo immediately")
    ap.add_argument("--headless", action="store_true", help="run the whole demo without GUI (self-test)")
    ap.add_argument("--no-hud", action="store_true", help="disable on-screen text panel (debug)")
    ap.add_argument("--shots", action="store_true", help="with --headless: save screenshots")
    a = ap.parse_args()
    app = App(gui=not a.headless, hud=not a.no_hud)
    if a.headless:
        r = app.run_headless(shots=a.shots)
        print(json.dumps({k: v for k, v in r.items() if k not in ("planning_reports", "rail_route_reports")}, indent=1))
    else:
        app.run(auto=a.auto)