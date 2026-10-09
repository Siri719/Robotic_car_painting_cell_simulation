"""Real collision checking with PyBullet queries (nothing is faked).

  p.getContactPoints  -> actual contact / penetration between robot and obstacle
  p.getClosestPoints  -> signed distance to the nearest obstacle (used for the WARNING band)

Obstacles: CAR (compound collision body) and FENCE (virtual safety walls). The spray gun links and the
TCP are separate links of the robot, so 'robot vs car', 'gun vs car' and 'robot vs boundary' are all
reported with the robot link name involved. Self-collision of the arm is NOT checked (limitation).
"""
import math
import pybullet as p
import config as C
from industrial_robot import IndustrialRobot as R

SAFE, WARNING, COLLISION = "SAFE", "WARNING / TOO CLOSE", "COLLISION"


class CollisionChecker:
    def __init__(self, robot, car_id, fence_id):
        self.robot, self.car, self.fence = robot, car_id, fence_id
        self.obstacles = (("CAR", car_id), ("FENCE", fence_id))

    def check(self, fresh=False, cap=1.5):
        """Returns dict(status, min_dist, detail, tcp_dist)."""
        if fresh:                       # after resetJointState (planning) force a collision update
            p.performCollisionDetection()
        rid = self.robot.id
        min_d, who, hit = math.inf, None, None
        for name, oid in self.obstacles:
            for c in p.getContactPoints(rid, oid):
                if c[8] <= 0.002 and hit is None:
                    hit = f"{R.LINK_NAMES.get(c[3], c[3])} vs {name}"
            for c in p.getClosestPoints(rid, oid, cap):
                if c[8] < min_d:
                    min_d, who = c[8], f"{R.LINK_NAMES.get(c[3], c[3])} vs {name}"
        tcp_d = math.inf
        for c in p.getClosestPoints(rid, self.car, cap, linkIndexA=R.TCP):
            tcp_d = min(tcp_d, c[8])
        if hit or min_d < C.HIT_DIST:
            status, detail = COLLISION, hit or who
        elif min_d < C.WARN_DIST:
            status, detail = WARNING, who
        else:
            status, detail = SAFE, ""
        return dict(status=status, min_dist=min_d, detail=detail, tcp_dist=tcp_d)

    def clearance(self):
        """Planning-time clearance of the robot at its current (reset) configuration."""
        return self.check(fresh=True)["min_dist"]
