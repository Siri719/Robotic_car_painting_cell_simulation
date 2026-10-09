"""Central configuration. SI units everywhere (m, rad, s, kg).

WORLD FRAME  : origin on the floor under the car centre, +X = car front, +Y = car LEFT, +Z = up.
BASE FRAME   : robot base on the floor, x axis = direction the robot turret faces at J1 = 0.
TOOL FRAME   : z axis = spray direction (flange -> nozzle), TCP = nozzle tip.
"""
import math
import numpy as np

# ---------------------------------------------------------------- simulation
DT = 1.0 / 240.0            # physics step
CHECK_EVERY = 4             # collision check every N steps (60 Hz)
VIS_EVERY = 4               # spray / TCP visual update every N steps
HUD_HZ = 12.0
SHADOWS = True

# ---------------------------------------------------------------- car (4.8 x 1.9 x 1.6 m)
CAR_L, CAR_W, CAR_H = 4.8, 1.9, 1.6

# ---------------------------------------------------------------- robot (large painting robot, ~3.1 m reach)
ARM = dict(ped_h=0.50, ped_r=0.38, shoulder=(0.20, 0.0, 0.55),
           upper=1.60, fore=1.50, w1=0.20, w2=0.18)
TOOL_LEN = 0.40                                   # flange -> nozzle tip (TCP)
J_LOWER = np.array([-3.30, -1.30, -2.60, -5.50, -2.00, -6.00])
J_UPPER = np.array([ 3.30,  2.20,  2.60,  5.50,  2.00,  6.00])
J_VEL   = np.array([ 1.40,  1.40,  1.60,  2.50,  2.50,  3.50])   # rad/s  (limits)
J_ACC   = np.array([ 2.00,  2.00,  2.50,  4.00,  4.00,  5.00])   # rad/s^2
J_FORCE = np.array([5000., 5000., 3500., 1200., 800., 400.])      # motor max torque (N m)
VEL_SCALE = 0.5             # planner uses 50 % of the velocity limits
ACC_SCALE = 0.5
Q_ZERO = np.array([0.0, 0.0, 1.57, 0.0, 0.0, 0.0])   # power-on / calibration pose
Q_HOME = np.array([3.0, -0.6, 1.6, 0.0, -1.0, 0.0])   # compact tucked HOME pose (max radius ~0.75 m)

# ---------------------------------------------------------------- cell layout
# The robot pedestal rides on a rectangular rail loop (7th axis, kinematic carriage) around the car.
RING_HALF = (3.4, 3.2)                     # rail loop half sizes (x, y)
STATIONS = {                               # carriage stop positions (x, y) on the floor
    'LEFT':  (0.0,  3.2),
    'RIGHT': (0.0, -3.2),
    'FRONT': (3.4,  0.0),
    'REAR': (-3.4,  0.0),
    'ROOF':  (0.0, -2.1),                  # spur of the rail, close to the right side
}
CARRIAGE_SPEED = 0.9                       # m/s peak
FENCE_HALF = (5.0, 4.3)                    # safety fence half sizes (x, y)

# ---------------------------------------------------------------- painting
STANDOFF = 0.30             # nozzle -> surface distance while spraying
APPROACH = 0.45             # extra distance for lead-in / lead-out
SPRAY_HALF_ANGLE = math.radians(18.0)
SPRAY_RANGE = (0.15, 0.50)  # valid spray distance window
MAX_INCIDENCE = math.radians(45.0)
PAINT_SPEED = 0.35          # m/s along a pass
TRANSIT_SPEED = 0.60
CART_ACC = 1.0              # m/s^2 used for rest-to-rest segments
SAMPLE_STEP = 0.12          # distance between IK nodes along a pass
PASS_SPACING = 0.14
EDGE_INSET = 0.05
CELL = 0.08                 # coverage grid resolution

# ---------------------------------------------------------------- collision thresholds
WARN_DIST = 0.10            # closer than this -> WARNING
HIT_DIST = 0.02             # closer than this (or contact) -> COLLISION, motion halted
PLAN_MIN_CLEAR = 0.10       # planner rejects poses closer than this
IK_TOL_POS = 1e-3
IK_TOL_ANG = math.radians(0.5)
JUMP_LIMIT = 0.45           # rad: max joint change between neighbouring IK nodes

# ---------------------------------------------------------------- surfaces to paint
# origin/u/v span the panel, normal points OUT of the car. tilt tilts the gun in-plane (gives the
# wrist a comfortable posture and avoids singular poses).
SURFACES = {
    'LEFT':  dict(station='LEFT',  origin=(-2.10,  0.95, 0.42), u=(1, 0, 0), v=(0, 0, 1), normal=(0, 1, 0),
                  size=(4.20, 0.50), tilt_deg=0,  tilt_dir=(0, 0, 1), color=(0.85, 0.10, 0.10, 1)),
    'RIGHT': dict(station='RIGHT', origin=(-2.10, -0.95, 0.42), u=(1, 0, 0), v=(0, 0, 1), normal=(0, -1, 0),
                  size=(4.20, 0.50), tilt_deg=0,  tilt_dir=(0, 0, 1), color=(0.10, 0.30, 0.85, 1)),
    'ROOF':  dict(station='ROOF',  origin=(-0.90, -0.70, 1.60), u=(1, 0, 0), v=(0, 1, 0), normal=(0, 0, 1),
                  size=(1.40, 1.40), tilt_deg=25, tilt_dir=(0, 1, 0), color=(0.95, 0.75, 0.10, 1)),
    'HOOD':  dict(station='FRONT', origin=(1.00, -0.80, 0.95), u=(0, 1, 0), v=(1, 0, 0), normal=(0, 0, 1),
                  size=(1.60, 1.20), tilt_deg=20, tilt_dir=(-1, 0, 0), color=(0.15, 0.65, 0.30, 1)),
    'TRUNK': dict(station='REAR',  origin=(-2.20, -0.80, 0.95), u=(0, 1, 0), v=(1, 0, 0), normal=(0, 0, 1),
                  size=(1.60, 0.85), tilt_deg=20, tilt_dir=(1, 0, 0), color=(0.60, 0.25, 0.75, 1)),
}
DEMO_ORDER = ['LEFT', 'RIGHT', 'ROOF', 'HOOD', 'TRUNK']
