# Robotic Car Painting Cell — PyBullet

Original, from-scratch simulation: procedural 4.8 x 1.9 x 1.6 m car, 6-DOF industrial painting robot on a rail loop,
spray gun with TCP, planned zig-zag painting trajectories, IK reachability check, real collision checking, coverage model.
No external assets, no third-party code.

## Run (Windows)
1. Install Python 3.9+ (tick "Add to PATH"). 2. `pip install pybullet numpy` (or `pip install -r requirements.txt`)
3. `python main.py`          (add `--auto` to start the demo immediately)

Self-test without GUI: `python main.py --headless --shots` (runs the full demo, writes `report/results.json` and `screenshots/`).
> If `pip install pybullet` tries to compile and fails on Windows, install "Microsoft C++ Build Tools" or use Python 3.10/3.11 (prebuilt wheels).

## Keys
`1` HOME · `2` LEFT · `3` RIGHT · `4` ROOF · `5` HOOD · `6` TRUNK · `A` automatic demo · `S` stop (then `R`) · `R` reset ·
`P` pause/resume · `X` collision test · `F1-F6` / `C` cameras · `V` auto-camera · `F` fast-forward · `K` screenshot.
Surface keys only work when the robot is idle; after a STOP or COLLISION press `R`.

## Architecture
| File | Role |
|---|---|
| `config.py` | all numbers (geometry, limits, speeds, thresholds, surfaces) |
| `car_model.py` | visual car from primitives + ONE compound collision body |
| `cell_layout.py` | floor, rail loop, fence, virtual safety walls |
| `industrial_robot.py` | URDF generator, joint control, FK/IK |
| `painting_tool.py` | spray cone, TCP marker, planned/executed path lines |
| `trajectories.py` | Cartesian passes, IK planner/reachability, time parameterisation, motion segments |
| `collision_checker.py` | SAFE / WARNING / COLLISION from PyBullet queries |
| `painting_system.py` | spray conditions + coverage grid + paint tiles |
| `camera_manager.py`, `ui.py` | camera presets, HUD, keyboard |
| `main.py` | executor (state machine), demo sequence, headless test |

## Robotics concepts (what to say in the interview)
* **Frames**: world (+X front, +Y left, +Z up); robot base frame (x faces the car); tool frame (z = spray direction, origin = TCP).
* **TCP**: nozzle tip, a fixed link 0.40 m beyond the flange. IK targets the TCP pose, so position AND gun orientation are controlled.
* **7th axis (honest note)**: the arm has 6 revolute joints, but a single arm cannot reach both sides of a 1.9 m wide car from one spot,
  so the pedestal rides on a rectangular rail loop (kinematic carriage, smooth min-jerk motion). Paint-line robots use rails the same way.
  Joint-space painting motion is done only by the 6 arm joints with the carriage stationary.
* **Inverse kinematics**: PyBullet damped-least-squares IK with joint limits, seeded from the previous node (branch continuity), multi-seed fallback,
  and FK verification (error < 1 mm / 0.5°). Near-singular wrists (|J5| < 0.15 rad) are rejected.
* **Trajectory generation**: per panel, zig-zag passes (spacing 0.14 m, spray footprint ~0.2 m) at 0.30 m standoff, constant tool orientation
  (normal, optionally tilted for roof/hood/trunk), lead-in/lead-out. IK at nodes → joint-space cubic Hermite spline; zero velocity at pass ends;
  time from Cartesian speed, rest-to-rest acceleration and 50 % of joint velocity limits.
* **Joint control**: `setJointMotorControlArray` (position control + velocity feed-forward). No teleporting (only RESET re-initialises the pose).
* **Safe joint routes**: HOME ↔ work pose is staged (turret, then shoulder/elbow, then wrist) because a straight joint interpolation swept the arm through the fence/car (found by the planner's own clearance check).
* **Collision checking**: `getContactPoints` + `getClosestPoints` against the car and the safety walls, every 60 Hz. <0.10 m = WARNING, <0.02 m or contact = COLLISION → motion halted, involved link/body printed and shown. TCP distance via `linkIndexA`. Planner uses the same checker to reject poses/routes.
* **Reachability**: every node must have a verified IK solution, no joint jumps, clearance ≥ 0.10 m; otherwise the surface is reported FAILED and refused (never silently skipped).
* **Painting/coverage**: spray active when planned spray flag AND distance in 0.15–0.50 m AND incidence ≤ 45°; spray ray ∩ panel plane → grid cells (8 cm) inside the footprint are painted (tile spawned). Coverage = painted cells / cells. Visual model only — not paint thickness.

## Results
Run the demo and read `report/results.json` (written at the end of the automatic demo): execution time, nodes, reachable nodes, clearance, warnings, collisions, coverage per surface. Quote only those numbers.

## Limitations
No self-collision check; coverage is a binary grid (no thickness/overspray); constant tool orientation per panel; kinematic rail carriage (no dynamics); circular footprint approximation under tilt; GUI HUD/camera/keyboard were not visually verified in the author's headless sandbox (logic was tested headless); wheels/pillars are low-poly.

## Testing checklist
1. `python main.py` opens window, HUD top-left, robot in L-shaped zero pose.  2. `A`: 14 phases run, coverage reaches 100 %, `report/results.json` appears.
3. `2`…`6` individually; `P` pauses; `S` then `R` recovers.  4. `X`: WARNING then COLLISION, motion halts, `R` clears.  5. `F1`–`F6` cameras; `V` auto-camera.
6. Console prints the reachability table at start (all 5 surfaces OK).
