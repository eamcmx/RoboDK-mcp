"""
square_trajectory.py
====================
Moves the Right SDA10F master along a square on the world ZY plane.
The MCP master-slave sync daemon updates Left SDA10F automatically.

Strategy:
  Phase 1 -- Move from current pose to the square start corner (iterative IK).
  Phase 2 -- Execute the square: +Y -> +Z -> -Y -> -Z -> back to start corner.

Key config
----------
  SIDE_MM : side length of the square (mm)
  SQ_DY0  : Y offset of start corner from current TCP (world frame)
  SQ_DZ0  : Z offset of start corner from current TCP (world frame)

Workspace tip: run probe_workspace.py first to determine safe SIDE_MM and
offset values for the current robot pose.

Run via MATLAB MCP:
    [s, o] = system('python "path\\to\\square_trajectory.py" 2>&1'); disp(o);
"""

import time
import math
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk.robomath import Mat, invH

# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------
MASTER_NAME = "Right SDA10F"
SIDE_MM     = 300.0    # square side length (mm)
STEP_MM     = 5.0      # Cartesian step between IK calls (keep <= 5 mm)
DELAY_S     = 0.04     # seconds between steps (~25 fps)
HOST        = "localhost"
PORT        = 20500

# Start-corner offsets from current TCP (world ZY plane).
# Default centres a 300 mm square on the home TCP.
SQ_DY0 = -150.0
SQ_DZ0 = -150.0
# --------------------------------------------------------------------------


def world_tcp(robot):
    """TCP in world frame via FK: base_in_world * FK(q) * PoseTool."""
    parent = robot.Parent()
    base   = parent.PoseAbs() if parent.Valid() else Mat()
    return base * robot.SolveFK(robot.Joints()) * robot.PoseTool()


def solve_ik(robot, T_target_world):
    """
    Inverse kinematics seeded from current joints.
    Converts the world-frame target to the robot's base frame before calling
    SolveIK -- required for sub-robots whose base is not at world origin.
    Returns a flat joint list on success, None on failure.
    """
    parent = robot.Parent()
    base   = parent.PoseAbs() if parent.Valid() else Mat()
    T_base = invH(base) * T_target_world   # world -> base frame
    q_seed = robot.Joints().list()         # seed keeps IK on same branch
    sol    = robot.SolveIK(T_base, q_seed)
    try:
        joints = sol.tolist()              # flat list of N joint angles
        if not joints or len(joints) < 6:
            return None
        return joints
    except Exception:
        return None


def lerp_pts(p1, p2, step_mm):
    """Yield 3-D points interpolated from p1 to p2 in step_mm increments."""
    dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    dist  = math.sqrt(dx*dx + dy*dy + dz*dz)
    steps = max(1, int(math.ceil(dist / step_mm)))
    for i in range(1, steps + 1):
        t = i / steps
        yield (p1[0] + dx*t, p1[1] + dy*t, p1[2] + dz*t)


def move_to(robot, target_xyz, T0, label=""):
    """Iteratively move the robot TCP to target_xyz (world). Returns IK fail count."""
    cur = world_tcp(robot)
    cur_xyz = (cur[0, 3], cur[1, 3], cur[2, 3])
    fails = 0
    for (x, y, z) in lerp_pts(cur_xyz, target_xyz, STEP_MM):
        T_t = Mat([
            [T0[0, 0], T0[0, 1], T0[0, 2], x],
            [T0[1, 0], T0[1, 1], T0[1, 2], y],
            [T0[2, 0], T0[2, 1], T0[2, 2], z],
            [0, 0, 0, 1],
        ])
        q = solve_ik(robot, T_t)
        if q is None:
            fails += 1
        else:
            robot.setJoints(q)
        time.sleep(DELAY_S)
    if label:
        print(f"  {label} -> done  ({fails} IK fails)")
    return fails


# --------------------------------------------------------------------------
# Connect
# --------------------------------------------------------------------------
rdk    = Robolink(robodk_ip=HOST, port=PORT)
master = rdk.Item(MASTER_NAME, ITEM_TYPE_ROBOT)
if not master.Valid():
    raise RuntimeError(f"Robot '{MASTER_NAME}' not found in station.")

# Capture current TCP (orientation is preserved throughout the trajectory)
T0 = world_tcp(master)
x0, y0, z0 = T0[0, 3], T0[1, 3], T0[2, 3]
print(f"\nCurrent TCP  x={x0:.1f}  y={y0:.1f}  z={z0:.1f}")

# --------------------------------------------------------------------------
# Build square corners
# --------------------------------------------------------------------------
sy0 = y0 + SQ_DY0
sz0 = z0 + SQ_DZ0

corners = [
    (x0, sy0,           sz0),
    (x0, sy0 + SIDE_MM, sz0),
    (x0, sy0 + SIDE_MM, sz0 + SIDE_MM),
    (x0, sy0,           sz0 + SIDE_MM),
    (x0, sy0,           sz0),            # close the loop
]
print(f"\nSquare start corner: y={sy0:.1f}  z={sz0:.1f}")
print("Square corners (world):")
for i, c in enumerate(corners):
    print(f"  [{i}]  y={c[1]:.1f}  z={c[2]:.1f}")

# --------------------------------------------------------------------------
# Phase 1: move to start corner
# --------------------------------------------------------------------------
print("\n-- Phase 1: Moving to square start corner --")
move_to(master, corners[0], T0, label=f"Start corner (y={sy0:.1f}, z={sz0:.1f})")

# --------------------------------------------------------------------------
# Phase 2: execute the square
# --------------------------------------------------------------------------
print("\n-- Phase 2: Executing square trajectory --")
total_pts = 0
ik_fails  = 0
side_labels = ["+Y", "+Z", "-Y", "-Z"]

for seg, (c1, c2) in enumerate(zip(corners[:-1], corners[1:])):
    print(f"\nSide {seg+1}/4  ({side_labels[seg]})  "
          f"y:{c1[1]:.0f}->{c2[1]:.0f}  z:{c1[2]:.0f}->{c2[2]:.0f}")
    seg_pts = seg_fails = 0

    for (x, y, z) in lerp_pts(c1, c2, STEP_MM):
        T_target = Mat([
            [T0[0, 0], T0[0, 1], T0[0, 2], x],
            [T0[1, 0], T0[1, 1], T0[1, 2], y],
            [T0[2, 0], T0[2, 1], T0[2, 2], z],
            [0,        0,        0,        1],
        ])
        q = solve_ik(master, T_target)
        if q is None:
            seg_fails += 1
            ik_fails  += 1
            if seg_fails <= 3:
                print(f"  IK fail  y={y:.1f} z={z:.1f}")
        else:
            master.setJoints(q)
        seg_pts   += 1
        total_pts += 1
        time.sleep(DELAY_S)

    print(f"  Done  ({seg_pts} pts, {seg_fails} IK fails)")

print(f"\nTrajectory complete.  Total points: {total_pts}  IK failures: {ik_fails}")
