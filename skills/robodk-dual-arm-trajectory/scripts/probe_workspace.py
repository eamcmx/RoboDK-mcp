"""
probe_workspace.py
==================
Probes the reachable envelope of the Right SDA10F arm in the world ZY plane.
Sweeps world Y and Z at constant X (the arm's current X) and checks IK
feasibility using the correct base-frame conversion.

Outputs:
  - ASCII reachability map (50 mm grid)
  - Table of max reachable square side from current TCP

Run via MATLAB MCP:
    [s, o] = system('python "path\\to\\probe_workspace.py" 2>&1'); disp(o);
"""

import math
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk.robomath import Mat, invH

HOST = "localhost"
PORT = 20500

rdk    = Robolink(robodk_ip=HOST, port=PORT)
master = rdk.Item("Right SDA10F", ITEM_TYPE_ROBOT)

parent = master.Parent()
base   = parent.PoseAbs()


def world_tcp(robot):
    parent = robot.Parent()
    b      = parent.PoseAbs() if parent.Valid() else Mat()
    return b * robot.SolveFK(robot.Joints()) * robot.PoseTool()


def try_ik(T_world):
    """Returns joint list if reachable, None if not."""
    T_base = invH(base) * T_world
    sol = master.SolveIK(T_base)
    try:
        j = sol.tolist()
        return j if j and len(j) >= 6 else None
    except Exception:
        return None


T0 = world_tcp(master)
x0, y0, z0 = T0[0, 3], T0[1, 3], T0[2, 3]
R0 = [[T0[i, j] for j in range(3)] for i in range(3)]

print(f"Start: x={x0:.1f}  y={y0:.1f}  z={z0:.1f}")

# Sweep +/-300 mm to +700 mm relative to start in 50 mm steps
results = {}
y_range = range(-300, 701, 50)
z_range = range(-300, 701, 50)

for dy in y_range:
    for dz in z_range:
        T_t = Mat([
            [R0[0][0], R0[0][1], R0[0][2], x0],
            [R0[1][0], R0[1][1], R0[1][2], y0 + dy],
            [R0[2][0], R0[2][1], R0[2][2], z0 + dz],
            [0, 0, 0, 1]
        ])
        results[(dy, dz)] = try_ik(T_t) is not None

# Print reachability grid
print("\nReachability map ('+' = reachable, '.' = not)")
print("dy \\ dz:", "  ".join(f"{dz:+4d}" for dz in z_range))
for dy in y_range:
    row = "  ".join("  + " if results[(dy, dz)] else "  . " for dz in z_range)
    print(f"dy={dy:+4d}: {row}")

# Table of max reachable square from start
print("\n\nMax reachable square sides from start (0,0):")
for side in [600, 500, 400, 350, 300, 250, 200]:
    corners = [(0, 0), (side, 0), (side, side), (0, side)]
    all_ok = True
    for (dy, dz) in corners:
        T_t = Mat([
            [R0[0][0], R0[0][1], R0[0][2], x0],
            [R0[1][0], R0[1][1], R0[1][2], y0 + dy],
            [R0[2][0], R0[2][1], R0[2][2], z0 + dz],
            [0, 0, 0, 1]
        ])
        if try_ik(T_t) is None:
            all_ok = False
            break
    print(f"  {side}mm: {'REACHABLE' if all_ok else 'out of reach'}")

print("\nDone.")
