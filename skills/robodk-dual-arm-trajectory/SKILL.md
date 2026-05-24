---
name: robodk-dual-arm-trajectory
description: >
  Move dual robots in RoboDK along Cartesian trajectories (square, line, or any path)
  using iterative IK on the master, with the slave following automatically via master-slave sync.
  Encodes all hard-won pitfalls for sub-robots whose bases are NOT at world origin (e.g. SDA10F,
  NEXTAGE): correct base-frame conversion, flat sol.tolist() parsing, joint seeding for branch
  continuity, and workspace probing before committing to a trajectory.

  ALWAYS use this skill when the user asks to:
  - move the master robot along a path / trajectory / shape (square, circle, line, etc.)
  - make dual robots move together / synchronously along a trajectory
  - reset the robots to their initial / home pose
  - probe the reachable workspace before executing a motion
  - run any iterative IK motion on a sub-robot in RoboDK
  Also trigger on: "draw a square", "trace a path", "move the arm along X",
  "make both arms follow a shape", "go back to initial pose", "reset the robots".
---

# RoboDK Dual-Arm Trajectory Skill

This skill drives a master robot along a Cartesian trajectory while the slave follows
via the MCP master-slave sync daemon. It encodes all pitfalls specific to sub-robots
whose bases are not at the world origin.

---

## Execution Bridge: MATLAB MCP

The bash sandbox cannot reach RoboDK's TCP API. Always execute Python scripts by
writing them to the outputs folder and running them through the MATLAB MCP:

```matlab
[status, out] = system('python "C:\path\to\script.py" 2>&1');
disp(out);
```

Never use mcp__robodk__solve_ik or mcp__robodk__move_linear for sub-robots --
they pass world-frame poses to SolveIK without the required base-frame conversion,
which silently fails every time.

---

## Critical Frame Rules (READ BEFORE WRITING ANY IK CODE)

### Rule 1 -- SolveIK expects a BASE-FRAME pose, not world-frame

Sub-robots (arms of SDA10F, NEXTAGE, etc.) have bases that are NOT at world origin.
RoboDK's r.SolveIK(T) always interprets T relative to the robot's own base frame.
Convert world-frame targets before passing them:

```python
parent = robot.Parent()                        # e.g. "Right Arm" item
base   = parent.PoseAbs() if parent.Valid() else Mat()
T_base = invH(base) * T_target_world          # <- always do this
sol    = robot.SolveIK(T_base, q_seed)
```

### Rule 2 -- sol.tolist() returns a FLAT list of joint angles

Unlike the MCP tool which returns nested JSON, the Python API returns a Mat whose
tolist() gives a flat list: [-3.7e-14, -60.0, 0.0, -80.0, ...].
Do NOT index it with [0] -- that gives the first joint scalar, not a joint array.

```python
sol    = robot.SolveIK(T_base, q_seed)
joints = sol.tolist()                 # flat: [j0, j1, j2, ...]
if not joints or len(joints) < 6:
    return None
return joints
```

### Rule 3 -- Seed IK from current joints to stay on the same branch

Without a seed, RoboDK's redundancy resolver may jump to a distant IK branch
between steps, causing jerks or failures. Always pass current joints as seed:

```python
q_seed = robot.Joints().list()
sol    = robot.SolveIK(T_base, q_seed)
```

### Rule 4 -- World TCP reconstruction

Use FK + tool + base to get the true world TCP (never Pose() * PoseAbs()):

```python
def world_tcp(robot):
    parent = robot.Parent()
    base   = parent.PoseAbs() if parent.Valid() else Mat()
    return base * robot.SolveFK(robot.Joints()) * robot.PoseTool()
```

---

## Standard solve_ik Helper

Copy this function into every trajectory script:

```python
from robodk.robomath import Mat, invH

def solve_ik(robot, T_target_world):
    parent = robot.Parent()
    base   = parent.PoseAbs() if parent.Valid() else Mat()
    T_base = invH(base) * T_target_world
    q_seed = robot.Joints().list()
    sol    = robot.SolveIK(T_base, q_seed)
    try:
        joints = sol.tolist()
        if not joints or len(joints) < 6:
            return None
        return joints
    except Exception:
        return None
```

---

## Bundled Scripts

The scripts/ folder contains ready-to-run utilities. Copy them to the outputs
folder and run via MATLAB MCP.

| Script                | Purpose                                              |
|-----------------------|------------------------------------------------------|
| reset_pose.py         | Reset both arms to the saved initial joints          |
| probe_workspace.py    | Sweep the ZY plane and print a reachability map      |
| square_trajectory.py  | Execute a configurable square trajectory             |

### Workflow for a trajectory request

1. Reset   -- run reset_pose.py to start from a known configuration.
2. Probe   -- run probe_workspace.py to see which side lengths and offsets
              are reachable from the current TCP. Prints a grid map and a table
              of maximum reachable square sizes.
3. Configure -- edit SIDE_MM, SQ_DY0, SQ_DZ0 in square_trajectory.py based on
              the probe output. The offset parameters shift the start corner so
              all four corners land inside the reachable region.
4. Sync    -- ensure master-slave sync is running via mcp__robodk__start_master_slave_sync
              before executing the trajectory.
5. Execute -- run square_trajectory.py. Phase 1 moves to the start corner;
              Phase 2 executes the square. Watch the IK fail count per side.

---

## Workspace Geometry (SDA10F reference)

For the Motoman SDA10F with arms at home joints [0, -60, 0, -80, 0, -30, 0]:

- Right arm base in world: [192.5, 265, 1200], orientation Pose(-90, 0, 180 deg)
- Home TCP in world: [766, -3, 1200]
- Reachable region in ZY world plane (x=766): roughly a circle of radius ~400 mm
  centred at world (y=265, z=1200). The home TCP is 268 mm from this centre.
- Max square from home TCP: 350 mm per side (all corners reachable)
- 600 mm square: achievable by offsetting start corner (SQ_DY0=-200, SQ_DZ0=-250)
- 300 mm square: centred on home TCP with (SQ_DY0=-150, SQ_DZ0=-150)

Always re-run probe_workspace.py when working with a different robot or starting pose.

---

## Key Config Parameters in square_trajectory.py

```python
MASTER_NAME = "Right SDA10F"   # robot item name in the station
SIDE_MM     = 300.0            # square side length (mm)
STEP_MM     = 5.0              # Cartesian step size per IK call (keep <= 5 mm)
DELAY_S     = 0.04             # pause between steps (~25 fps)
SQ_DY0      = -150.0           # start corner Y offset from current TCP (world)
SQ_DZ0      = -150.0           # start corner Z offset from current TCP (world)
HOST        = "localhost"
PORT        = 20500
```

---

## Adapting to Other Shapes

The same solve_ik + lerp_pts pattern works for any path. Replace the corners list
with waypoints for a circle, line, or arbitrary path. Invariants to preserve:

- Keep TCP orientation fixed (copy the rotation block from T0).
- Interpolate in small steps (STEP_MM <= 5 mm).
- Always seed IK from current joints.
- Verify all waypoints with the workspace probe first.
- Use only plain ASCII in print() calls -- Windows CP1252 rejects box-drawing
  characters and Unicode arrows.

---

## Troubleshooting

| Symptom                      | Cause                            | Fix                                   |
|------------------------------|----------------------------------|---------------------------------------|
| 100% IK failures             | World-frame pose to SolveIK      | Apply invH(base) * T_world            |
| sol.tolist()[0] is a float   | Flat list indexed as nested      | Use sol.tolist() directly             |
| IK branch jumps / jerks      | No joint seeding                 | Pass q_seed = robot.Joints().list()   |
| Slave does not follow        | Sync daemon not running          | Start sync before trajectory          |
| Corners out of reach         | Square too large or wrong offset | Run probe, adjust offsets             |
| UnicodeEncodeError (Windows) | Non-ASCII chars in print()       | Use plain ASCII only                  |
