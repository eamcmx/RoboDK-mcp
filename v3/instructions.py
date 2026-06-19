"""
RoboDK MCP -- server instructions
=================================
Exposed as ``DUAL_ROBOT_INSTRUCTIONS`` and passed to ``FastMCP(..., instructions=...)``
in ``robodk_mcp_server_v3.py`` so every LLM that loads this MCP sees them.

Scope: dual-robot patterns (master-slave TCP sync, handover, dual-arm
pick-and-place, collision-aware coordinated motion) plus the MATLAB-style
Newton-Jacobian IK fallback. Distilled from real debugging sessions, not
generic theory.

Keep edits below the section headers; tooling parses the markdown for the
README.
"""

DUAL_ROBOT_INSTRUCTIONS = r"""# RoboDK MCP -- Dual-Robot Coordination

Patterns for multi-robot work in this server. Apply the four frame-handling
rules in section 1 to every script before reaching for kinematics; they
eliminate ~90 percent of the silent failures in dual-robot setups.

## 1. Frame-handling rules (READ FIRST)

### 1.1 World TCP must come from joints + base, never from `Pose() * PoseAbs()`

```python
def world_tcp(r):
    parent = r.Parent()                            # 'UR51 Base', etc.
    base_in_world = parent.PoseAbs() if parent.Valid() else Mat()
    tcp_in_base   = r.SolveFK(r.Joints()) * r.PoseTool()
    return base_in_world * tcp_in_base
```

`r.Pose()` returns TCP relative to the **active user reference frame** which
may not be the robot's own base; multiplying by `r.PoseAbs()` then produces a
compound that is mathematically meaningless when frames cross.

### 1.2 Pin slave IK to the slave's own base before solving

```python
slave.setPoseFrame(slave.Parent())
```

Otherwise `SolveIK` may interpret the target in another robot's frame.

### 1.3 Convert master-derived targets into the slave's base before IK

```python
from robodk.robomath import invH
slave_base_inv  = invH(slave.Parent().PoseAbs())
Ts_target_base  = slave_base_inv * Ts_target_world
```

### 1.4 Seed IK with the slave's current joints

```python
jq = slave.SolveIK(Ts_target_base, slave.Joints().list())
```

Without a seed, RoboDK may flip branches between ticks and the slave will
jerk.

## 2. Locked tool-to-tool offset (master-slave sync)

Core pattern. Capture the offset once, apply it every tick.

```python
# Once, at start-up
T_rel = invH(world_tcp(master)) * world_tcp(slave)

# Every tick
Tm   = world_tcp(master)
Ts_w = Tm * T_rel
Ts_b = invH(slave.Parent().PoseAbs()) * Ts_w
slave.setJoints(slave.SolveIK(Ts_b, slave.Joints().list()).list())
```

`T_rel` is purely tool-to-tool -- moving either base afterwards does not
change `T_rel`, only the slave's reachability.

### Reach precheck (do this before claiming the IK is broken)

```python
import math
sb = slave.Parent().PoseAbs()
if math.dist((sb[0,3], sb[1,3], sb[2,3]),
             (Ts_w[0,3], Ts_w[1,3], Ts_w[2,3])) > MAX_REACH:
    # Move the slave base closer or constrain the master workspace.
    ...
```

UR5 / UR5e reach is approximately 850 mm.  Slave bases more than 1700 mm from
the master workspace cannot share a captured offset.

## 3. Dual-arm handover

1. Approach -- both robots pre-position.
2. Lock -- capture `T_rel`, run master-slave sync while object is dual-grasped.
3. Release -- one robot detaches; use `attach_object_to_robot` /
   `detach_object`.

```python
T_rel = invH(world_tcp(master)) * world_tcp(slave)
for Tm in trajectory_world:
    move_master_to(Tm)
    sync_slave(Tm * T_rel)
slave.DetachAll()
```

## 4. Dual-arm pick-and-place

Two robots picking different objects from different bins into a shared
destination. Independent motions but co-planned to avoid mutual collision.

1. Plan arm A's trajectory.
2. Sample arm A's swept volume at N waypoints.
3. Plan arm B's trajectory treating those waypoints as time-varying obstacles.
4. Execute on a shared time base. Use `move_joint_test` / `move_linear_test`
   to pre-validate each segment.

For tight clearances, gate one arm on the other's progress (barrier
synchronisation) rather than running them fully parallel.

## 5. Collision-aware coordinated motion

```python
from robodk.robolink import COLLISION_ON
RDK.setCollisionActive(COLLISION_ON)
```

Then before every synchronised move, run it in simulation and check
`get_all_collisions()`. Re-plan or insert a retreat step if the pair count is
nonzero. For master-slave sync specifically: if `T_rel` brings the slave too
close at certain configurations, bound the master workspace or compose a
retreat translation into `T_rel`.

## 6. Newton-Jacobian IK fallback

If RoboDK's `SolveIK` returns empty or NaN joints (branch failure, singular
target), fall back to a numerical Newton-Raphson solver with the 12-component
residual used in the MATLAB CD_UR5 / NewtonJ pipeline.

```python
import numpy as np

def newton_ik(forward_fn, T_target, q_seed,
              max_iter=100, tol=1e-9, damp=1.0):
    q = np.array(q_seed, dtype=float)
    def err(q_vec):
        T = forward_fn(q_vec)
        e = np.empty(12)
        e[0:9]  = (T[:3, :3] - T_target[:3, :3]).flatten()
        e[9:12] = T[:3, 3] - T_target[:3, 3]
        return e
    for _ in range(max_iter):
        f0 = err(q)
        if np.max(np.abs(f0)) < tol:
            return (((q + 180.0) % 360.0) - 180.0).tolist()
        J = np.empty((12, 6))
        for k in range(6):
            qp = q.copy(); qp[k] += damp
            J[:, k] = (err(qp) - f0) / damp
        q = q + np.linalg.pinv(J) @ (-f0)
    return None
```

`forward_fn` must return the pose in the same frame as `T_target` (i.e.
include the robot's base offset if `T_target` is in world).

## 7. Pitfalls observed in practice

| Symptom | Cause | Fix |
| --- | --- | --- |
| Slave never moves; loop silent | Drag in 3D view didn't commit joints | Jog joints from the joint panel |
| Slave snaps to a wrong pose on first tick | Hard-coded `T_rel` from a different base layout | Capture on startup |
| `slave unreachable` every tick | Slave base too far for the locked offset | Reach precheck (section 2); move slave base closer |
| `get_robot_joints` errors `'float' object is not iterable` | Wrapper bug returning a flat list | Use `robot.Joints().list()` from the robodk package, or upgrade to v3.1 |
| `get_tcp_pose` returns identical numbers for two robots | v3 bug: returns TCP in robot's own base, not world | Upgrade to v3.1 or compute world TCP via section 1.1 |
| Slave jerks / flips branches between ticks | IK called without seed | Always seed with current joints |
| `.robot` library import fails | Wrong filename | Try `UR5.robot`, `UR5e.robot`, `Universal Robots UR5.robot` |

## 8. Minimal reference script

```python
import time
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk.robomath import invH

MASTER, SLAVE, HZ = "UR51", "UR52", 30.0

def world_tcp(r):
    p = r.Parent()
    base = p.PoseAbs() if p.Valid() else type(r.Pose())()
    return base * r.SolveFK(r.Joints()) * r.PoseTool()

rdk = Robolink()
m, s = rdk.Item(MASTER, ITEM_TYPE_ROBOT), rdk.Item(SLAVE, ITEM_TYPE_ROBOT)
s.setPoseFrame(s.Parent())
T_rel       = invH(world_tcp(m)) * world_tcp(s)
s_base_inv  = invH(s.Parent().PoseAbs())

period, last = 1.0/HZ, None
while True:
    t0 = time.perf_counter()
    Tm = world_tcp(m)
    if last is None or any(abs(Tm[i,3] - last[i,3]) > 0.05 for i in range(3)):
        Ts_w = Tm * T_rel
        jq   = s.SolveIK(s_base_inv * Ts_w, s.Joints().list())
        jl   = jq.list() if hasattr(jq, 'list') else list(jq)
        if len(jl) == 6 and not any(v != v for v in jl):
            s.setJoints(jl); last = Tm
    dt = period - (time.perf_counter() - t0)
    if dt > 0: time.sleep(dt)
```
"""
