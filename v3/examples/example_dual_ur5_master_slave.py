"""
example_dual_ur5_master_slave.py
================================
End-to-end worked example: load the bundled ``Dual UR5t.rdk`` station, capture
the tool-to-tool offset, then sync the slave (UR52) to the master (UR51) as
the user drags / jogs the master in RoboDK.

Drop this file in ``v3/examples/`` and the station file in
``v3/examples/stations/`` of the RoboDK-MCP repo. Run from a terminal on the
host running RoboDK::

    pip install robodk
    python v3/examples/example_dual_ur5_master_slave.py

Then drag or jog UR51 in RoboDK; UR52 will mirror it preserving the offset
captured at start-up. Ctrl+C to stop.
"""

from __future__ import annotations

import os
import sys
import time

try:
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import Mat, invH
except ImportError:
    sys.stderr.write("robodk package not found. Install it with: pip install robodk\n")
    sys.exit(1)


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
STATION_REL = os.path.join("stations", "Dual UR5t.rdk")
MASTER_NAME = "UR51"
SLAVE_NAME  = "UR52"
LOOP_HZ     = 30.0
MAX_REACH_MM = 850.0      # UR5 / UR5e
POSITION_EPS_MM = 0.05


# --------------------------------------------------------------------------
# Helpers (apply the four frame-handling rules from the MCP instructions)
# --------------------------------------------------------------------------
def world_tcp(r) -> Mat:
    parent = r.Parent()
    base_in_world = parent.PoseAbs() if parent.Valid() else Mat()
    return base_in_world * r.SolveFK(r.Joints()) * r.PoseTool()


def reach_ok(slave, target_world: Mat, max_reach_mm: float) -> bool:
    sb = slave.Parent().PoseAbs()
    bx, by, bz = sb[0, 3], sb[1, 3], sb[2, 3]
    tx, ty, tz = target_world[0, 3], target_world[1, 3], target_world[2, 3]
    return ((bx-tx)**2 + (by-ty)**2 + (bz-tz)**2) ** 0.5 <= max_reach_mm


def pose_changed(a: Mat, b: Mat, eps_mm: float = POSITION_EPS_MM) -> bool:
    dx, dy, dz = a[0,3]-b[0,3], a[1,3]-b[1,3], a[2,3]-b[2,3]
    if dx*dx + dy*dy + dz*dz > eps_mm * eps_mm:
        return True
    dot = a[0,0]*b[0,0] + a[1,0]*b[1,0] + a[2,0]*b[2,0]
    return dot < 1.0 - 1e-6


# --------------------------------------------------------------------------
def main() -> int:
    rdk = Robolink()

    # Load the bundled station if it's not already open
    here = os.path.dirname(os.path.abspath(__file__))
    station_path = os.path.join(here, STATION_REL)
    if os.path.exists(station_path):
        # AddFile opens the .rdk and makes its station active
        rdk.AddFile(station_path)
    else:
        print(f"[warn] {station_path} not found; using whatever station is currently open.")

    master = rdk.Item(MASTER_NAME, ITEM_TYPE_ROBOT)
    slave  = rdk.Item(SLAVE_NAME,  ITEM_TYPE_ROBOT)
    if not master.Valid() or not slave.Valid():
        sys.exit(f"Could not find {MASTER_NAME}/{SLAVE_NAME} in the station.")

    # Rule 1.2: pin slave IK to its own base
    slave.setPoseFrame(slave.Parent())

    # Capture the locked tool-to-tool offset from current poses
    Tm0 = world_tcp(master)
    Ts0 = world_tcp(slave)
    T_rel = invH(Tm0) * Ts0
    slave_base_inv = invH(slave.Parent().PoseAbs())

    print(f"[init] master TCP world = ({Tm0[0,3]:8.3f}, {Tm0[1,3]:8.3f}, {Tm0[2,3]:8.3f})")
    print(f"[init] slave  TCP world = ({Ts0[0,3]:8.3f}, {Ts0[1,3]:8.3f}, {Ts0[2,3]:8.3f})")
    print(f"[init] tool-to-tool distance = "
          f"{((Tm0[0,3]-Ts0[0,3])**2+(Tm0[1,3]-Ts0[1,3])**2+(Tm0[2,3]-Ts0[2,3])**2)**0.5:.3f} mm")
    print(f"[init] Syncing {SLAVE_NAME} <- {MASTER_NAME} at {LOOP_HZ:.0f} Hz. Ctrl+C to stop.\n")

    period = 1.0 / LOOP_HZ
    last_master = Tm0
    tick = 0
    try:
        while True:
            t0 = time.perf_counter()
            Tm = world_tcp(master)
            if pose_changed(Tm, last_master):
                Ts_w = Tm * T_rel
                # Reach precheck (saves a confusing "unreachable" spam)
                if not reach_ok(slave, Ts_w, MAX_REACH_MM):
                    if tick % int(LOOP_HZ) == 0:
                        print(f"[{tick:>5}] out of slave reach at master="
                              f"({Tm[0,3]:7.2f},{Tm[1,3]:7.2f},{Tm[2,3]:7.2f})")
                else:
                    Ts_b = slave_base_inv * Ts_w
                    seed = slave.Joints().list()
                    jq = slave.SolveIK(Ts_b, seed)
                    jl = jq.list() if hasattr(jq, "list") else list(jq)
                    if len(jl) == 6 and not any(v != v for v in jl):
                        slave.setJoints(jl)
                        last_master = Tm
                        if tick % int(LOOP_HZ) == 0:
                            print(f"[{tick:>5}] master=({Tm[0,3]:7.2f},{Tm[1,3]:7.2f},{Tm[2,3]:7.2f}) "
                                  f"slave_tgt=({Ts_w[0,3]:7.2f},{Ts_w[1,3]:7.2f},{Ts_w[2,3]:7.2f})")
                    else:
                        if tick % int(LOOP_HZ) == 0:
                            print(f"[{tick:>5}] ik_failed at master="
                                  f"({Tm[0,3]:7.2f},{Tm[1,3]:7.2f},{Tm[2,3]:7.2f})")
            tick += 1
            dt = period - (time.perf_counter() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
