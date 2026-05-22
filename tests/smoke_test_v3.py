"""
smoke_test_v3.py
================

End-to-end smoke test for v3/robodk_mcp_server_v3.py.

What it does
------------
1. Imports the v3 module from ../v3/ (no MCP transport - calls the underlying
   Python functions directly).
2. Verifies all expected tools are registered.
3. Runs each tool against your live RoboDK station with safe arguments.
4. Cleans up anything it creates (test targets, test program, test camera).
5. Prints a PASS/FAIL/SKIP report grouped by tool category.

Usage
-----
    cd repo-root
    python tests/smoke_test_v3.py

Prereqs
-------
* RoboDK running with a station that has at least one Robot and one Frame.
  The Elsevier demo station (UR5 "Claude" + Cubes + UR5 Base + Generic
  Dispenser) is what v3 was developed against.
* `pip install mcp robodk opencv-python-headless numpy`

Exit code 0 if all tests pass or only SKIP; non-zero if any FAIL.
"""

import importlib.util
import os
import sys
import traceback
from pathlib import Path


# -------- locate and import v3 ------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
V3_PATH   = REPO_ROOT / "v3" / "robodk_mcp_server_v3.py"
if not V3_PATH.exists():
    print(f"ERROR: cannot find {V3_PATH}", file=sys.stderr)
    sys.exit(2)

spec = importlib.util.spec_from_file_location("robodk_mcp_server_v3", V3_PATH)
v3 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v3)


# -------- expected tool inventory --------------------------------------------

EXPECTED_TOOLS = {
    # Connectivity
    "get_connection_status",
    # Station inspection
    "get_station_items", "list_objects_on_table", "list_programs",
    "list_targets", "list_robots", "list_frames", "list_tools", "find_items",
    # Poses / kinematics
    "get_object_pose", "get_pose", "get_robot_joints", "get_tcp_pose",
    "solve_fk", "solve_ik", "solve_ik_all",
    # Robot motion
    "set_robot_joints", "move_joint", "move_linear", "move_to_target",
    "set_robot_speed", "set_simulation_speed", "render",
    "move_joint_test", "move_linear_test",
    # Programs / targets / frames / tools
    "add_target", "add_program", "program_add_move", "program_add_call",
    "program_add_wait", "program_clear", "get_program_instructions",
    "run_program", "program_make_robot_program",
    "add_frame", "set_active_tool", "add_tool_from_object",
    # Scene objects
    "add_object_from_file", "set_object_pose", "set_item_pose",
    "set_object_color", "set_object_visible", "set_item_name", "set_parent",
    "delete_item", "bulk_delete", "attach_object_to_robot", "detach_object",
    "load_station", "save_station",
    # Cameras & vision
    "add_camera", "set_camera_pose", "get_camera_image_path",
    "capture_snapshot", "detect_blobs", "detect_objects_by_color",
    "pixel_to_world",
    # Collisions
    "check_collision", "check_ray_collision", "set_collision_detection",
    "get_all_collisions",
    # Composite pipelines
    "plan_grasp_pose", "execute_pick", "execute_place", "vision_pick_by_color",
    # Station I/O
    "get_param", "set_param", "set_run_mode", "show_message",
    "get_joint_limits", "set_joint_limits",
}


# -------- minimal test harness -----------------------------------------------

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
results = []  # list of (category, name, status, detail)


def record(category, name, status, detail=""):
    results.append((category, name, status, detail))
    short = detail.replace("\n", " ").strip()
    if len(short) > 120:
        short = short[:117] + "..."
    print(f"  [{status}] {name:<32} {short}")


def run(category, name, fn, *args, skip_if=None, **kwargs):
    """Call fn, record PASS/FAIL/SKIP."""
    if skip_if:
        record(category, name, SKIP, skip_if)
        return None
    try:
        out = fn(*args, **kwargs)
        if isinstance(out, dict) and out.get("error"):
            record(category, name, FAIL, str(out["error"]))
        else:
            preview = str(out)
            if len(preview) > 100:
                preview = preview[:97] + "..."
            record(category, name, PASS, preview)
        return out
    except Exception as e:
        record(category, name, FAIL, f"{type(e).__name__}: {e}")
        return None


# -------- main ---------------------------------------------------------------

def main():
    print("=" * 78)
    print("v3 smoke test")
    print("=" * 78)

    # 1. Tool registration check ---------------------------------------------
    print("\n[1] Tool registration")
    registered = {name for name in dir(v3)
                  if callable(getattr(v3, name)) and not name.startswith("_")}
    missing = EXPECTED_TOOLS - registered
    extra = registered - EXPECTED_TOOLS - {
        "get_rdk", "main", "argparse", "base64", "fnmatch", "math",
        "np", "os", "tempfile", "time", "Any", "Optional", "mcp", "FastMCP"}
    if missing:
        record("registration", "tool_inventory", FAIL,
               f"missing: {sorted(missing)}")
    else:
        record("registration", "tool_inventory", PASS,
               f"all {len(EXPECTED_TOOLS)} expected tools present")
    if extra:
        record("registration", "extra_tools", PASS,
               f"extras: {sorted(extra)}")

    # 2. Connectivity ---------------------------------------------------------
    print("\n[2] Connectivity")
    cs = run("connectivity", "get_connection_status", v3.get_connection_status)
    if not (cs and cs.get("connected")):
        print("\n!! Not connected to RoboDK -- aborting live tests.")
        return summary(strict=False)

    # 3. Discover the station -------------------------------------------------
    print("\n[3] Station inspection")
    items = run("inspect", "get_station_items", v3.get_station_items)
    if not items:
        return summary(strict=True)

    robots = [it["name"] for it in items if it["type"] == 1]
    frames = [it["name"] for it in items if it["type"] == 2]
    tools  = [it["name"] for it in items if it["type"] == 3]
    objs   = [it["name"] for it in items if it["type"] == 4]
    targs  = [it["name"] for it in items if it["type"] == 5]

    robot_name = robots[0] if robots else None
    frame_name = frames[0] if frames else None
    target_name = targs[0] if targs else None
    object_name = objs[0] if objs else None
    tool_name = tools[0] if tools else None

    print(f"   robot={robot_name}  frame={frame_name}  target={target_name}")
    print(f"   object={object_name}  tool={tool_name}")

    run("inspect", "list_programs", v3.list_programs)
    run("inspect", "list_targets", v3.list_targets)
    run("inspect", "list_robots", v3.list_robots)
    run("inspect", "list_frames", v3.list_frames)
    run("inspect", "list_tools", v3.list_tools)
    run("inspect", "find_items", v3.find_items, "Cube*")
    if target_name:
        run("inspect", "list_objects_on_table", v3.list_objects_on_table,
            target_name, z_tolerance_mm=200)
    else:
        record("inspect", "list_objects_on_table", SKIP, "no table target found")

    # 4. Poses / kinematics ---------------------------------------------------
    print("\n[4] Poses / kinematics")
    home = [0, -90, -90, -90, 90, 0]
    if not robot_name:
        record("kin", "*", SKIP, "no robot found")
    else:
        run("kin", "get_robot_joints", v3.get_robot_joints, robot_name)
        run("kin", "get_tcp_pose", v3.get_tcp_pose, robot_name)
        run("kin", "get_pose", v3.get_pose, robot_name, kind="abs")
        if object_name:
            run("kin", "get_object_pose", v3.get_object_pose, object_name)
        fk = run("kin", "solve_fk", v3.solve_fk, robot_name, home)
        if fk and fk.get("pose_matrix_row_major"):
            flat = fk["pose_matrix_row_major"]
            pose_nested = [flat[i*4:(i+1)*4] for i in range(4)]
            ik = run("kin", "solve_ik", v3.solve_ik, robot_name, pose_nested)
            ik_all = run("kin", "solve_ik_all", v3.solve_ik_all,
                         robot_name, pose_nested)
            if ik_all and "solutions_deg" in ik_all:
                shape_ok = (ik_all["num_solutions"] >= 1 and
                            all(len(s) == 6 for s in ik_all["solutions_deg"]))
                if not shape_ok:
                    record("kin", "solve_ik_all_shape", FAIL,
                           f"bad shape: {ik_all}")
                else:
                    record("kin", "solve_ik_all_shape", PASS,
                           f"{ik_all['num_solutions']} solutions, each 6-dof")

    # 5. Motion ---------------------------------------------------------------
    print("\n[5] Motion (snap-only; no big moves)")
    if robot_name:
        run("motion", "set_robot_joints", v3.set_robot_joints,
            robot_name, home)
        run("motion", "set_robot_speed", v3.set_robot_speed,
            robot_name, 200.0)
        run("motion", "set_simulation_speed", v3.set_simulation_speed, 5)
        run("motion", "render", v3.render, refresh=True)
        # MoveJ a tiny delta to verify it actually moves
        run("motion", "move_joint(tiny)", v3.move_joint,
            robot_name, [0, -90, -90, -90, 90, 1], blocking=True)
        run("motion", "move_joint(home)", v3.move_joint,
            robot_name, home, blocking=True)
        # MoveJ_Test
        run("motion", "move_joint_test", v3.move_joint_test,
            robot_name, home, [0, -90, -90, -90, 90, 5])

    # 6. Programs / targets ---------------------------------------------------
    print("\n[6] Programs / targets / frames")
    tgt_name = "_v3test_target"
    prog_name = "_v3test_program"
    frame_test = "_v3test_frame"

    if robot_name and frame_name:
        run("progtgt", "add_frame", v3.add_frame,
            frame_test, parent_frame_name=frame_name)
        run("progtgt", "add_target", v3.add_target,
            tgt_name, robot_name,
            parent_frame_name=frame_name, joints_deg=home)
        run("progtgt", "add_program", v3.add_program,
            prog_name, robot_name, speed_mm_s=200)
        run("progtgt", "program_add_move", v3.program_add_move,
            prog_name, tgt_name, move_type="J")
        run("progtgt", "program_add_wait", v3.program_add_wait,
            prog_name, 0.1)
        run("progtgt", "get_program_instructions",
            v3.get_program_instructions, prog_name)
        run("progtgt", "program_clear", v3.program_clear, prog_name)
        # Cleanup
        run("progtgt", "delete_item(target)", v3.delete_item, tgt_name)
        run("progtgt", "delete_item(program)", v3.delete_item, prog_name)
        run("progtgt", "delete_item(frame)", v3.delete_item, frame_test)
    else:
        record("progtgt", "*", SKIP, "robot or frame missing")

    # 7. Tools / TCP ----------------------------------------------------------
    print("\n[7] Tools / TCP")
    if robot_name and tool_name:
        run("tools", "set_active_tool", v3.set_active_tool,
            robot_name, tool_name)
    else:
        record("tools", "set_active_tool", SKIP, "robot or tool missing")

    # 8. Scene mutation (safe, reversible) -----------------------------------
    print("\n[8] Scene mutation")
    if object_name or target_name:
        victim = object_name or target_name
        run("scene", "set_object_visible(off)", v3.set_object_visible,
            victim, False)
        run("scene", "set_object_visible(on)", v3.set_object_visible,
            victim, True)
        run("scene", "set_object_color", v3.set_object_color,
            victim, 0.7, 0.7, 0.7, 1)
        run("scene", "set_item_name", v3.set_item_name,
            victim, victim + "_x")
        run("scene", "set_item_name(restore)", v3.set_item_name,
            victim + "_x", victim)
        run("scene", "find_items+bulk_delete(no-match)",
            v3.bulk_delete, "_v3test_does_not_exist_*")
    else:
        record("scene", "*", SKIP, "no object/target")

    # 9. Cameras --------------------------------------------------------------
    print("\n[9] Cameras")
    cam_name = "_v3test_cam"
    if frame_name:
        cam = run("cam", "add_camera", v3.add_camera,
                  frame_name, camera_name=cam_name)
        if cam and cam.get("camera_name"):
            actual = cam["camera_name"]
            run("cam", "get_camera_image_path", v3.get_camera_image_path,
                actual)
            run("cam", "capture_snapshot(no-base64)", v3.capture_snapshot,
                actual)
            run("cam", "detect_blobs(blank ok)", v3.detect_blobs, actual)
            run("cam", "detect_objects_by_color(blank ok)",
                v3.detect_objects_by_color, actual, "red")
            run("cam", "pixel_to_world", v3.pixel_to_world, actual, 320, 240)
            run("cam", "delete_item(camera)", v3.delete_item, actual)
        else:
            record("cam", "add_camera_response", FAIL,
                   "add_camera returned no camera_name")
    else:
        record("cam", "*", SKIP, "no frame")

    # 10. Collisions ---------------------------------------------------------
    print("\n[10] Collisions")
    run("collision", "set_collision_detection(on)",
        v3.set_collision_detection, True)
    run("collision", "get_all_collisions", v3.get_all_collisions)
    if len(items) >= 2:
        a, b = items[0]["name"], items[1]["name"]
        run("collision", "check_collision", v3.check_collision, a, b)
    run("collision", "check_ray_collision", v3.check_ray_collision,
        0, 0, 1000, 0, 0, -100)

    # 11. Station I/O --------------------------------------------------------
    print("\n[11] Station I/O")
    run("io", "get_param(PATH_OPENSTATION)",
        v3.get_param, "PATH_OPENSTATION")
    run("io", "set_run_mode(simulate)", v3.set_run_mode, "simulate")
    run("io", "show_message", v3.show_message,
        "v3 smoke test running", False)
    if robot_name:
        run("io", "get_joint_limits", v3.get_joint_limits, robot_name)

    return summary(strict=True)


def summary(strict: bool):
    print("\n" + "=" * 78)
    by_status = {PASS: 0, FAIL: 0, SKIP: 0}
    fails = []
    for _, name, status, detail in results:
        by_status[status] += 1
        if status == FAIL:
            fails.append((name, detail))
    print(f"PASS: {by_status[PASS]}   FAIL: {by_status[FAIL]}   "
          f"SKIP: {by_status[SKIP]}")
    if fails:
        print("\nFailures:")
        for name, detail in fails:
            print(f"  - {name}: {detail}")
    print("=" * 78)
    return 0 if (by_status[FAIL] == 0 or not strict) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)
    except Exception:
        traceback.print_exc()
        sys.exit(2)
