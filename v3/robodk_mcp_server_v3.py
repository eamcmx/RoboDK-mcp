"""
RoboDK MCP Server -- v3
=======================
Complete rewrite of the v2 server. Adds ~30 new tools and fixes 11 documented
bugs while keeping the v2 tool names backwards-compatible.

What's new vs v2
----------------
* Targets / Programs / Frames / Tools
    add_target, add_program, program_add_move, program_add_call, program_add_wait,
    program_clear, get_program_instructions, program_make_robot_program,
    add_frame, set_active_tool, add_tool_from_object
* Generic item handling
    get_pose, set_item_pose, set_item_name, set_parent
* Lists
    list_targets, list_robots, list_frames, list_tools, find_items, bulk_delete
* Motion testing
    move_joint_test, move_linear_test
* Station I/O
    save_station, get_param, set_param, set_run_mode, show_message
* Joint limits
    get_joint_limits, set_joint_limits

Bugs fixed
----------
1. solve_ik_all      -- returns N x 6 (v2 returned a flattened single solution).
2. get_all_collisions-- pairs list matches the count (v2 returned count=1, pairs=[]).
3. list_objects_on_table -- returns the real name, not "." for attached objects.
4. Robot name resolution -- accepts the display name ("Claude") not only the
   underlying robot name ("UR5"). All *_robot helpers normalise via _robot().
5. move_linear       -- pre-seeds from current joints, falls back through a
   MoveJ to align IK branch before MoveL. No more "Joint axes outside limits"
   on reachable targets.
6. move_to_target    -- looks up the target with ITEM_TYPE_TARGET so the
   target name no longer collides with an object of the same name.
7. add_camera        -- returns JSON-serializable metadata. Honours
   camera_name. (v2 raised "Object of type Item is not JSON serializable".)
8. add_camera        -- honours the camera_name argument (v2 ignored it).
9. capture_snapshot  -- base64 payload is opt-in. Default response is just
   the file path (v2 always returned ~400 KB and blew the context window).
10. detect_blobs / detect_objects_by_color -- fast early-exit on near-uniform
    scenes so a blank camera doesn't hang the LLM turn.
11. pixel_to_world   -- actually uses the camera frame's world pose when
    projecting (v2 returned multi-metre offsets for the image center).

Usage
-----
  python robodk_mcp_server_v3.py
  python robodk_mcp_server_v3.py --host X --port Y

Requirements
------------
  pip install mcp robodk opencv-python-headless numpy
"""

import argparse
import base64
import fnmatch
import math
import os
import tempfile
import time
from typing import Any, Optional

import numpy as np

from mcp.server.fastmcp import FastMCP


# ---------------------------------------------------------------------------
# Connection management
# ---------------------------------------------------------------------------

_RDK = None
_ROBODK_HOST = "localhost"
_ROBODK_PORT = 20500


def get_rdk():
    """Lazy, auto-reconnecting Robolink connection."""
    global _RDK
    from robodk.robolink import Robolink
    try:
        if _RDK is None:
            _RDK = Robolink(robodk_ip=_ROBODK_HOST, port=_ROBODK_PORT)
        # Cheap liveness ping
        _RDK.getParam("PATH_OPENSTATION")
        return _RDK
    except Exception:
        _RDK = Robolink(robodk_ip=_ROBODK_HOST, port=_ROBODK_PORT)
        return _RDK


# ---------------------------------------------------------------------------
# Item resolution -- fixes the "Claude vs UR5" name confusion in v2
# ---------------------------------------------------------------------------

def _item(name: str, item_type: Optional[int] = None):
    """Resolve an item by name, with a typed fallback search."""
    rdk = get_rdk()
    it = rdk.Item(name, item_type) if item_type is not None else rdk.Item(name)
    if it.Valid():
        return it
    if item_type is not None:
        for n in rdk.ItemList(item_type, 1):
            if str(n).lower() == name.lower():
                return rdk.Item(n, item_type)
    raise ValueError(f"Item '{name}' not found (type={item_type}).")


def _robot(name: str):
    from robodk.robolink import ITEM_TYPE_ROBOT
    return _item(name, ITEM_TYPE_ROBOT)


def _frame(name: str):
    from robodk.robolink import ITEM_TYPE_FRAME
    return _item(name, ITEM_TYPE_FRAME)


def _tool_item(name: str):
    from robodk.robolink import ITEM_TYPE_TOOL
    return _item(name, ITEM_TYPE_TOOL)


def _target(name: str):
    from robodk.robolink import ITEM_TYPE_TARGET
    return _item(name, ITEM_TYPE_TARGET)


def _program(name: str):
    from robodk.robolink import ITEM_TYPE_PROGRAM
    return _item(name, ITEM_TYPE_PROGRAM)


# ---------------------------------------------------------------------------
# Pose / matrix helpers
# ---------------------------------------------------------------------------

def _mat(data: Any):
    """4x4 nested list or 16-element flat list -> robodk Mat."""
    from robodk.robomath import Mat
    if isinstance(data, list):
        if data and isinstance(data[0], list):
            return Mat(data)
        if len(data) == 16:
            return Mat([data[i * 4:(i + 1) * 4] for i in range(4)])
    raise ValueError("pose must be 4x4 nested list or 16-element flat list")


def _pose_rows(pose) -> list:
    return [[round(v, 4) for v in row] for row in pose.Rows()]


def _pose_flat(pose) -> list:
    return [round(v, 4) for row in pose.Rows() for v in row]


def _xyz(pose) -> dict:
    p = pose.Pos()
    return {"x": round(p[0], 3), "y": round(p[1], 3), "z": round(p[2], 3)}


# ---------------------------------------------------------------------------
# FastMCP
# ---------------------------------------------------------------------------

mcp = FastMCP("robodk-v3")


# ===========================================================================
# 1. CONNECTIVITY
# ===========================================================================

@mcp.tool()
def get_connection_status() -> dict:
    """Check whether RoboDK is running and reachable."""
    try:
        rdk = get_rdk()
        station = rdk.getParam("PATH_OPENSTATION")
        return {"connected": True, "station": station,
                "host": _ROBODK_HOST, "port": _ROBODK_PORT}
    except Exception as e:
        return {"connected": False, "error": str(e),
                "host": _ROBODK_HOST, "port": _ROBODK_PORT}


# ===========================================================================
# 2. STATION INSPECTION
# ===========================================================================

@mcp.tool()
def get_station_items() -> list:
    """List every item in the station tree (name, type, valid)."""
    rdk = get_rdk()
    items = []
    for name in rdk.ItemList(-1, 1):
        it = rdk.Item(name)
        items.append({"name": str(name), "type": it.Type(),
                      "valid": it.Valid()})
    return items


@mcp.tool()
def list_objects_on_table(table_name: str,
                          z_tolerance_mm: float = 50) -> dict:
    """Find every item whose absolute Z is within z_tolerance of a table."""
    rdk = get_rdk()
    table = _item(table_name)
    table_z = table.PoseAbs().Pos()[2]
    found = []
    for name in rdk.ItemList(-1, 1):
        it = rdk.Item(name)
        if not it.Valid():
            continue
        try:
            z = it.PoseAbs().Pos()[2]
        except Exception:
            continue
        if abs(z - table_z) <= z_tolerance_mm:
            real_name = it.Name() or str(name)
            found.append({"name": real_name, "type": it.Type(),
                          "position_mm": _xyz(it.PoseAbs())})
    return {"table": table_name, "table_z_mm": round(table_z, 3),
            "objects_found": found}


@mcp.tool()
def list_programs() -> dict:
    """Names of all Program items."""
    from robodk.robolink import ITEM_TYPE_PROGRAM
    names = [str(n) for n in get_rdk().ItemList(ITEM_TYPE_PROGRAM, 1)]
    return {"programs": names, "count": len(names)}


@mcp.tool()
def list_targets() -> dict:
    """Names of all Target items."""
    from robodk.robolink import ITEM_TYPE_TARGET
    names = [str(n) for n in get_rdk().ItemList(ITEM_TYPE_TARGET, 1)]
    return {"targets": names, "count": len(names)}


@mcp.tool()
def list_robots() -> dict:
    """Names of all Robot items."""
    from robodk.robolink import ITEM_TYPE_ROBOT
    names = [str(n) for n in get_rdk().ItemList(ITEM_TYPE_ROBOT, 1)]
    return {"robots": names, "count": len(names)}


@mcp.tool()
def list_frames() -> dict:
    """Names of all reference Frame items."""
    from robodk.robolink import ITEM_TYPE_FRAME
    names = [str(n) for n in get_rdk().ItemList(ITEM_TYPE_FRAME, 1)]
    return {"frames": names, "count": len(names)}


@mcp.tool()
def list_tools() -> dict:
    """Names of all Tool items."""
    from robodk.robolink import ITEM_TYPE_TOOL
    names = [str(n) for n in get_rdk().ItemList(ITEM_TYPE_TOOL, 1)]
    return {"tools": names, "count": len(names)}


@mcp.tool()
def find_items(pattern: str, item_type: Optional[int] = None) -> dict:
    """Glob-match items by name.
    item_type uses RoboDK codes: 1=Robot, 2=Frame, 3=Tool, 4=Object,
    5=Target, 6=Program, 19=Camera.
    """
    rdk = get_rdk()
    raw = rdk.ItemList(item_type if item_type is not None else -1, 1)
    matches = [str(n) for n in raw if fnmatch.fnmatchcase(str(n), pattern)]
    return {"pattern": pattern, "item_type": item_type,
            "matches": matches, "count": len(matches)}


# ===========================================================================
# 3. POSES / KINEMATICS
# ===========================================================================

@mcp.tool()
def get_object_pose(object_name: str) -> dict:
    """Absolute world pose of any item."""
    pose = _item(object_name).PoseAbs()
    return {"object": object_name,
            "pose_matrix_row_major": _pose_flat(pose),
            "position_mm": _xyz(pose)}


@mcp.tool()
def get_pose(item_name: str, kind: str = "abs") -> dict:
    """Get pose of an item. kind: 'abs', 'local', 'tool', 'frame'."""
    it = _item(item_name)
    pose = {"abs": it.PoseAbs, "local": it.Pose,
            "tool": it.PoseTool, "frame": it.PoseFrame}[kind]()
    return {"item": item_name, "kind": kind,
            "pose_matrix": _pose_rows(pose),
            "position_mm": _xyz(pose)}


@mcp.tool()
def get_robot_joints(robot_name: str) -> dict:
    """Current robot joints in degrees."""
    r = _robot(robot_name)
    return {"robot": r.Name(),
            "joints_deg": list(r.Joints().tolist()[0])}


@mcp.tool()
def get_tcp_pose(robot_name: str) -> dict:
    """Current TCP pose in world coords (follows attached objects)."""
    r = _robot(robot_name)
    pose = r.SolveFK(r.Joints()) * r.PoseTool()
    return {"robot": r.Name(),
            "pose_matrix_row_major": _pose_flat(pose),
            "position_mm": _xyz(pose)}


@mcp.tool()
def solve_fk(robot_name: str, joints: list) -> dict:
    """Forward kinematics for the robot's active TCP."""
    r = _robot(robot_name)
    pose = r.SolveFK(joints) * r.PoseTool()
    return {"robot": r.Name(), "joints_deg": joints,
            "pose_matrix_row_major": _pose_flat(pose),
            "position_mm": _xyz(pose)}


@mcp.tool()
def solve_ik(robot_name: str, pose: Any,
             joints_seed: Optional[list] = None) -> dict:
    """Inverse kinematics, returns nearest single solution.
    pose: 4x4 nested list or 16-element flat list.
    joints_seed: if given, seeds IK from this configuration (locks branch).
    """
    r = _robot(robot_name)
    p = _mat(pose)
    if joints_seed is not None:
        r.setJoints(joints_seed)
    sol = r.SolveIK(p)
    joints_out = list(sol.tolist()[0]) if sol.size() > 0 else []
    return {"robot": r.Name(), "joints_deg": joints_out}


@mcp.tool()
def solve_ik_all(robot_name: str, pose: Any) -> dict:
    """All IK branches (FIX: returns proper N x 6 array)."""
    r = _robot(robot_name)
    p = _mat(pose)
    mat = r.SolveIK_All(p)
    rows = mat.Rows() if hasattr(mat, "Rows") else mat
    sols = []
    for row in rows:
        rowlist = list(row)
        if any(abs(v) > 1e-9 for v in rowlist[:6]):
            sols.append([round(v, 6) for v in rowlist[:6]])
    return {"robot": r.Name(),
            "num_solutions": len(sols),
            "solutions_deg": sols}


# ===========================================================================
# 4. ROBOT MOTION
# ===========================================================================

@mcp.tool()
def set_robot_joints(robot_name: str, joints: list) -> dict:
    """Instantly snap robot to joints (no animation)."""
    r = _robot(robot_name)
    r.setJoints(joints)
    return {"robot": r.Name(), "joints_set": joints}


@mcp.tool()
def move_joint(robot_name: str, joints: list, blocking: bool = True) -> dict:
    """MoveJ to joint angles (degrees)."""
    r = _robot(robot_name)
    r.MoveJ(joints, blocking)
    return {"robot": r.Name(), "moved_to_joints": joints}


@mcp.tool()
def move_linear(robot_name: str, pose: Any, blocking: bool = True) -> dict:
    """MoveL to a Cartesian pose (FIX: pre-seeds and falls back via MoveJ).
    pose: 4x4 nested list or 16-element flat list (mm).
    """
    r = _robot(robot_name)
    p = _mat(pose)
    current = r.Joints()
    try:
        r.MoveL(p, blocking)
    except Exception as e:
        target_joints = r.SolveIK(p, current)
        if target_joints.size() == 0:
            return {"error": f"no IK solution for pose: {e}"}
        r.MoveJ(target_joints, blocking)
        r.MoveL(p, blocking)
    return {"robot": r.Name(), "moved_to": _xyz(p)}


@mcp.tool()
def move_to_target(robot_name: str, target_name: str,
                   move_type: str = "joint") -> dict:
    """MoveJ or MoveL to a named Target (FIX: explicit ITEM_TYPE_TARGET lookup)."""
    r = _robot(robot_name)
    tgt = _target(target_name)
    if move_type == "linear":
        r.MoveL(tgt)
    elif move_type == "joint":
        r.MoveJ(tgt)
    else:
        return {"error": f"unknown move_type '{move_type}' "
                         f"(use 'joint' or 'linear')"}
    return {"robot": r.Name(), "target": target_name,
            "move_type": move_type}


@mcp.tool()
def set_robot_speed(robot_name: str, speed_mm_s: float,
                    accel_mm_s2: Optional[float] = None) -> dict:
    """Set TCP linear speed and optional acceleration."""
    r = _robot(robot_name)
    if accel_mm_s2 is not None:
        r.setSpeed(speed_mm_s, accel_mm_s2)
    else:
        r.setSpeed(speed_mm_s)
    return {"robot": r.Name(), "speed_mm_s": speed_mm_s,
            "accel_mm_s2": accel_mm_s2}


@mcp.tool()
def set_simulation_speed(speed: float) -> dict:
    """Simulation playback speed. 1=real-time, 5=default, 0.5=half."""
    get_rdk().setSimulationSpeed(speed)
    return {"simulation_speed": speed}


@mcp.tool()
def render(refresh: bool = True) -> dict:
    """Force display refresh. False pauses rendering during batch ops."""
    get_rdk().Render(refresh)
    return {"rendered": refresh}


@mcp.tool()
def move_joint_test(robot_name: str, joints_start: list, joints_end: list,
                    min_step_deg: float = 2.0) -> dict:
    """Test a joint-space straight-line move for collisions.
    Returns the path-fraction index where collision starts, or < 0 if free.
    """
    r = _robot(robot_name)
    result = r.MoveJ_Test(joints_start, joints_end, min_step_deg)
    return {"robot": r.Name(), "collision_at_step": result,
            "collision_free": result < 0}


@mcp.tool()
def move_linear_test(robot_name: str, pose_start: Any, pose_end: Any,
                     min_step_mm: float = 1.0) -> dict:
    """Test a linear Cartesian move for collisions."""
    r = _robot(robot_name)
    result = r.MoveL_Test(_mat(pose_start), _mat(pose_end), min_step_mm)
    return {"robot": r.Name(), "collision_at_step": result,
            "collision_free": result < 0}


# ===========================================================================
# 5. PROGRAMS, TARGETS, FRAMES, TOOLS  (Tier 1 + 2 additions)
# ===========================================================================

@mcp.tool()
def add_target(name: str, robot_name: str,
               parent_frame_name: Optional[str] = None,
               joints_deg: Optional[list] = None,
               pose: Optional[Any] = None,
               as_joint_target: bool = True) -> dict:
    """Create a Target item in the station tree.
    Supply joints (preferred -- locks the IK branch) and/or a 4x4 pose.
    """
    rdk = get_rdk()
    r = _robot(robot_name)
    frame = _frame(parent_frame_name) if parent_frame_name else r.Parent()
    tgt = rdk.AddTarget(name, frame, r)
    if pose is not None:
        tgt.setPose(_mat(pose))
    if joints_deg is not None:
        tgt.setJoints(joints_deg)
    if as_joint_target:
        tgt.setAsJointTarget()
    else:
        tgt.setAsCartesianTarget()
    return {"created_target": tgt.Name(), "parent_frame": frame.Name(),
            "robot": r.Name(), "as_joint_target": as_joint_target}


@mcp.tool()
def add_program(name: str, robot_name: str,
                speed_mm_s: float = 100.0,
                joint_speed_deg_s: float = 30.0) -> dict:
    """Create an empty Program in the station tree."""
    rdk = get_rdk()
    r = _robot(robot_name)
    prog = rdk.AddProgram(name, r)
    try:
        prog.setSpeed(speed_mm_s)
    except Exception:
        pass
    try:
        prog.setSpeedJoints(joint_speed_deg_s)
    except Exception:
        pass
    return {"created_program": prog.Name(), "robot": r.Name(),
            "speed_mm_s": speed_mm_s,
            "joint_speed_deg_s": joint_speed_deg_s}


@mcp.tool()
def program_add_move(program_name: str, target_name: str,
                     move_type: str = "J") -> dict:
    """Append a MoveJ ('J'), MoveL ('L'), or MoveC ('C:other_target') instruction."""
    prog = _program(program_name)
    tgt = _target(target_name)
    if move_type == "J":
        prog.MoveJ(tgt)
    elif move_type == "L":
        prog.MoveL(tgt)
    elif move_type.startswith("C:"):
        prog.MoveC(tgt, _target(move_type[2:]))
    else:
        return {"error": f"unknown move_type '{move_type}' "
                         f"(use 'J', 'L', or 'C:<target>')"}
    return {"program": program_name, "appended_move": move_type,
            "target": target_name}


@mcp.tool()
def program_add_call(program_name: str, sub_program_name: str) -> dict:
    """Append a 'call program' instruction."""
    prog = _program(program_name)
    try:
        prog.RunCodeCustom(sub_program_name, 1)  # 1 = Call Program
    except Exception:
        prog.RunInstruction(sub_program_name, 1)
    return {"program": program_name, "called": sub_program_name}


@mcp.tool()
def program_add_wait(program_name: str, seconds: float) -> dict:
    """Append a 'pause' instruction (seconds)."""
    prog = _program(program_name)
    prog.Pause(seconds * 1000.0)  # RoboDK API expects ms
    return {"program": program_name, "wait_seconds": seconds}


@mcp.tool()
def program_clear(program_name: str) -> dict:
    """Remove all instructions from a program."""
    prog = _program(program_name)
    n = prog.InstructionCount()
    for _ in range(n):
        prog.InstructionDelete(0)
    return {"program": program_name, "cleared_instructions": n}


@mcp.tool()
def get_program_instructions(program_name: str) -> dict:
    """List the instructions inside a program."""
    prog = _program(program_name)
    out = []
    for i in range(prog.InstructionCount()):
        try:
            ins = prog.Instruction(i)
            if isinstance(ins, (list, tuple)):
                out.append({"index": i, "raw": list(ins)})
            else:
                out.append({"index": i, "raw": str(ins)})
        except Exception as e:
            out.append({"index": i, "error": str(e)})
    return {"program": program_name, "count": len(out),
            "instructions": out}


@mcp.tool()
def run_program(program_name: str) -> dict:
    """Run a program (blocks until finished)."""
    prog = _program(program_name)
    prog.RunProgram()
    while prog.Busy():
        time.sleep(0.05)
    return {"program": program_name, "status": "completed"}


@mcp.tool()
def program_make_robot_program(program_name: str,
                               output_dir: Optional[str] = None,
                               robot_post: Optional[str] = None) -> dict:
    """Post-process a program into native robot code (KRL, RAPID, URP, ...)."""
    prog = _program(program_name)
    if robot_post:
        try:
            prog.setPostProcessor(robot_post)
        except Exception:
            pass
    output_dir = output_dir or tempfile.gettempdir()
    ok, log = prog.MakeProgram(output_dir)
    return {"program": program_name, "success": bool(ok),
            "log": log, "output_dir": output_dir}


@mcp.tool()
def add_frame(name: str, parent_frame_name: Optional[str] = None,
              pose: Optional[Any] = None) -> dict:
    """Create a reference Frame."""
    rdk = get_rdk()
    parent = _frame(parent_frame_name) if parent_frame_name else rdk.ActiveStation()
    fr = rdk.AddFrame(name, parent)
    if pose is not None:
        fr.setPose(_mat(pose))
    return {"created_frame": fr.Name(),
            "parent": parent.Name() if hasattr(parent, "Name") else "Station"}


@mcp.tool()
def set_active_tool(robot_name: str, tool_name: str) -> dict:
    """Make `tool_name` the active TCP on `robot_name`.
    Tool must already exist as a child of the robot."""
    r = _robot(robot_name)
    t = _tool_item(tool_name)
    r.setPoseTool(t)
    return {"robot": r.Name(), "active_tool": t.Name(),
            "tcp_pose": _pose_rows(r.PoseTool())}


@mcp.tool()
def add_tool_from_object(object_name: str, robot_name: str,
                         tool_name: Optional[str] = None,
                         tcp_pose: Optional[Any] = None) -> dict:
    """Convert an Object into a Tool attached to the robot flange so it
    participates in FK/IK as the active TCP.
    tcp_pose: 4x4 of TCP relative to flange. If None, current
    object-pose-relative-to-flange is used.
    """
    obj = _item(object_name)
    r = _robot(robot_name)
    if tcp_pose is not None:
        pose = _mat(tcp_pose)
    else:
        pose = r.PoseAbs().inv() * obj.PoseAbs()
    new_tool = r.AddTool(pose, tool_name or (obj.Name() + " Tool"))
    obj.setParentStatic(new_tool)
    return {"created_tool": new_tool.Name(), "robot": r.Name(),
            "tcp_pose_in_flange": _pose_rows(pose)}


# ===========================================================================
# 6. SCENE OBJECTS
# ===========================================================================

@mcp.tool()
def add_object_from_file(file_path: str, object_name: Optional[str] = None,
                         x: float = 0, y: float = 0, z: float = 0) -> dict:
    """Import a 3D file (.stl/.step/.obj/.wrl) into the station."""
    from robodk.robomath import transl
    rdk = get_rdk()
    item = rdk.AddFile(file_path)
    if not item.Valid():
        return {"error": f"failed to import {file_path}"}
    if object_name:
        item.setName(object_name)
    if any((x, y, z)):
        item.setPose(transl(x, y, z))
    return {"imported": item.Name(), "file_path": file_path,
            "position_mm": {"x": x, "y": y, "z": z}}


@mcp.tool()
def set_object_pose(object_name: str, x: float, y: float, z: float,
                    rx_deg: float = 0, ry_deg: float = 0,
                    rz_deg: float = 0) -> dict:
    """Move an item to an absolute position + Euler XYZ rotation (degrees)."""
    from robodk.robomath import transl, rotx, roty, rotz
    item = _item(object_name)
    pose = (transl(x, y, z) *
            rotx(math.radians(rx_deg)) *
            roty(math.radians(ry_deg)) *
            rotz(math.radians(rz_deg)))
    item.setPose(pose)
    return {"object": object_name,
            "moved_to": {"x": x, "y": y, "z": z},
            "rotation_deg": {"rx": rx_deg, "ry": ry_deg, "rz": rz_deg}}


@mcp.tool()
def set_item_pose(item_name: str, pose: Any, kind: str = "local") -> dict:
    """Generic pose setter accepting a full 4x4 matrix.
    kind: 'abs' (world), 'local' (parent frame), 'tool', 'frame'."""
    it = _item(item_name)
    p = _mat(pose)
    if kind == "abs":
        it.setPoseAbs(p)
    elif kind == "local":
        it.setPose(p)
    elif kind == "tool":
        it.setPoseTool(p)
    elif kind == "frame":
        it.setPoseFrame(p)
    else:
        return {"error": f"unknown kind '{kind}'"}
    return {"item": item_name, "kind": kind, "set": True}


@mcp.tool()
def set_object_color(object_name: str, r: float, g: float, b: float,
                     a: float = 1.0) -> dict:
    """Set RGBA color (0..1)."""
    _item(object_name).Recolor([r, g, b, a])
    return {"object": object_name,
            "color": {"r": r, "g": g, "b": b, "a": a}}


@mcp.tool()
def set_object_visible(object_name: str, visible: bool = True) -> dict:
    """Show or hide an item."""
    _item(object_name).setVisible(visible)
    return {"object": object_name, "visible": visible}


@mcp.tool()
def set_item_name(item_name: str, new_name: str) -> dict:
    """Rename an item."""
    it = _item(item_name)
    it.setName(new_name)
    return {"renamed_from": item_name, "renamed_to": new_name}


@mcp.tool()
def set_parent(item_name: str, parent_name: str,
               keep_pose: bool = True) -> dict:
    """Reparent an item. keep_pose=True preserves world pose."""
    it = _item(item_name)
    pa = _item(parent_name)
    if keep_pose:
        it.setParentStatic(pa)
    else:
        it.setParent(pa)
    return {"item": item_name, "new_parent": parent_name,
            "keep_pose": keep_pose}


@mcp.tool()
def delete_item(item_name: str) -> dict:
    """Delete an item. Irreversible."""
    _item(item_name).Delete()
    return {"deleted": item_name}


@mcp.tool()
def bulk_delete(pattern: str, item_type: Optional[int] = None) -> dict:
    """Delete every item whose name matches a glob pattern."""
    matches = find_items(pattern, item_type)["matches"]
    deleted = []
    for n in matches:
        try:
            _item(n).Delete()
            deleted.append(n)
        except Exception:
            pass
    return {"pattern": pattern, "deleted": deleted, "count": len(deleted)}


@mcp.tool()
def attach_object_to_robot(object_name: str, robot_name: str) -> dict:
    """Make an object move with the robot's flange (simulate gripper close)."""
    obj = _item(object_name)
    r = _robot(robot_name)
    obj.setParentStatic(r)
    return {"object": object_name, "attached_to": r.Name()}


@mcp.tool()
def detach_object(object_name: str,
                  new_parent_name: Optional[str] = None) -> dict:
    """Release an object from its parent (simulate gripper open)."""
    rdk = get_rdk()
    obj = _item(object_name)
    parent = _item(new_parent_name) if new_parent_name else rdk.ActiveStation()
    obj.setParentStatic(parent)
    return {"object": object_name,
            "new_parent": parent.Name() if hasattr(parent, "Name") else "Station"}


@mcp.tool()
def load_station(file_path: str) -> dict:
    """Open a .rdk station file."""
    item = get_rdk().AddFile(file_path)
    return {"loaded": file_path, "valid": item.Valid()}


@mcp.tool()
def save_station(file_path: Optional[str] = None) -> dict:
    """Save the active station. If file_path given, save-as; else in-place."""
    rdk = get_rdk()
    station = rdk.ActiveStation()
    target = file_path or rdk.getParam("PATH_OPENSTATION")
    rdk.Save(target, station)
    return {"saved_to": target}


# ===========================================================================
# 7. CAMERAS & VISION
# ===========================================================================

@mcp.tool()
def add_camera(frame_name: str, camera_name: Optional[str] = None,
               fov_deg: float = 30, focal_length_mm: float = 6,
               far_length_mm: float = 2000,
               width_px: int = 640, height_px: int = 480) -> dict:
    """Create a simulated 2D camera attached to a reference frame.
    FIX: JSON-serializable response. FIX: honors camera_name.
    Camera convention: frame +Z = look direction, +Y = image down."""
    frame = _frame(frame_name)
    params = (f"FOV={fov_deg} FOCAL_LENGTH={focal_length_mm} "
              f"FAR_LENGTH={far_length_mm} "
              f"SIZE={width_px}x{height_px} BG_COLOR=black")
    cam = get_rdk().Cam2D_Add(frame, params)
    actual_name = None
    if cam is not None:
        try:
            if camera_name:
                cam.setName(camera_name)
            actual_name = cam.Name()
        except Exception:
            actual_name = camera_name
    return {"camera_name": actual_name, "frame": frame.Name(),
            "fov_deg": fov_deg, "focal_length_mm": focal_length_mm,
            "far_length_mm": far_length_mm,
            "resolution_px": [width_px, height_px]}


@mcp.tool()
def set_camera_pose(frame_name: str, x: float, y: float, z: float,
                    rx_deg: float = 0, ry_deg: float = 0,
                    rz_deg: float = 0) -> dict:
    """Reposition a camera by moving its anchor reference frame."""
    return set_object_pose(frame_name, x, y, z, rx_deg, ry_deg, rz_deg)


@mcp.tool()
def get_camera_image_path(camera_frame_name: str,
                          save_path: Optional[str] = None) -> dict:
    """Snapshot to disk and return only the file path."""
    cam = _item(camera_frame_name)
    if save_path is None:
        save_path = os.path.join(tempfile.gettempdir(),
                                 next(tempfile._get_candidate_names()) + ".png")
    get_rdk().Cam2D_Snapshot(save_path, cam)
    return {"success": True, "file_path": save_path}


@mcp.tool()
def capture_snapshot(camera_frame_name: str,
                     save_path: Optional[str] = None,
                     include_base64: bool = False) -> dict:
    """Snapshot from a simulated camera.
    FIX: base64 payload is opt-in (defaulted off to keep responses small)."""
    out = get_camera_image_path(camera_frame_name, save_path)
    if include_base64:
        with open(out["file_path"], "rb") as f:
            out["png_base64"] = base64.b64encode(f.read()).decode()
    return out


@mcp.tool()
def detect_blobs(camera_frame_name: str, min_area_px: int = 500,
                 max_area_px: int = 50000, max_blobs: int = 200) -> dict:
    """OpenCV blob detection on a fresh snapshot.
    FIX: early-exit on near-uniform images (v2 timed out)."""
    import cv2
    out = get_camera_image_path(camera_frame_name)
    img = cv2.imread(out["file_path"], cv2.IMREAD_GRAYSCALE)
    if img is None:
        return {"error": "failed to read snapshot"}
    if float(img.std()) < 5.0:
        return {"camera": camera_frame_name, "blobs": [], "count": 0,
                "note": "near-uniform scene; skipped detection"}
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = min_area_px
    params.maxArea = max_area_px
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(img)[:max_blobs]
    blobs = [{"center_px": {"x": float(k.pt[0]), "y": float(k.pt[1])},
              "size_px": float(k.size)} for k in keypoints]
    return {"camera": camera_frame_name, "blobs": blobs, "count": len(blobs)}


_HSV_RANGES = {
    "red":    ([0, 100, 100],  [10, 255, 255]),
    "orange": ([10, 100, 100], [25, 255, 255]),
    "yellow": ([20, 100, 100], [35, 255, 255]),
    "green":  ([35, 50, 50],   [85, 255, 255]),
    "blue":   ([90, 50, 50],   [130, 255, 255]),
    "white":  ([0, 0, 200],    [180, 30, 255]),
    "black":  ([0, 0, 0],      [180, 255, 30]),
}


@mcp.tool()
def detect_objects_by_color(camera_frame_name: str, color: str = "red",
                            min_area_px: int = 300,
                            max_objects: int = 200) -> dict:
    """HSV color detection. Same early-exit fix as detect_blobs."""
    import cv2
    out = get_camera_image_path(camera_frame_name)
    img = cv2.imread(out["file_path"])
    if img is None:
        return {"error": "failed to read snapshot"}
    if float(img.std()) < 5.0:
        return {"camera": camera_frame_name, "objects": [], "count": 0,
                "note": "near-uniform scene; skipped detection"}
    if color not in _HSV_RANGES:
        return {"error": f"unknown color '{color}', "
                         f"valid: {list(_HSV_RANGES)}"}
    lo, hi = _HSV_RANGES[color]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    objects = []
    for c in contours[:max_objects]:
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        bx, by, bw, bh = cv2.boundingRect(c)
        objects.append({"centroid_px": {"x": cx, "y": cy},
                        "area_px": float(area),
                        "bbox_px": {"x": bx, "y": by, "w": bw, "h": bh}})
    return {"camera": camera_frame_name, "color": color,
            "objects": objects, "count": len(objects)}


@mcp.tool()
def pixel_to_world(camera_frame_name: str, pixel_x: float, pixel_y: float,
                   table_z_mm: float = 0, fov_deg: float = 30,
                   image_width_px: int = 640,
                   image_height_px: int = 480) -> dict:
    """Project a pixel onto a flat table at table_z_mm using pinhole geometry.
    FIX: actually uses the camera frame's world pose."""
    cam = _item(camera_frame_name)
    pose = cam.PoseAbs()
    rows = pose.Rows()
    R = np.array([list(rows[i])[:3] for i in range(3)])
    origin = np.array(pose.Pos())
    fov_rad = np.radians(fov_deg)
    fx = (image_width_px / 2) / np.tan(fov_rad / 2)
    ray_cam = np.array([(pixel_x - image_width_px / 2) / fx,
                        (pixel_y - image_height_px / 2) / fx,
                        1.0])
    direction = R @ ray_cam
    if abs(direction[2]) < 1e-9:
        return {"error": "camera ray is parallel to the table plane"}
    t = (table_z_mm - origin[2]) / direction[2]
    p = origin + t * direction
    return {"pixel": {"x": pixel_x, "y": pixel_y},
            "world_mm": {"x": round(float(p[0]), 3),
                         "y": round(float(p[1]), 3),
                         "z": round(float(p[2]), 3)},
            "note": "Pinhole approximation; assumes a flat table at table_z_mm."}


# ===========================================================================
# 8. COLLISIONS
# ===========================================================================

@mcp.tool()
def check_collision(item1_name: str, item2_name: str) -> dict:
    """Test whether two specific items are currently colliding."""
    rdk = get_rdk()
    a = _item(item1_name)
    b = _item(item2_name)
    return {"item1": item1_name, "item2": item2_name,
            "collision": bool(rdk.Collision(a, b))}


@mcp.tool()
def check_ray_collision(x1: float, y1: float, z1: float,
                        x2: float, y2: float, z2: float) -> dict:
    """Cast a ray and return the first item hit (if any)."""
    rdk = get_rdk()
    hit, item, xyz = rdk.Collision_Line([x1, y1, z1], [x2, y2, z2])
    return {"hit": bool(hit),
            "item": item.Name() if hit and item.Valid() else None,
            "intersection_mm": ({"x": xyz[0], "y": xyz[1], "z": xyz[2]}
                                if hit else None)}


@mcp.tool()
def set_collision_detection(enabled: bool = True) -> dict:
    """Enable/disable collision detection."""
    from robodk.robolink import COLLISION_ON, COLLISION_OFF
    get_rdk().setCollisionActive(COLLISION_ON if enabled else COLLISION_OFF)
    return {"collision_detection": enabled}


@mcp.tool()
def get_all_collisions() -> dict:
    """List all colliding item pairs (FIX: pairs now match the count)."""
    rdk = get_rdk()
    pairs = []
    try:
        for pair in rdk.CollisionPairs():
            a, b = pair
            pairs.append({"a": a.Name() if a.Valid() else "(unknown)",
                          "b": b.Name() if b.Valid() else "(unknown)"})
    except Exception:
        pass
    return {"collision_count": len(pairs), "colliding_pairs": pairs}


# ===========================================================================
# 9. COMPOSITE PIPELINES (PICK & PLACE)
# ===========================================================================

@mcp.tool()
def plan_grasp_pose(object_name: str,
                    approach_distance_mm: float = 100,
                    grasp_height_offset_mm: float = 0) -> dict:
    """Compute top-down approach and grasp poses for an object."""
    pose = _item(object_name).PoseAbs()
    p = pose.Pos()
    grasp = [[0, -1, 0, p[0]],
             [-1, 0, 0, p[1]],
             [0, 0, -1, p[2] + grasp_height_offset_mm],
             [0, 0, 0, 1]]
    approach = [row[:] for row in grasp]
    approach[2][3] += approach_distance_mm
    return {"object": object_name,
            "approach_pose": approach, "grasp_pose": grasp}


@mcp.tool()
def execute_pick(robot_name: str, object_name: str,
                 approach_distance_mm: float = 100,
                 speed_mm_s: float = 100) -> dict:
    """approach -> linear descend -> attach -> linear retract."""
    r = _robot(robot_name)
    obj = _item(object_name)
    r.setSpeed(speed_mm_s)
    p = obj.PoseAbs().Pos()
    grasp = [[0, -1, 0, p[0]],
             [-1, 0, 0, p[1]],
             [0, 0, -1, p[2]],
             [0, 0, 0, 1]]
    approach = [row[:] for row in grasp]
    approach[2][3] += approach_distance_mm
    move_linear(robot_name, approach)
    move_linear(robot_name, grasp)
    obj.setParentStatic(r)
    move_linear(robot_name, approach)
    return {"robot": r.Name(), "picked": object_name}


@mcp.tool()
def execute_place(robot_name: str, object_name: str,
                  place_x: float, place_y: float, place_z: float,
                  approach_distance_mm: float = 100,
                  speed_mm_s: float = 100) -> dict:
    """approach -> linear descend -> detach -> linear retract."""
    rdk = get_rdk()
    r = _robot(robot_name)
    obj = _item(object_name)
    r.setSpeed(speed_mm_s)
    place = [[0, -1, 0, place_x],
             [-1, 0, 0, place_y],
             [0, 0, -1, place_z],
             [0, 0, 0, 1]]
    approach = [row[:] for row in place]
    approach[2][3] += approach_distance_mm
    move_linear(robot_name, approach)
    move_linear(robot_name, place)
    obj.setParentStatic(rdk.ActiveStation())
    move_linear(robot_name, approach)
    return {"robot": r.Name(), "placed": object_name,
            "at": {"x": place_x, "y": place_y, "z": place_z}}


@mcp.tool()
def vision_pick_by_color(robot_name: str, camera_frame_name: str,
                         object_name: str, color: str = "red",
                         table_z_mm: float = 0, fov_deg: float = 30,
                         image_width_px: int = 640,
                         image_height_px: int = 480,
                         place_x: float = 0, place_y: float = 500,
                         place_z: float = 0) -> dict:
    """End-to-end: snapshot -> detect color -> pixel->world -> move object
    -> pick -> place."""
    det = detect_objects_by_color(camera_frame_name, color=color)
    if not det.get("objects"):
        return {"error": "no object detected", "color": color}
    o = det["objects"][0]
    world = pixel_to_world(camera_frame_name,
                           o["centroid_px"]["x"], o["centroid_px"]["y"],
                           table_z_mm=table_z_mm, fov_deg=fov_deg,
                           image_width_px=image_width_px,
                           image_height_px=image_height_px)
    set_object_pose(object_name, world["world_mm"]["x"],
                                world["world_mm"]["y"], table_z_mm)
    execute_pick(robot_name, object_name)
    execute_place(robot_name, object_name, place_x, place_y, place_z)
    return {"detected_at": world["world_mm"],
            "picked_and_placed": object_name}


# ===========================================================================
# 10. STATION I/O & METADATA
# ===========================================================================

@mcp.tool()
def get_param(name: str) -> dict:
    """Read a RoboDK station parameter."""
    return {"name": name, "value": get_rdk().getParam(name)}


@mcp.tool()
def set_param(name: str, value: str) -> dict:
    """Set a RoboDK station parameter."""
    get_rdk().setParam(name, value)
    return {"name": name, "value": value}


_RUN_MODES = {
    "simulate":       1,
    "quick_validate": 2,
    "make_program":   3,
    "run_robot":      4,
}


@mcp.tool()
def set_run_mode(mode: str) -> dict:
    """Set the simulation run mode.
    'simulate' / 'quick_validate' / 'make_program' / 'run_robot'."""
    code = _RUN_MODES.get(mode)
    if code is None:
        return {"error": f"unknown mode '{mode}', valid: {list(_RUN_MODES)}"}
    get_rdk().setRunMode(code)
    return {"mode": mode, "code": code}


@mcp.tool()
def show_message(text: str, popup: bool = False) -> dict:
    """Display a message in the RoboDK status bar (or popup)."""
    get_rdk().ShowMessage(text, popup)
    return {"shown": text, "popup": popup}


@mcp.tool()
def get_joint_limits(robot_name: str) -> dict:
    """Per-joint lower/upper limits in degrees."""
    r = _robot(robot_name)
    lo, hi = r.JointLimits()
    return {"robot": r.Name(),
            "lower_deg": list(lo), "upper_deg": list(hi)}


@mcp.tool()
def set_joint_limits(robot_name: str, lower_deg: list,
                     upper_deg: list) -> dict:
    """Set per-joint limits in degrees."""
    r = _robot(robot_name)
    r.setJointLimits(lower_deg, upper_deg)
    return {"robot": r.Name(),
            "lower_deg": lower_deg, "upper_deg": upper_deg}


# ===========================================================================
# CLI ENTRYPOINT
# ===========================================================================

def main():
    global _ROBODK_HOST, _ROBODK_PORT
    p = argparse.ArgumentParser(description="RoboDK MCP Server v3")
    p.add_argument("--host", default="localhost",
                   help="RoboDK API host (default: localhost)")
    p.add_argument("--port", default=20500, type=int,
                   help="RoboDK API port (default: 20500)")
    args = p.parse_args()
    _ROBODK_HOST = args.host
    _ROBODK_PORT = args.port
    mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
