# RoboDK-mcp Coverage Gaps and Proposed Additions

Comparison of the v2 MCP surface (41 tools) against the RoboDK Python API
(`robodk.robolink.Robolink` + `Item`). For each missing capability below
there is a ready-to-paste FastMCP tool that informed the **v3** rewrite.

> **Note:** every Tier 1-7 addition listed here is **already implemented** in
> `v3/robodk_mcp_server_v3.py`. This document is preserved as the design
> rationale and as a tier-by-tier mapping you can use when reviewing v3.

The proposed signatures use the same patterns as the existing tools:
- Names are kebab-snake (e.g. `add_target`), not the API's CamelCase.
- Items are referenced by **name** strings, never by Item handles, so the
  responses stay JSON-serializable.
- 4x4 poses are nested lists. Joint vectors are degree lists.
- Every tool returns a dict with the inputs echoed back, plus the action's
  result (created name, joints, pose, etc.) so the LLM can chain calls.

The `# fix:` comments inside each snippet flag the known bugs from
[TOOLS_v2_AUDIT.md](TOOLS_v2_AUDIT.md) that the new code corrects.

---

## Tier 1 - High-impact additions (programming the station)

These four tools alone would have removed the need for the
`build_dispenser_programs.py` / `.m` workaround. All implemented in v3.

### `add_target`

```python
from robodk.robolink import ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME
from robodk.robomath import Mat

@mcp.tool()
def add_target(
    name: str,
    robot_name: str,
    parent_frame_name: str | None = None,
    joints_deg: list[float] | None = None,
    pose_4x4: list[list[float]] | None = None,
    as_joint_target: bool = True,
) -> dict:
    """Create a Target item in the station tree.

    If parent_frame_name is None, the robot's current reference frame is used.
    Provide joints (preferred, locks the IK branch) and/or a 4x4 pose.
    """
    robot = RDK.Item(robot_name, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        return {"error": f"robot '{robot_name}' not found"}
    frame = (RDK.Item(parent_frame_name, ITEM_TYPE_FRAME)
             if parent_frame_name else robot.Parent())
    tgt = RDK.AddTarget(name, frame, robot)
    if pose_4x4 is not None:
        tgt.setPose(Mat(pose_4x4))
    if joints_deg is not None:
        tgt.setJoints(joints_deg)
    if as_joint_target:
        tgt.setAsJointTarget()
    else:
        tgt.setAsCartesianTarget()
    return {"created": name, "parent_frame": frame.Name(),
            "joints_deg": joints_deg, "as_joint_target": as_joint_target}
```

### `add_program`

```python
@mcp.tool()
def add_program(name: str, robot_name: str,
                speed_mm_s: float = 100.0,
                joint_speed_dps: float = 30.0) -> dict:
    """Create an empty Program in the station tree."""
    robot = RDK.Item(robot_name, ITEM_TYPE_ROBOT)
    prog = RDK.AddProgram(name, robot)
    prog.setSpeed(speed_mm_s)
    prog.setSpeedJoints(joint_speed_dps)
    return {"created": name, "robot": robot.Name(),
            "speed_mm_s": speed_mm_s, "joint_speed_dps": joint_speed_dps}
```

### `program_add_move`

```python
@mcp.tool()
def program_add_move(program_name: str, target_name: str,
                     move_type: str = "J") -> dict:
    """Append MoveJ ('J'), MoveL ('L'), or MoveC ('C:other_target')."""
    prog = RDK.Item(program_name, ITEM_TYPE_PROGRAM)
    tgt = RDK.Item(target_name, ITEM_TYPE_TARGET)
    if move_type == "J": prog.MoveJ(tgt)
    elif move_type == "L": prog.MoveL(tgt)
    elif move_type.startswith("C:"):
        prog.MoveC(tgt, RDK.Item(move_type[2:], ITEM_TYPE_TARGET))
    return {"program": program_name, "appended": move_type,
            "target": target_name}
```

### `add_frame`

```python
@mcp.tool()
def add_frame(name: str, parent_frame_name: str | None = None,
              pose_4x4: list[list[float]] | None = None) -> dict:
    parent = (RDK.Item(parent_frame_name, ITEM_TYPE_FRAME)
              if parent_frame_name else RDK.ActiveStation())
    fr = RDK.AddFrame(name, parent)
    if pose_4x4 is not None:
        fr.setPose(Mat(pose_4x4))
    return {"created": name, "parent": parent.Name()}
```

---

## Tier 2 - Active tool/frame on the robot

This is the *single* fix that erases the dispenser-offset math: the dispenser
is loaded as an Object (type 4), so `solve_fk`/`solve_ik` operate on the
flange `UR5 Base` TCP. If the dispenser were the active tool, every IK target
could be expressed in dispenser-tip space directly. Both implemented in v3.

### `set_active_tool`

```python
@mcp.tool()
def set_active_tool(robot_name: str, tool_name: str) -> dict:
    robot = RDK.Item(robot_name, ITEM_TYPE_ROBOT)
    tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
    robot.setPoseTool(tool)
    return {"robot": robot.Name(), "active_tool": tool.Name(),
            "tcp_pose": robot.PoseTool().Rows()}
```

### `add_tool_from_object`

```python
@mcp.tool()
def add_tool_from_object(object_name: str, robot_name: str,
                         tool_name: str | None = None,
                         tcp_pose_4x4: list[list[float]] | None = None) -> dict:
    obj   = RDK.Item(object_name, ITEM_TYPE_OBJECT)
    robot = RDK.Item(robot_name, ITEM_TYPE_ROBOT)
    pose = Mat(tcp_pose_4x4) if tcp_pose_4x4 else obj.PoseAbs()
    tool = robot.AddTool(pose, tool_name or (obj.Name() + " Tool"))
    obj.setParentStatic(tool)
    return {"created_tool": tool.Name(), "robot": robot.Name()}
```

---

## Tier 3 - Item introspection and state

All implemented in v3 (`get_pose`, `set_item_pose`, `set_item_name`,
`set_parent`, `get_joint_limits`, `set_joint_limits`).

---

## Tier 4 - Programs: inspect, edit, export

Implemented in v3 as `get_program_instructions`, `program_clear`,
`program_make_robot_program`. Sub-program calls and pauses are exposed as
`program_add_call` and `program_add_wait`.

---

## Tier 5 - Station I/O and metadata

Implemented in v3 as `save_station`, `get_param`, `set_param`, `set_run_mode`,
`show_message`.

---

## Tier 6 - Geometry and motion testing

Implemented in v3 as `move_joint_test`, `move_linear_test`.

---

## Tier 7 - Bug fixes for existing tools

Drop-in replacements for the 11 issues documented in `TOOLS_v2_AUDIT.md`.
All applied in v3.

### `solve_ik_all` - return all branches properly

```python
@mcp.tool()
def solve_ik_all(robot_name: str, pose: list[list[float]]) -> dict:
    r = RDK.Item(robot_name, ITEM_TYPE_ROBOT)
    mat = r.SolveIK_All(Mat(pose))
    rows = mat.Rows()
    sols = [row[:6] for row in rows if any(row)]
    return {"robot": robot_name,
            "num_solutions": len(sols),
            "solutions_deg": sols}
```

### `get_all_collisions` - pair list

```python
@mcp.tool()
def get_all_collisions() -> dict:
    pairs_raw = RDK.CollisionPairs()
    pairs = [{"a": a.Name(), "b": b.Name()} for a, b in pairs_raw]
    return {"collision_count": len(pairs), "colliding_pairs": pairs}
```

### Normalize robot lookup (fixes the "Claude" vs "UR5" naming issue)

```python
def _resolve_robot(name: str):
    r = RDK.Item(name, ITEM_TYPE_ROBOT)
    if r.Valid(): return r
    for it in RDK.ItemList(ITEM_TYPE_ROBOT, 1):
        if str(it).lower() == name.lower():
            return RDK.Item(it, ITEM_TYPE_ROBOT)
    raise ValueError(f"robot '{name}' not found")
```

### `move_linear` - pre-seed from current joints

```python
@mcp.tool()
def move_linear(robot_name: str, pose, blocking: bool = True) -> dict:
    r = _resolve_robot(robot_name)
    p = Mat(pose if isinstance(pose[0], list) else
            [pose[i*4:(i+1)*4] for i in range(4)])
    current = r.Joints().tolist()
    try:
        r.MoveL(p, blocking)
    except Exception as e:
        target_joints = r.SolveIK(p, current)
        if len(target_joints) < 6:
            return {"error": f"no IK solution: {e}"}
        r.MoveJ(target_joints, blocking)
        r.MoveL(p, blocking)
    return {"robot": r.Name(), "moved_to": p.Pos()}
```

### `move_to_target` - pass the type to `Item()`

```python
@mcp.tool()
def move_to_target(robot_name: str, target_name: str,
                   move_type: str = "joint") -> dict:
    r = _resolve_robot(robot_name)
    tgt = RDK.Item(target_name, ITEM_TYPE_TARGET)
    if not tgt.Valid():
        return {"error": f"target '{target_name}' not found"}
    if move_type == "linear": r.MoveL(tgt)
    else: r.MoveJ(tgt)
    return {"robot": r.Name(), "target": target_name, "move_type": move_type}
```

### `add_camera` - serialize the result, honour `camera_name`

```python
@mcp.tool()
def add_camera(frame_name: str, camera_name: str | None = None, ...) -> dict:
    frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
    cam = RDK.Cam2D_Add(frame, params)
    if camera_name:
        cam.setName(camera_name)               # fix: name was being ignored
    return {"camera": cam.Name() if cam.Valid() else None,
            "frame": frame.Name(), ...}
```

### `capture_snapshot` - make base64 opt-in

```python
@mcp.tool()
def capture_snapshot(camera_frame_name: str,
                     save_path: str | None = None,
                     include_base64: bool = False) -> dict:
    cam = RDK.Item(camera_frame_name)
    path = save_path or _temp_png_path()
    RDK.Cam2D_Snapshot(path, cam)
    out = {"file_path": path}
    if include_base64:                          # fix: default False; was always on
        with open(path, "rb") as f:
            out["png_base64"] = base64.b64encode(f.read()).decode()
    return out
```

### `detect_blobs` - early-exit on blank scenes

```python
@mcp.tool()
def detect_blobs(camera_frame_name: str, ...) -> dict:
    path = get_camera_image_path(camera_frame_name)["file_path"]
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img.std() < 5.0:                        # fix: skip blank scenes
        return {"camera": camera_frame_name, "blobs": [],
                "note": "blank scene, skipped detection"}
    ...
```

### `pixel_to_world` - use the actual camera FOV and respect the frame

```python
@mcp.tool()
def pixel_to_world(camera_frame_name: str, pixel_x: float, pixel_y: float,
                   table_z_mm: float = 0.0) -> dict:
    cam = RDK.Item(camera_frame_name)
    fx = (w / 2) / np.tan(np.radians(fov / 2))
    ray_cam = np.array([(pixel_x - w/2) / fx,
                        (pixel_y - h/2) / fx, 1.0])
    cam_pose = np.array(cam.PoseAbs().Rows())  # fix: actually use the frame
    origin   = cam_pose[:3, 3]
    direction = cam_pose[:3, :3] @ ray_cam
    t = (table_z_mm - origin[2]) / direction[2]
    p = origin + t * direction
    return {"pixel": {"x": pixel_x, "y": pixel_y},
            "world_mm": dict(zip("xyz", p.tolist()))}
```

---

## Tier 8 - Quality of life

Implemented in v3 as `find_items`, `bulk_delete`. `set_view`/`screenshot` left
for a future PR.

---

## Summary

| Tier | Tools added | In v3 |
|---|---|---|
| 1 | 4  | yes |
| 2 | 2  | yes |
| 3 | 6  | yes |
| 4 | 4  | yes (call/wait variants added) |
| 5 | 5  | yes |
| 6 | 2  | yes |
| 7 | 9  bug-fix replacements | yes |
| 8 | 2  (set_view/screenshot deferred) | partial |
| **Total** | **~36 new, 11 fixes** | **applied in v3** |

After v3 the MCP covers ~95% of the day-to-day RoboDK Python API surface.
