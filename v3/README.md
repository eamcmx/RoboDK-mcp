# RoboDK-mcp v3 (current: v3.1.0)

Complete rewrite of the v2 server. **~71 tools** (vs 41 in v2), every known
v2/v3 bug fixed (13 total), and new capabilities for building targets,
programs, frames, and tools directly from chat without side-channel Python
or MATLAB scripts.

## Install

```bash
pip install mcp robodk opencv-python-headless numpy
```

## Run

```bash
python v3/robodk_mcp_server_v3.py
# or with a custom RoboDK API host/port:
python v3/robodk_mcp_server_v3.py --host 192.168.1.50 --port 20500
```

## Claude Desktop config

```json
{
  "mcpServers": {
    "robodk": {
      "command": "C:\\Path\\To\\Python\\python.exe",
      "args": ["C:\\path\\to\\v3\\robodk_mcp_server_v3.py"]
    }
  }
}
```

## What's new vs v2

### New tool groups

| Group | Tools |
|---|---|
| Targets / Programs | `add_target`, `add_program`, `program_add_move`, `program_add_call`, `program_add_wait`, `program_clear`, `get_program_instructions`, `program_make_robot_program` |
| Frames / Tools | `add_frame`, `set_active_tool`, `add_tool_from_object` |
| Generic items | `get_pose`, `set_item_pose`, `set_item_name`, `set_parent` |
| Lists / search | `list_targets`, `list_robots`, `list_frames`, `list_tools`, `find_items`, `bulk_delete` |
| Motion testing | `move_joint_test`, `move_linear_test` |
| Station I/O | `save_station`, `get_param`, `set_param`, `set_run_mode`, `show_message` |
| Joint limits | `get_joint_limits`, `set_joint_limits` |

### Bugs fixed (vs v2)

1. **`solve_ik_all`** — returns proper N×6 array (v2 returned a flattened single solution).
2. **`get_all_collisions`** — pairs list matches the count (v2 returned count=1, pairs=[]).
3. **`list_objects_on_table`** — returns the real item name (v2 returned `"."` for attached objects).
4. **Robot name resolution** — accepts the display name (e.g. `"Claude"`) not only the underlying robot name (`"UR5"`). All `*_robot` helpers normalize via `_robot()`.
5. **`move_linear`** — pre-seeds from current joints and falls back through a MoveJ to align IK branch before MoveL. No more "Joint axes outside limits" on reachable targets.
6. **`move_to_target`** — looks up the target with `ITEM_TYPE_TARGET` so the name no longer collides with an object of the same name.
7. **`add_camera`** — returns JSON-serializable metadata (v2 raised "Object of type Item is not JSON serializable").
8. **`add_camera`** — honors the `camera_name` argument (v2 ignored it).
9. **`capture_snapshot`** — base64 payload is opt-in (`include_base64=True`). Default response is just the file path; v2 always returned ~400 KB and blew the LLM context window.
10. **`detect_blobs` / `detect_objects_by_color`** — fast early-exit on near-uniform scenes so a blank camera doesn't hang the LLM turn.
11. **`pixel_to_world`** — actually uses the camera frame's world pose when projecting (v2 returned multi-metre offsets for the image center).

## What's new in v3.1

### Bugs fixed (vs v3.0)

12. **`get_tcp_pose`** — now returns the TCP **in world coordinates** as the docstring promises. v3 returned `SolveFK(Joints) * PoseTool`, i.e. the TCP in the *robot's own base frame*, breaking every cross-robot relative-pose computation (two robots at identical joints reported identical TCPs even when their bases were 1050 mm apart).
13. **`get_robot_joints`** — no longer raises `'float' object is not iterable`. Now uses the documented `Mat.list()` method instead of `Mat.tolist()[0]`.

### Server `instructions` (dual-robot guide)

The server now ships a dual-robot coordination guide in its `instructions`
field (`v3/instructions.py`). Every LLM that loads this MCP automatically
sees:

- **Four frame-handling rules** that eliminate the silent failures we hit
  in multi-robot setups (world-TCP computation, slave IK pinning, target
  frame conversion, IK seeding).
- **Master-slave TCP sync pattern** including the reach precheck.
- **Dual-arm handover, dual-arm pick-and-place, and collision-aware
  coordinated motion** recipes.
- **MATLAB-style Newton-Jacobian IK fallback** with the 12-residual /
  pseudo-inverse formulation, for cases where RoboDK's built-in IK flips
  branches or refuses near-singular targets.
- **Pitfall table** — symptom → cause → fix, drawn from real sessions.

See `v3/instructions.py` for the full text and edit history.

## Dual-robot coordination — worked example

`v3/examples/example_dual_ur5_master_slave.py` loads the bundled station at
`v3/examples/stations/Dual UR5t.rdk` (two UR5s already positioned to face
each other), captures the tool-to-tool offset, and keeps the slave locked
to the master while you drag UR51 in the 3D view.

```bash
pip install robodk
python v3/examples/example_dual_ur5_master_slave.py
```

Then drag or jog **UR51** in RoboDK; **UR52** mirrors it preserving the
offset captured at start-up. Stop with `Ctrl+C`.

The example is intentionally self-contained — it doesn't depend on the MCP
server being running, so you can validate your robodk install before
wiring up Claude Desktop.

## Tool reference

See [`docs/TOOLS.md`](../docs/TOOLS.md) for the full per-tool reference
(signatures, params, returns, underlying RoboDK API mapping).
