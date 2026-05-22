# RoboDK-mcp v3

Complete rewrite of the v2 server. **~71 tools** (vs 41 in v2), every known v2
bug fixed, and new capabilities for building targets, programs, frames, and
tools directly from chat without side-channel Python or MATLAB scripts.

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

## Tool reference

See [`docs/TOOLS.md`](../docs/TOOLS.md) for the full per-tool reference (signatures, params, returns, underlying RoboDK API mapping).
