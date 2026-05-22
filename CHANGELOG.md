# Changelog

## v3.0.0 (2026-05-22)

Complete rewrite of the MCP server. **~71 tools** (vs 41 in v2), every known
v2 bug fixed, and new capabilities for building targets, programs, frames,
and tools directly from chat without side-channel Python or MATLAB scripts.

### Added

- **Targets / Programs:** `add_target`, `add_program`, `program_add_move`,
  `program_add_call`, `program_add_wait`, `program_clear`,
  `get_program_instructions`, `program_make_robot_program`
- **Frames / Tools:** `add_frame`, `set_active_tool`, `add_tool_from_object`
- **Generic items:** `get_pose`, `set_item_pose`, `set_item_name`, `set_parent`
- **Lists / search:** `list_targets`, `list_robots`, `list_frames`,
  `list_tools`, `find_items`, `bulk_delete`
- **Motion testing:** `move_joint_test`, `move_linear_test`
- **Station I/O:** `save_station`, `get_param`, `set_param`, `set_run_mode`,
  `show_message`
- **Joint limits:** `get_joint_limits`, `set_joint_limits`

### Fixed (11 v2 bugs)

1. `solve_ik_all` returns proper N×6 array.
2. `get_all_collisions` pairs list matches the count.
3. `list_objects_on_table` returns real names (no more `"."`).
4. Robot name resolution accepts display name (e.g. `"Claude"`) not only the
   underlying name (`"UR5"`).
5. `move_linear` pre-seeds from current joints and falls back through a MoveJ
   to align the IK branch (no more `"Joint axes outside limits"` on
   reachable poses).
6. `move_to_target` uses `ITEM_TYPE_TARGET` so it no longer fails on names
   that exist as both a Target and another type.
7. `add_camera` returns JSON-serializable metadata (v2 raised
   `"Object of type Item is not JSON serializable"`).
8. `add_camera` honors the `camera_name` argument (v2 ignored it).
9. `capture_snapshot` base64 payload is opt-in (default response is just the
   file path; v2 always returned ~400 KB and blew the context window).
10. `detect_blobs` and `detect_objects_by_color` early-exit on near-uniform
    scenes (v2 timed out on blank views).
11. `pixel_to_world` actually uses the camera frame's world pose for ray
    projection (v2 returned multi-metre offsets for the image center).

### Documentation

- `v3/README.md` - install, run, full tool list
- `docs/TOOLS_v2_AUDIT.md` - live-tested v2 reference + bug log
- `docs/GAPS_v2_AUDIT.md` - gap analysis that motivated v3

---

## v2.0.0 (2026-05-20)

### Added
- Camera tools: `add_camera`, `capture_snapshot`, `get_camera_image_path`, `set_camera_pose`
- Vision tools: `detect_blobs`, `detect_objects_by_color` (OpenCV HSV), `pixel_to_world`
- Object manipulation: `get_object_pose`, `set_object_pose`, `set_object_color`, `set_object_visible`, `attach_object_to_robot`, `detach_object`, `add_object_from_file`, `delete_item`, `list_objects_on_table`
- Collision detection: `check_collision`, `get_all_collisions`, `set_collision_detection`, `check_ray_collision`
- Pick & Place pipeline: `plan_grasp_pose`, `execute_pick`, `execute_place`, `vision_pick_by_color`
- Full end-to-end vision-guided pick pipeline in single tool call
- Base64 PNG return from camera snapshots for direct display

### Dependencies
- Added: `opencv-python-headless`, `numpy`

---

## v1.0.0 (2026-05-20)

### Added
- Initial release: 20 tools for robot motion and kinematics
- `get_connection_status`, `get_station_items`, `load_station`, `set_simulation_speed`, `render`
- `get_robot_joints`, `set_robot_joints`, `move_joint`, `move_linear`, `get_tcp_pose`, `move_to_target`
- `solve_fk`, `solve_ik`, `solve_ik_all`
- `get_targets`, `run_program`, `list_programs`
- `set_robot_speed`, `get_item_pose`, `add_reference_frame`
- MCP stdio transport (Claude Desktop) and SSE transport (HTTP)
- Lazy RoboDK connection with auto-reconnect
