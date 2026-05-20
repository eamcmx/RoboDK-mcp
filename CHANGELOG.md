# Changelog

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
