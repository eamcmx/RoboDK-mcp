# RoboDK-mcp Tool Reference (v2)

Complete reference for every MCP tool exposed by `v2/robodk_mcp_server_v2.py`,
verified live against an open RoboDK station (UR5 + 7 cube targets + Generic
Dispenser object) on 2026‑05‑22. Every tool listed below is wrapping the
RoboDK Python API (the `robodk.robolink` package); the underlying call is
noted next to each entry.

> **Note:** this document audits **v2** and lists the 11 bugs that motivated
> the v3 rewrite. v3 fixes every issue flagged here. See `v3/README.md` and
> `docs/GAPS_v2_AUDIT.md` for the design notes that became v3.

Item types used throughout: `1=Robot, 2=Frame, 3=Tool, 4=Object, 5=Target, 6=Program, 19=Camera`.

---

## 0. Connectivity

### `get_connection_status()`
**Wraps:** `Robolink()` ping.
**Returns:** `{connected: bool, station: <path>}`.
**Notes:** Connects to the running RoboDK on `localhost:20500`. Reconnects lazily.

```text
{ "connected": true, "station": "C:/Users/eamer/Downloads/Elsevier" }
```

---

## 1. Station inspection

### `get_station_items()`
**Wraps:** `RDK.ItemList(-1)` plus `Item.Type()` for each.
**Returns:** list of `{name, type, valid}` for every item in the tree.

### `list_objects_on_table(table_name, z_tolerance_mm=50)`
**Wraps:** iterate `RDK.ItemList()` and filter by Z proximity to the named table.
**Returns:** list of items near the table surface.
⚠ **Bug:** the active Generic Dispenser object is returned with name `"."`.

### `list_programs()`
**Wraps:** `RDK.ItemList(ITEM_TYPE_PROGRAM, 1)`.
**Returns:** list of program names (empty on a fresh station).

### `get_object_pose(object_name)`
**Wraps:** `Item.PoseAbs()` (absolute pose).
**Returns:** 4×4 matrix (row‑major flat) plus `{x,y,z}` mm.

### `get_robot_joints(robot_name)`
**Wraps:** `Item.Joints()` → degrees.
⚠ **Bug:** rejects the robot's display name (`"Claude"`); only the underlying name (`"UR5"`) is accepted, even though `get_station_items` lists the robot as `"Claude"`.

### `get_tcp_pose(robot_name)`
**Wraps:** `Item.SolveFK(joints)` chained with the **currently rendered tool TCP** (so if you attach an object to the flange the returned pose follows it).
**Returns:** 4×4 matrix + `{x,y,z}` mm.
**Important:** this is **not** the same as `solve_fk()` — see §3.

---

## 2. Kinematics

### `solve_fk(robot_name, joints)`
**Wraps:** `Item.SolveFK(joints)` → pose of the **active tool** (not any attached object).
**Returns:** 4×4 matrix + `{x,y,z}` mm.

### `solve_ik(robot_name, pose)`
**Wraps:** `Item.SolveIK(pose)` → nearest single solution from the current robot configuration.
**Input:** 4×4 nested list.
**Returns:** 6 joint angles in degrees, or `[0]` if no solution.
**Tip:** to lock a branch, call `set_robot_joints(...)` first so SolveIK seeds from a known posture.

### `solve_ik_all(robot_name, pose)`
**Wraps:** `Item.SolveIK_All(pose)` → up to 8 branches.
⚠ **Bug:** returns a flattened single‑solution array instead of N×6. Reports `num_solutions: 8` but the `solutions_deg` field is the wrong shape.

---

## 3. Robot motion

### `move_joint(robot_name, joints, blocking=true)`
**Wraps:** `Item.MoveJ(joints)`. The workhorse — used throughout this session.

### `move_linear(robot_name, pose, blocking=true)`
**Wraps:** `Item.MoveL(pose)`.
⚠ **Bug:** unreliable. Often returns "Joint axes outside limits" or an empty error even when (a) the target pose has a valid `solve_ik` solution and (b) the move is 1 mm in a non‑singular configuration. Likely the wrapper isn't pre‑seeding from current joints or isn't honouring tool/frame state.
**Workaround:** `solve_ik(...)` → `move_joint(...)`.

### `move_to_target(robot_name, target_name, move_type='joint')`
**Wraps:** `Item.MoveJ(target)` or `MoveL(target)` where `target` is an existing Target item.
⚠ **Bug:** rejects valid existing targets ("Table" returned `Invalid item`). The wrapper likely isn't requesting `ITEM_TYPE_TARGET` from `RDK.Item(name)`, so names that exist as multiple types collide.

### `set_robot_joints(robot_name, joints)`
**Wraps:** `Item.setJoints(joints)`. Instant snap, no motion.

### `set_robot_speed(robot_name, speed_mm_s, accel_mm_s2=None)`
**Wraps:** `Item.setSpeed(speed_mm_s, accel_mm_s2)`.

### `set_simulation_speed(speed)`
**Wraps:** `RDK.setSimulationSpeed(speed)`. `1`=real‑time, `5`=default, `0.5`=half.

### `render(refresh=true)`
**Wraps:** `RDK.Render(refresh)`. `false` pauses the display for batch ops, `true` refreshes.

---

## 4. Programs

### `run_program(program_name)`
**Wraps:** `RDK.Item(name).RunProgram()`. Blocks until finished.
**Note:** there is currently no way to *create* a program from the v2 MCP — fixed in v3 (`add_program` + `program_add_move`).

---

## 5. Scene objects, frames and tools

### `add_object_from_file(file_path, object_name=None, x=0, y=0, z=0)`
**Wraps:** `RDK.AddFile(path)` + optional `Item.setPose(transl(x,y,z))`.
**Accepted formats:** `.stl, .step, .obj, .wrl`.

### `set_object_pose(object_name, x, y, z, rx_deg=0, ry_deg=0, rz_deg=0)`
**Wraps:** `Item.setPose(transl(x,y,z) * rotxyz)` (Euler XYZ).
**Note:** only XYZ + Euler — no way to pass a full 4×4 matrix in v2. Fixed in v3 (`set_item_pose`).

### `set_object_color(object_name, r, g, b, a=1)`
**Wraps:** `Item.Recolor([r,g,b,a])`. Floats 0–1.

### `set_object_visible(object_name, visible=true)`
**Wraps:** `Item.setVisible(visible)`.

### `delete_item(item_name)`
**Wraps:** `Item.Delete()`. Irreversible.

### `attach_object_to_robot(object_name, robot_name)`
**Wraps:** `Item.setParentStatic(robot)`. The object moves with the flange.

### `detach_object(object_name, new_parent_name=None)`
**Wraps:** `Item.setParentStatic(new_parent or RDK.ActiveStation())`.

### `load_station(file_path)`
**Wraps:** `RDK.AddFile(path)` for `.rdk` files. Opens the station.

---

## 6. Cameras and vision (OpenCV)

### `add_camera(frame_name, camera_name=None, fov_deg=30, focal_length_mm=6, far_length_mm=2000, width_px=640, height_px=480)`
**Wraps:** `RDK.Cam2D_Add(reference_frame, params)`.
⚠ **Bug 1:** the wrapper returns the new `Item` object directly → `"Object of type Item is not JSON serializable"`. The camera IS created in the station despite the error.
⚠ **Bug 2:** `camera_name` is ignored; the camera is always named `"Camera 1"`, `"Camera 2"`, … by RoboDK's internal counter.
**Camera convention:** the frame's +Z is the look direction, +Y is image‑down.
**Educational license caveat:** capped at 2 simulated cameras.

### `set_camera_pose(frame_name, x, y, z, rx_deg=0, ry_deg=0, rz_deg=0)`
**Wraps:** `Item.setPose(...)` on the camera's anchor frame.
**Note:** because cameras are attached to **existing** frames, repositioning the camera repositions the frame — so don't anchor a test camera to the robot base frame.

### `capture_snapshot(camera_frame_name, save_path=None)`
**Wraps:** `RDK.Cam2D_Snapshot(path, camera)` + base64 of the saved PNG.
⚠ **Bug:** response includes the base64 PNG (~384 KB for a 640×480 capture) and can exceed an LLM tool‑result budget.
**Workaround:** use `get_camera_image_path` instead and let the consumer read the file themselves.

### `get_camera_image_path(camera_frame_name, save_path=None)`
**Wraps:** same `Cam2D_Snapshot` but returns only `{file_path}`. ✅

### `detect_blobs(camera_frame_name, min_area_px=500, max_area_px=50000)`
**Wraps:** snapshot → OpenCV `SimpleBlobDetector`.
⚠ **Bug:** times out (no result in ~30 s) when the camera scene has nothing in view. Likely the blob detector is iterating to convergence on a blank image. Consider a fast early‑exit.

### `detect_objects_by_color(camera_frame_name, color='red', min_area_px=300)`
**Wraps:** snapshot → HSV threshold + contour detection. Same timeout issue as `detect_blobs` on blank scenes.

### `pixel_to_world(camera_frame_name, pixel_x, pixel_y, table_z_mm=0, fov_deg=30, image_width_px=640, image_height_px=480)`
**Wraps:** pinhole projection through the camera frame onto a `z=table_z` plane.
⚠ **Bug:** returned `(−712, −4777)` for the image center of a camera anchored at world origin looking up — the world XY should be near `(0, 0)`. Looks like a sign/frame mistake in the ray equation, or the `fov_deg` default doesn't actually match the camera that was created.

### `plan_grasp_pose(...)`
Builds a top‑down grasp pose above an object. Not exercised live this session.

---

## 7. Collisions

### `check_collision(item1_name, item2_name)`
**Wraps:** `RDK.Collision(item1, item2)`. Returns `{collision: bool}`.

### `check_ray_collision(x1, y1, z1, x2, y2, z2)`
**Wraps:** `RDK.Collision_Line([p1, p2])`. Returns hit + item picked.

### `set_collision_detection(enabled=true)`
**Wraps:** `RDK.setCollisionActive(COLLISION_ON|OFF)`.

### `get_all_collisions()`
**Wraps:** `RDK.Collisions()` + `RDK.CollisionPairs()`.
⚠ **Bug:** in the current state I got `{collision_count: 1, colliding_pairs: []}`. Either `Collisions()` and `CollisionPairs()` disagree, or the pairs list isn't being populated.

---

## 8. Composite pipelines (built on top of the primitives)

### `execute_pick(robot_name, object_name, approach_distance_mm=100, speed_mm_s=100)`
Approach → linear descend → `attach_object_to_robot` → linear retract.
**Caveat:** built on `move_linear`, inherits its reliability problems.

### `execute_place(robot_name, object_name, place_x, place_y, place_z, approach_distance_mm=100, speed_mm_s=100)`
Approach → linear descend → `detach_object` → linear retract.

### `vision_pick_by_color(...)`
Snapshot → colour seg → pixel→world → move object → pick → place.
**Caveat:** depends on the buggy vision tools above.

---

## 9. Known robot/tool naming quirk in this station

The robot item in the tree is named **"Claude"** (type 1, with `"UR5"` as the
parent reference frame of type 2). The MCP wrappers that look up items via
`RDK.Item(name, ITEM_TYPE_ROBOT)` succeed with `"UR5"` because `RDK.Item("UR5",
ITEM_TYPE_ROBOT)` falls back through child resolution; calls without the type
argument resolve `"Claude"` correctly but motion wrappers don't seem to. This
is the most confusing thing for new users and should be normalized in the
server.

---

## 10. What's missing in v2 (addressed in v3)

See `docs/GAPS_v2_AUDIT.md` for the complete list. Highest‑value gaps:

| Capability | Tool added in v3 |
|---|---|
| Create a target in the tree | `add_target` |
| Create a program | `add_program` + `program_add_move` |
| Set the active TCP on a robot | `set_active_tool` |
| Create a reference frame | `add_frame` |
| Save the station | `save_station` |
| Inspect program instructions | `get_program_instructions` |
