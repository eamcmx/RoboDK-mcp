# RoboDK MCP Server

Connect Claude (or any [Model Context Protocol](https://modelcontextprotocol.io)
client) directly to **RoboDK**. Control robots in natural language: build
targets and programs, move joints, solve inverse kinematics, capture camera
frames, run computer vision, manipulate objects, check collisions, and execute
vision-guided pick & place — all from chat.

## Demo

https://github.com/user-attachments/assets/8995e3a9-16ba-4fcb-9efc-c9d3e7ef18b0

> Claude driving a UR5 through the demo station — touch sequences, contour
> traces, and a finale flourish — entirely from chat.

## Versions

| Version | Tools | Status |
|---|---|---|
| **v3** (current) | **~71** | Recommended. Full coverage of the RoboDK Python API surface that matters in chat: targets, programs, frames, active tool, save station, motion testing, generic pose handling. Every known v2 bug fixed. |
| v2 | ~41 | Stable, kept for backwards compatibility. |
| v1 | ~20 | Original release — robot motion + kinematics only. |

See [CHANGELOG.md](CHANGELOG.md) for the full diff between versions.

## Repository structure

```
v1/   robodk_mcp_server.py             original release (~20 tools)
v2/   robodk_mcp_server_v2.py          + cameras, vision, objects, pick & place
v3/   robodk_mcp_server_v3.py          + targets, programs, frames, active tool,
      README.md                        save station, generic pose, joint limits,
                                       11 v2 bug fixes -- 57/57 smoke tests pass

docs/ TOOLS_v2_AUDIT.md                live-tested v2 reference + bug log
      GAPS_v2_AUDIT.md                 gap analysis that motivated v3

tests/ smoke_test_v3.py                live test harness for all 71 v3 tools

station/ Claude.rdk                    demo station used throughout this README
ClaudeRoboDK.mp4                       demo video embedded above
CHANGELOG.md                           per-version changelog
```

## Requirements

```bash
pip install mcp robodk opencv-python-headless numpy
```

RoboDK must be running before the MCP client makes any tool call. The server
connects to RoboDK over TCP on `localhost:20500` (RoboDK's default API port).

## Quick start

1. Install the dependencies above.
2. Add the server to `claude_desktop_config.json` (or your MCP client's config):

   ```json
   {
     "mcpServers": {
       "robodk": {
         "command": "C:\\Users\\YourUser\\AppData\\Local\\Programs\\Python\\Python313\\python.exe",
         "args": ["C:\\path\\to\\v3\\robodk_mcp_server_v3.py"]
       }
     }
   }
   ```

   To stick with v2 instead, point `args` at `v2/robodk_mcp_server_v2.py`.

3. Start RoboDK and open a station (`station/Claude.rdk` reproduces every
   example here).
4. Fully quit and restart the MCP client.

## Run the smoke test

Before flipping a real workflow over to v3, run the live smoke test against
your station:

```bash
python tests/smoke_test_v3.py
```

The test imports `v3/robodk_mcp_server_v3.py` directly (no MCP transport),
auto-discovers items in the open station, exercises every tool with safe
arguments, cleans up after itself, and prints a PASS/FAIL/SKIP report. Exit
code 0 if everything green. Against `Claude.rdk` the expected result is
57 PASS / 0 FAIL.

## Tool groups (v3)

| Group | Tools |
|---|---|
| **Connectivity** | `get_connection_status` |
| **Station inspection** | `get_station_items`, `list_objects_on_table`, `list_programs`, `list_targets`, `list_robots`, `list_frames`, `list_tools`, `find_items` |
| **Poses / kinematics** | `get_object_pose`, `get_pose`, `get_robot_joints`, `get_tcp_pose`, `solve_fk`, `solve_ik`, `solve_ik_all` |
| **Robot motion** | `set_robot_joints`, `move_joint`, `move_linear`, `move_to_target`, `set_robot_speed`, `set_simulation_speed`, `render`, `move_joint_test`, `move_linear_test` |
| **Programs / targets / frames / tools** (new in v3) | `add_target`, `add_program`, `program_add_move`, `program_add_call`, `program_add_wait`, `program_clear`, `get_program_instructions`, `run_program`, `program_make_robot_program`, `add_frame`, `set_active_tool`, `add_tool_from_object` |
| **Scene objects** | `add_object_from_file`, `set_object_pose`, `set_item_pose`, `set_object_color`, `set_object_visible`, `set_item_name`, `set_parent`, `delete_item`, `bulk_delete`, `attach_object_to_robot`, `detach_object`, `load_station`, `save_station` |
| **Cameras & vision** | `add_camera`, `set_camera_pose`, `get_camera_image_path`, `capture_snapshot`, `detect_blobs`, `detect_objects_by_color`, `pixel_to_world` |
| **Collisions** | `check_collision`, `check_ray_collision`, `set_collision_detection`, `get_all_collisions` |
| **Composite pipelines** | `plan_grasp_pose`, `execute_pick`, `execute_place`, `vision_pick_by_color` |
| **Station I/O** | `get_param`, `set_param`, `set_run_mode`, `show_message`, `get_joint_limits`, `set_joint_limits` |

## What's new in v3 (vs v2)

**Build targets and programs from chat.** v2 could only run programs that
already existed in the station tree. v3 adds `add_target`, `add_program`,
`program_add_move`, `add_frame`, and friends — so you can ask the assistant
to "build a touch program that visits Cube1..Cube7" and end up with a real
Program item in the station tree, runnable from the GUI or via `run_program`.

**Active tool / TCP control.** v3 adds `set_active_tool` and
`add_tool_from_object`. Attaching a dispenser, gripper or scribe to the
robot's flange makes the entire FK/IK chain operate on the tool tip — no
more manual offset math.

**11 v2 bugs fixed:**

1. `solve_ik_all` returns a proper N×6 array (v2 returned a flattened
   single solution).
2. `get_all_collisions` pairs list matches the count.
3. `list_objects_on_table` returns real names (v2 returned `"."` for
   attached objects).
4. Robot name resolution accepts display names (e.g. `"Claude"`) not just
   underlying names (`"UR5"`).
5. `move_linear` pre-seeds from current joints and falls back through a
   MoveJ to align the IK branch — no more "Joint axes outside limits" on
   reachable targets.
6. `move_to_target` uses `ITEM_TYPE_TARGET` so it no longer fails on names
   that exist as both a Target and another type.
7. `add_camera` returns JSON-serializable metadata (v2 raised
   `"Object of type Item is not JSON serializable"`).
8. `add_camera` honors the `camera_name` argument (v2 ignored it).
9. `capture_snapshot` base64 payload is opt-in (default response is just
   the file path; v2 always returned ~400 KB and blew the LLM context).
10. `detect_blobs` and `detect_objects_by_color` early-exit on near-uniform
    scenes (v2 timed out).
11. `pixel_to_world` actually uses the camera frame's world pose when
    projecting (v2 returned multi-metre offsets for the image center).

See [docs/TOOLS_v2_AUDIT.md](docs/TOOLS_v2_AUDIT.md) for the full live-tested
audit of v2 that motivated each of these fixes, and
[docs/GAPS_v2_AUDIT.md](docs/GAPS_v2_AUDIT.md) for the gap analysis that
became the v3 tool surface.

## The demo station

The reference station ([`station/Claude.rdk`](station/Claude.rdk)) is what
the test harness and every example here assume.

| Tree item | RoboDK type | Notes |
|---|---|---|
| `Claude` | Station (1) | The `.rdk` container — *not* a callable robot. |
| `UR5` | Robot (2) | The actual UR5 — pass this as `robot_name`. |
| `UR5 Base` | Frame (3) | Robot base reference frame. |
| `.` | Tool (4) | Generic Dispenser attached to the flange. |
| `Table` | Object (5) | Flat object at world origin used as table reference. |
| `Cube1` … `Cube7` | Object (5) | 100 mm cube meshes, `Rz = −90°`, frame at one bottom corner. Cube body extends `+X, −Y` in world from the frame, so the top-face center is at `(frame_x + 50, frame_y − 50, 100)`. |

Default cube layout (world coordinates, mm):

```
Cube1  (360,   60, 0)        Cube4  (130, -250, 0)
Cube2  (370, -140, 0)        Cube5  (250, -530, 0)
Cube3  (370, -310, 0)        Cube6  ( 60, -530, 0)
                             Cube7  (-110, -550, 0)
```

UR5 base is at world origin with the standard home pose
`[0, −90, −90, −90, 90, 0]°` placing the OLD TCP near `(487, −109, 432)`
pointing down. With the Generic Dispenser attached, the dispenser tip is
offset `(0, −162, −185)` mm from the OLD TCP in world coordinates at home.

## Example prompts

Real prompts that worked end-to-end against the demo station:

- *"List every robot and frame in the station."*
- *"Move the UR5 home, then touch the top face center of Cube1, then return home."*
- *"Touch Cube1, Cube5, Cube2, Cube6, Cube3, Cube7, Cube4 in that order — approach from 100 mm above each, descend to the top face, retract."*
- *"Trace the perimeter of each cube's top face (a 100 mm square at z = 100) for all 7 cubes."*
- *"Weave the tool through the gaps between the cubes without touching any of them."*
- *"Slow the simulation to 1× and the robot speed to 150 mm/s, then re-run the touch sequence."*
- *"Solve IK for a downward-pointing TCP at world (300, −250, 200), and show me the joint angles."*
- *"Build a Program that visits each cube top in order, save it to the station so I can run it from the GUI."* (v3 only)
- *"Make the Generic Dispenser the active TCP, then solve IK so the dispenser tip lands on Cube3."* (v3 only)
- *"Save the station to disk."* (v3 only)

For each, the assistant chains the relevant tools — typically
`get_object_pose` → compute target → `solve_ik` → `move_joint` — without you
writing a line of robot code.

## Architecture

```
MCP client (Claude Desktop / Claude Code / etc.)
        |
        | MCP over stdio
        v
robodk_mcp_server_v3.py     (local Python process, FastMCP)
        |
        | RoboDK Python API (TCP port 20500)
        v
RoboDK.exe                  (GUI / simulator)
        |
        v
Simulated robot (or driver-bridged physical robot)
```

The server caches a lazy `Robolink` connection and reconnects automatically if
RoboDK is restarted between tool calls.

## Known limitations

- **Camera creation** (`add_camera`) is capped at 2 simulated cameras on the
  educational RoboDK license. Add more cameras manually in the GUI if needed,
  then reference them by name.
- **IK branch flips:** `solve_ik` returns whichever branch is nearest to the
  robot's *current* joint configuration. When traversing multiple targets,
  pre-position the robot near the previous target (via `set_robot_joints` or
  the `joints_seed` argument) before re-solving to keep the arm in a
  consistent branch.
- **`pixel_to_world`** uses a pinhole approximation. Accurate FOV and a
  correct table-Z assumption are required for the projection to land on the
  real world point.
- **Object attachment timing in composite pipelines:** the `execute_pick` /
  attachment path has known timing quirks under the Python API. For complex
  sequences, prefer building a native RoboDK Program with `add_program` +
  `program_add_move` and calling `run_program`.
- **Collision detection is sticky:** if a previous workflow ended with
  `set_collision_detection(True)` and the robot is at a colliding pose, the
  next `move_joint` will return `{"error": "StoppedError: Collision detected"}`.
  v3 catches the error gracefully; disable collision detection (or move the
  robot off the collision) and retry.

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

---

### Maintainer notes — uploading the demo assets

The repo expects these two files alongside the server code:

- `station/Claude.rdk` — open this in RoboDK to reproduce every example here.
- `ClaudeRoboDK.mp4` — short demo clip embedded at the top of this README.

To attach the video to the README inline (recommended, no commit needed):

1. Open the README on github.com and click the pencil icon to edit.
2. Drag `ClaudeRoboDK.mp4` into the editor where the placeholder URL sits.
3. GitHub uploads it to `user-attachments.githubusercontent.com` and replaces
   the placeholder with a real `<video>` tag.
4. Commit.

To commit the station file:

```bash
mkdir -p station
mv Claude.rdk station/
git add station/Claude.rdk
git commit -m "Add demo station Claude.rdk"
git push
```
