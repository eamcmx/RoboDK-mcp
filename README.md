# RoboDK MCP Server

Connect Claude (or any [Model Context Protocol](https://modelcontextprotocol.io)
client) directly to **RoboDK**. Control robots in natural language: move joints,
solve inverse kinematics, capture camera frames, run computer vision, manipulate
objects, check collisions, and execute vision-guided pick & place.

## Demo

<!-- VIDEO: drag-drop ClaudeRoboDK.mp4 here in the GitHub web editor.
     GitHub will replace this comment with a hosted <video> tag automatically. -->



https://github.com/user-attachments/assets/8995e3a9-16ba-4fcb-9efc-c9d3e7ef18b0



> Claude driving a UR5 through the demo station — touch sequences, contour
> traces, and a finale flourish — entirely from chat.

## Repository structure

```
v1/                            # core server: robot motion, kinematics, programs
  robodk_mcp_server.py
v2/                            # adds cameras, vision, objects, collision, pick & place
  robodk_mcp_server_v2.py
station/
  Claude.rdk                   # the demo station used throughout this README
ClaudeRoboDK.mp4               # short demo video (also linked above)
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
         "args": ["C:\\path\\to\\v2\\robodk_mcp_server_v2.py"]
       }
     }
   }
   ```

3. Start RoboDK and open a station.
4. Restart the MCP client.

## The demo station

The reference station ([`station/Claude.rdk`](station/Claude.rdk)) ships with:

| Item | Type | Notes |
|---|---|---|
| `Claude` | Robot | UR5, 6 DOF |
| `UR5` | Frame | UR5 base reference frame |
| `UR5 Base` | Tool | Flange-mounted tool plate (no gripper) |
| `Table` | Reference target at world origin | |
| `Cube1` … `Cube7` | Target | 100 mm wireframe cubes, all `Rz = −90°`, frame at one bottom corner. Cube body extends `+X, −Y` in world from the frame, so the top-face center is at `(frame_x + 50, frame_y − 50, 100)`. |

Default cube layout (world coordinates, mm):

```
Cube1  (360,   60, 0)        Cube4  (130, -250, 0)
Cube2  (370, -140, 0)        Cube5  (250, -530, 0)
Cube3  (370, -310, 0)        Cube6  ( 60, -530, 0)
                             Cube7  (-110, -550, 0)
```

UR5 base is at world origin with the standard home pose
`[0, −90, −90, −90, 90, 0]°` placing the TCP near `(487, −109, 432)` pointing
down.

## Tool groups (v2)

| Group | Tools |
|---|---|
| **Station** | `get_connection_status`, `get_station_items`, `load_station`, `list_objects_on_table`, `set_simulation_speed`, `render` |
| **Robot motion** | `get_robot_joints`, `set_robot_joints`, `move_joint`, `move_linear`, `move_to_target`, `get_tcp_pose`, `set_robot_speed` |
| **Kinematics** | `solve_fk`, `solve_ik`, `solve_ik_all` |
| **Programs** | `list_programs`, `run_program` |
| **Camera** | `add_camera`, `set_camera_pose`, `capture_snapshot`, `get_camera_image_path` |
| **Vision** | `detect_blobs`, `detect_objects_by_color`, `pixel_to_world` |
| **Objects** | `get_object_pose`, `set_object_pose`, `set_object_color`, `set_object_visible`, `add_object_from_file`, `delete_item`, `attach_object_to_robot`, `detach_object` |
| **Collision** | `check_collision`, `get_all_collisions`, `set_collision_detection`, `check_ray_collision` |
| **Pick & Place** | `plan_grasp_pose`, `execute_pick`, `execute_place`, `vision_pick_by_color` |

## Example prompts

Real prompts that worked end-to-end against the demo station:

- *"List every item in the station."*
- *"Move the UR5 home, then touch the top face center of Cube1, then return home."*
- *"Touch Cube1, Cube5, Cube2, Cube6, Cube3, Cube7, Cube4 in that order — approach from 100 mm above each, descend to the top face, retract."*
- *"Trace the perimeter of each cube's top face (a 100 mm square at z = 100) for all 7 cubes."*
- *"Weave the tool through the gaps between the cubes without touching any of them."*
- *"Slow the simulation to 1× and the robot speed to 150 mm/s, then re-run the touch sequence."*
- *"Solve IK for a downward-pointing TCP at world (300, −250, 200), and show me the joint angles."*

For each, the assistant chains the relevant tools — typically
`get_object_pose` → compute target → `solve_ik` → `move_joint` — without you
writing a line of robot code.

## Architecture

```
MCP client (Claude Desktop / Claude Code / etc.)
        |
        | MCP over stdio
        v
robodk_mcp_server_v2.py     (local Python process)
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
- **Cube targets** in the demo station are RoboDK *targets* (type 5), not
  meshes (type 4). The wireframe cube body extends from the frame's corner in
  the local `+X, +Y, +Z` direction — with the demo cubes' `Rz = −90°` that
  maps to `+X, −Y, +Z` in world coordinates. Centering your TCP on the wrong
  offset is the most common reason a "touch" misses.
- **IK branch flips:** `solve_ik` returns whichever branch is nearest to the
  robot's *current* joint configuration. When traversing multiple targets,
  pre-position the robot near the previous target (via `set_robot_joints`)
  before re-solving to keep the arm in a consistent branch.
- **`pixel_to_world`** uses a pinhole approximation — accurate FOV and a
  correct table-Z assumption are required.
- **Object attachment timing:** the `execute_pick` / attachment path has known
  timing quirks under the Python API. Wrapping pick-and-place inside a native
  RoboDK *program* (then calling `run_program`) is more reliable for complex
  sequences.

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
