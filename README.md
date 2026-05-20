# RoboDK MCP Server

Connect Claude directly to RoboDK using the Model Context Protocol (MCP). Control robots, capture camera images, run computer vision, detect and manipulate objects, check collisions, and execute vision-guided pick & place — all through natural language.

## Repository Structure

```
v1/  robodk_mcp_server.py      # 20 tools: robot motion, kinematics, programs
v2/  robodk_mcp_server_v2.py   # 41 tools: adds camera, vision, objects, collision, pick & place
```

## Requirements

```bash
pip install mcp robodk opencv-python-headless
```

RoboDK must be running before Claude uses any tool.

## Quick Start

1. Install dependencies
2. Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "robodk": {
      "command": "C:\\Users\\YourUser\\AppData\\Local\\Programs\\Python\\Python313\\python.exe",
      "args": ["C:\\path\\to\\robodk_mcp_server_v2.py"]
    }
  }
}
```

3. Start RoboDK, restart Claude Desktop

## Tool Groups (v2)

| Group | Tools | Description |
|-------|-------|-------------|
| Station | 5 | Connect, list items, load station, simulation speed |
| Robot Motion | 7 | MoveJ, MoveL, get/set joints, TCP pose, speed |
| Kinematics | 3 | FK, IK (single), IK (all solutions) |
| Camera | 4 | Add camera, capture snapshot, reposition |
| Vision | 3 | Blob detection, colour detection (OpenCV), pixel-to-world |
| Objects | 7 | Move, colour, show/hide, attach/detach, import, delete |
| Collision | 4 | Pair check, all-pairs, enable/disable, ray cast |
| Pick & Place | 3 | Plan grasp, execute pick, execute place, vision pipeline |
| Programs | 2 | List and run programs |

## Example Prompts

- *"What robots are in my station?"*
- *"Move the UR5 to home position"*
- *"Capture a snapshot from Camera 1"*
- *"Detect all orange objects on the table"*
- *"Solve IK for a point 500mm above the robot base"*
- *"Pick the cube with the Adept Cobra and place it at (400, 200, 0)"*

## Architecture

```
Claude Desktop
     |
     | MCP / stdio
     v
robodk_mcp_server_v2.py  (local process)
     |
     | TCP port 20500
     v
RoboDK.exe
     |
     v
Simulated or physical robot
```

## Known Limitations

- Camera creation (`add_camera`) requires a paid RoboDK license
- Pick & place object attachment has timing sensitivity via the API; using RoboDK programs for complex sequences is more robust
- `pixel_to_world` uses a pinhole model approximation and requires accurate FOV and table Z values

## Changelog

See [CHANGELOG.md](CHANGELOG.md)
