"""
reset_pose.py
=============
Resets both arms of the Motoman SDA10F dual-arm station to the saved
initial joint configuration [0, -60, 0, -80, 0, -30, 0].

Run via MATLAB MCP:
    [s, o] = system('python "path\\to\\reset_pose.py" 2>&1'); disp(o);
"""
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT

rdk = Robolink(robodk_ip="localhost", port=20500)
r = rdk.Item("Right SDA10F", ITEM_TYPE_ROBOT)
l = rdk.Item("Left SDA10F", ITEM_TYPE_ROBOT)
q = [0, -60, 0, -80, 0, -30, 0]
r.setJoints(q)
l.setJoints(q)
print("Both arms reset to initial joints [0,-60,0,-80,0,-30,0]")
