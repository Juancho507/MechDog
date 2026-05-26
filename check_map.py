#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node("check_map", anonymous=True)
topic = rospy.wait_for_message("/mechdog/map", OccupancyGrid, timeout=3)
data = topic.data
w = topic.info.width
h = topic.info.height
total = len(data)
occupied = sum(1 for v in data if v > 70)
free = sum(1 for v in data if v < 0)
unknown = sum(1 for v in data if v == -1)
print("Map: {}x{}, total={}, occupied={}, free={}, unknown={}".format(w, h, total, occupied, free, unknown))
# Find occupied cells near robot
if occupied > 0:
    for y in range(h):
        for x in range(w):
            idx = y * w + x
            if data[idx] > 70:
                wx = x * 0.05 - 25.0
                wy = y * 0.05 - 25.0
                if -2 < wx < 3 and -2 < wy < 2:
                    print("  Occupied at cell({},{}) world({:.2f},{:.2f}) value={}".format(x, y, wx, wy, data[idx]))
