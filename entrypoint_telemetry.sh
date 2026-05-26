#!/bin/bash
set -e

# Patch rosbridge _buff bug for /clock messages (simulated time)
python3 << 'PYEOF'
import os, glob
path = '/opt/ros/noetic/lib/python3/dist-packages/rosbridge_library/internal/outgoing_message.py'
with open(path) as f:
    c = f.read()
c = c.replace('u"bytes": self._message._buff', 'u"bytes": getattr(self._message, "_buff", b"")')
with open(path, 'w') as f:
    f.write(c)
for f in glob.glob('/opt/ros/noetic/lib/python3/dist-packages/rosbridge_library/internal/__pycache__/outgoing_message*'):
    try: os.remove(f)
    except: pass
print('rosbridge patched OK')
PYEOF

# Launch rosbridge
source /opt/ros/noetic/setup.bash
exec roslaunch rosbridge_server rosbridge_websocket.launch
