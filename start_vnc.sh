#!/bin/bash
set -e

# 1. Start virtual display
Xvfb :99 -screen 0 1280x720x24 &
export DISPLAY=:99
sleep 1

# 2. Start VNC server (no password)
x11vnc -display :99 -nopw -listen localhost -xkb -forever &
sleep 1

# 3. Start noVNC (browser interface on port 6080)
websockify --web /usr/share/novnc/ 6080 localhost:5900 &
sleep 1

echo ""
echo "============================================"
echo "  Open your browser at: http://localhost:6080/vnc.html"
echo "============================================"
echo ""

# 4. Run the visualization
python visualize.py "$@"
