#!/bin/bash
# =============================================================================
# MechDog noVNC Startup Script
# Initializes virtual X server, VNC server, and web-based visualization
# =============================================================================

set -e

# Color output for logging
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# =============================================================================
# Environment Configuration
# =============================================================================

export DISPLAY=${DISPLAY:-:0}
RESOLUTION=${RESOLUTION:-1920x1080}
VNC_PORT=5900
NOVNC_PORT=6080

log_info "Starting MechDog visualization environment..."
log_info "Display: $DISPLAY"
log_info "Resolution: $RESOLUTION"

# =============================================================================
# 1. Start Xvfb (Virtual X Server)
# =============================================================================

log_info "Starting Xvfb virtual display..."
Xvfb $DISPLAY -screen 0 ${RESOLUTION}x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
log_info "Xvfb started (PID: $XVFB_PID)"

# Wait for Xvfb to initialize
sleep 2

# Verify Xvfb is running
if ! ps -p $XVFB_PID > /dev/null; then
    log_error "Xvfb failed to start"
    exit 1
fi

# =============================================================================
# 2. Configure and Start Fluxbox (Window Manager)
# =============================================================================

# Write a custom right-click menu with a working terminal and ROS shortcuts
mkdir -p ~/.fluxbox

cat > ~/.fluxbox/menu << 'MENU_EOF'
[begin] (MechDog)
  [exec] (Terminal) {xterm -fa 'Monospace' -fs 12 -ls} <>
  [exec] (RViz) {bash -c "source /opt/ros/noetic/setup.bash && source /app/catkin_ws/devel/setup.bash && QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 rosrun rviz rviz -d /app/catkin_ws/src/mechdog_description/launch/rviz.rviz"} <>
  [exec] (Gazebo GUI) {bash -c "source /opt/ros/noetic/setup.bash && GAZEBO_MASTER_URI=http://simulation:11345 LIBGL_ALWAYS_SOFTWARE=1 gzclient"} <>
  [separator]
  [submenu] (ROS Info)
    [exec] (Node list) {xterm -e "bash -c 'source /opt/ros/noetic/setup.bash && rosnode list; echo; echo Press Enter to close...; read'"} <>
    [exec] (Topic list) {xterm -e "bash -c 'source /opt/ros/noetic/setup.bash && rostopic list; echo; echo Press Enter to close...; read'"} <>
    [exec] (TF tree) {xterm -e "bash -c 'source /opt/ros/noetic/setup.bash && rosrun tf view_frames; echo TF tree saved; echo Press Enter to close...; read'"} <>
  [end]
  [separator]
  [exec] (Send goal (2,2)) {bash -c "source /opt/ros/noetic/setup.bash && source /app/catkin_ws/devel/setup.bash && python3 -c 'import rospy; from geometry_msgs.msg import PoseStamped; rospy.init_node(\"gp\", anonymous=True); pub = rospy.Publisher(\"/mechdog/goal\", PoseStamped, queue_size=1, latch=True); rospy.sleep(0.5); m = PoseStamped(); m.header.frame_id = \"map\"; m.header.stamp = rospy.Time.now(); m.pose.position.x = 2.0; m.pose.position.y = 2.0; m.pose.orientation.w = 1.0; pub.publish(m); print(\"Goal (2,2) published\")'"} <>
  [separator]
  [reconfig] (Reload Config)
  [restart] (Restart Fluxbox)
[end]
MENU_EOF

# Set single workspace, proper toolbar, and no desktop icons
cat > ~/.fluxbox/init << 'INIT_EOF'
session.menuFile: ~/.fluxbox/menu
session.keyFile: ~/.fluxbox/keys
session.styleFile: /usr/share/fluxbox/styles/ubuntu-light
session.configVersion: 13
session.screen0.toolbar.widthPercent: 100
session.screen0.strftimeFormat: %H:%M
session.screen0.toolbar.tools: prevworkspace, workspacename, nextworkspace, systemtray, clock
session.screen0.workspaces: 1
session.screen0.workspaceNames: MechDog,
session.screen0.desktopwheeling: false
session.screen0.desktopMouseWheel: false
session.screen0.fullMaximization: true
session.screen0.window.placement: RowCenterPlacement
session.autoRaise: true
session.autoRaiseDelay: 250
session.cacheMax: 2048
session.cacheLife: 10
session.colorsPerChannel: 128
session.doubleClickInterval: 250
session.focusTabMinWidth: 5
session.imageDither: true
session.justify: center
session.tabs: true
session.tabWidth: 64
INIT_EOF

log_info "Fluxbox configured: 1 workspace, custom menu with terminal + ROS tools"

log_info "Starting Fluxbox window manager..."
fluxbox &
FLUXBOX_PID=$!
log_info "Fluxbox started (PID: $FLUXBOX_PID)"

sleep 1

# =============================================================================
# 3. Start x11vnc (VNC Server)
# =============================================================================

log_info "Starting x11vnc server on port $VNC_PORT..."
x11vnc -display $DISPLAY \
       -forever \
       -shared \
       -rfbport $VNC_PORT \
       -nopw \
       -xkb \
       -ncache 10 \
       -ncache_cr \
       -bg \
       -o /tmp/x11vnc.log

# Wait for x11vnc to start
sleep 2

# Verify x11vnc is listening
if ! netstat -tuln | grep -q ":$VNC_PORT "; then
    log_error "x11vnc failed to start on port $VNC_PORT"
    cat /tmp/x11vnc.log
    exit 1
fi

log_info "x11vnc server started successfully"

# =============================================================================
# 4. Start noVNC (Web VNC Client via websockify)
# =============================================================================

log_info "Starting noVNC/websockify on port $NOVNC_PORT..."

# Find noVNC installation
if [ -d "/usr/share/novnc" ]; then
    NOVNC_PATH="/usr/share/novnc"
elif [ -d "/opt/noVNC" ]; then
    NOVNC_PATH="/opt/noVNC"
else
    log_error "noVNC not found in /usr/share/novnc or /opt/noVNC"
    exit 1
fi

log_info "Using noVNC from: $NOVNC_PATH"

# Start websockify
websockify --web=$NOVNC_PATH \
           $NOVNC_PORT \
           localhost:$VNC_PORT &
WEBSOCKIFY_PID=$!

sleep 2

# Verify websockify is running
if ! ps -p $WEBSOCKIFY_PID > /dev/null; then
    log_error "websockify failed to start"
    exit 1
fi

log_info "noVNC server started (PID: $WEBSOCKIFY_PID)"

# =============================================================================
# 5. Source ROS Workspace
# =============================================================================

log_info "Sourcing ROS Noetic and catkin workspace..."
source /opt/ros/noetic/setup.bash

if [ -f "/app/catkin_ws/devel/setup.bash" ]; then
    source /app/catkin_ws/devel/setup.bash
    log_info "Catkin workspace sourced successfully"
else
    log_warn "Catkin workspace not found at /app/catkin_ws/devel/setup.bash"
    log_warn "You may need to compile the workspace first"
fi

# =============================================================================
# 6. Display Access Information
# =============================================================================

echo ""
echo "======================================================================="
echo "  MechDog Visualization Environment Ready"
echo "======================================================================="
echo ""
echo "  Access the desktop via web browser:"
echo "  http://localhost:$NOVNC_PORT/vnc.html"
echo ""
echo "  Available launch commands:"
echo "  - roslaunch mechdog_sim simulation.launch"
echo "  - roslaunch mechdog_navigation navigation.launch"
echo "  - roslaunch mechdog_description display.launch"
echo ""
echo "  ROS Master URI: $ROS_MASTER_URI"
echo "  Display: $DISPLAY"
echo ""
echo "======================================================================="
echo ""

# =============================================================================
# 7. Cleanup Handler
# =============================================================================

cleanup() {
    log_info "Shutting down visualization services..."
    
    if [ ! -z "$WEBSOCKIFY_PID" ]; then
        kill $WEBSOCKIFY_PID 2>/dev/null || true
        log_info "Stopped websockify"
    fi
    
    killall x11vnc 2>/dev/null || true
    log_info "Stopped x11vnc"
    
    if [ ! -z "$FLUXBOX_PID" ]; then
        kill $FLUXBOX_PID 2>/dev/null || true
        log_info "Stopped fluxbox"
    fi
    
    if [ ! -z "$XVFB_PID" ]; then
        kill $XVFB_PID 2>/dev/null || true
        log_info "Stopped Xvfb"
    fi
    
    log_info "Cleanup complete"
    exit 0
}

trap cleanup SIGTERM SIGINT

# =============================================================================
# 8. Execute Command or Keep Container Running
# =============================================================================

# If arguments provided, execute them
if [ $# -gt 0 ]; then
    log_info "Executing command: $@"
    exec "$@"
else
    # Keep container running
    log_info "Container running. Press Ctrl+C to stop."
    tail -f /dev/null
fi
