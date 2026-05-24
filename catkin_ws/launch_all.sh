#!/bin/bash
# =============================================================================
# MechDog — Arranque interno del contenedor
# Ejecutar DENTRO del contenedor mechdog_viz:
#   bash /app/catkin_ws/launch_all.sh
# =============================================================================
set -e
source /app/catkin_ws/devel/setup.bash

# 1. Limpieza preventiva de procesos anteriores
echo "[1/5] Limpiando procesos anteriores..."
killall -9 gzserver gzclient rosmaster roslaunch python3 2>/dev/null || true
sleep 3
echo "      Listo"

# 2. Lanzar simulacion
echo "[2/5] Iniciando Gazebo (modo headless)..."
roslaunch mechdog_sim simulation.launch gui:=false rviz:=false > /tmp/sim.log 2>&1 &

# 3. Esperar /clock — indica que gzserver esta listo
echo "[3/5] Esperando Gazebo..."
TIMEOUT=90
ELAPSED=0
while true; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if rostopic list 2>/dev/null | grep -q "^/clock$"; then
        echo "      Gazebo listo (${ELAPSED}s)"
        break
    fi
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "[ERR] Gazebo no respondio. Log:" && tail -20 /tmp/sim.log
        exit 1
    fi
done

# 4. Esperar spawn del robot (odometria activa)
echo "[4/5] Esperando odometria del robot..."
for i in $(seq 1 20); do
    sleep 2
    if timeout 2 rostopic echo /mechdog/odom -n 1 2>/dev/null | grep -q "frame_id"; then
        echo "      Robot spawneado OK"
        break
    fi
    if [ $i -eq 20 ]; then
        echo "      (Advertencia: odometria no confirmada, continuando...)"
    fi
done

# 5. Lanzar navegacion
echo "[5/5] Iniciando stack de navegacion..."
roslaunch mechdog_navigation navigation.launch environment:=simulation > /tmp/nav.log 2>&1 &

# Esperar los 5 nodos
sleep 8
echo ""
echo "═══════════════════════════════════════════════"
echo "  Sistema listo. Nodos activos:"
rosnode list 2>/dev/null | grep -E '(gazebo|planner|safe|occupancy|manager|noise|sensor)' | sed 's/^/  /'
echo ""
echo "  Para enviar un goal:"
echo "  rostopic pub /mechdog/goal geometry_msgs/PoseStamped"
echo "    '{header: {frame_id: odom}, pose: {position: {x: 3.0},"
echo "     orientation: {w: 1.0}}}' --once"
echo "═══════════════════════════════════════════════"

wait
