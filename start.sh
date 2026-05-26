#!/bin/bash
# =============================================================================
# MechDog — Script de Arranque Limpio (Bridge Network)
# Uso: ./start.sh [all|sim|nav|stop|status]
#
# Con la red mechdog_net, cada servicio corre en su propio contenedor y se
# comunican por DNS interno (http://roscore:11311).
#
#   ./start.sh all     → roscore + simulación + navegación + VNC (default)
#   ./start.sh sim     → roscore + simulación + VNC (sin navegación)
#   ./start.sh nav     → roscore + navegación (sin simulación, requiere Gazebo externo)
#   ./start.sh stop    → Docker compose down
#   ./start.sh status  → Ver estado de todos los servicios
# =============================================================================

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERR]${NC} $1"; }

# ─── Esperar a que roscore esté listo ────────────────────────────────────────
wait_roscore() {
  echo -n "Esperando roscore"
  for i in $(seq 1 30); do
    sleep 2
    if docker compose exec -T roscore bash -c \
       "source /opt/ros/noetic/setup.bash && timeout 2 rostopic list >/dev/null 2>&1" 2>/dev/null; then
      echo "" && ok "roscore listo (${i}x2s)"
      return 0
    fi
    echo -n "."
  done
  echo "" && err "roscore no responde después de 60s"
  exit 1
}

# ─── Esperar a que la simulación publique /clock ──────────────────────────────
wait_sim() {
  echo -n "Esperando Gazebo"
  for i in $(seq 1 45); do
    sleep 2
    if docker compose exec -T roscore bash -c \
       "source /opt/ros/noetic/setup.bash && timeout 2 rostopic list 2>/dev/null | grep -q '^/clock$'" 2>/dev/null; then
      echo "" && ok "Gazebo listo (${i}x2s)"
      return 0
    fi
    echo -n "."
  done
  echo "" && err "Gazebo no respondió en 90s"
  exit 1
}

# ─── Esperar a que la navegación levante sus 5 nodos ─────────────────────────
wait_nav() {
  echo -n "Esperando nodos de navegación"
  for i in $(seq 1 20); do
    sleep 2
    COUNT=$(docker compose exec -T roscore bash -c \
      "source /opt/ros/noetic/setup.bash && \
       timeout 3 rosnode list 2>/dev/null | \
       grep -cE '(global_planner|local_planner|safe_learning|occupancy_grid|navigation_manager|scan_behavior)'" 2>/dev/null)
    if [ "${COUNT}" -ge "5" ] 2>/dev/null; then
      echo "" && ok "${COUNT} nodos de navegación activos"
      return 0
    fi
    echo -n "."
  done
  echo "" && warn "No todos los nodos de navegación arrancaron"
}

# ─── Mostrar estado ───────────────────────────────────────────────────────────
show_status() {
  echo ""
  echo "═══════════════════════════════════════════════"
  echo "  ESTADO DEL SISTEMA MechDog (Bridge Network)"
  echo "═══════════════════════════════════════════════"
  echo ""

  # Contenedores
  for svc in roscore simulation navigation telemetry mechdog_viz; do
    if docker ps --format "{{.Names}}" 2>/dev/null | grep -q "mechdog-${svc}-1"; then
      ok "${svc} — activo"
    else
      err "${svc} — inactivo"
    fi
  done
  echo ""

  # Nodos ROS (solo si roscore responde)
  if docker compose exec -T roscore bash -c \
     "source /opt/ros/noetic/setup.bash && timeout 3 rostopic list >/dev/null 2>&1" 2>/dev/null; then
    echo "Nodos ROS activos:"
    NODES=$(docker compose exec -T roscore bash -c \
      "source /opt/ros/noetic/setup.bash && rosnode list 2>/dev/null" 2>/dev/null)
    echo "${NODES}" | sed 's/^/  /'
    echo ""

    # Topics
    for topic in /clock /mechdog/odom /mechdog/scan /mechdog/ultrasonic /mechdog/navigation_status; do
      if docker compose exec -T roscore bash -c \
         "source /opt/ros/noetic/setup.bash && timeout 2 rostopic echo ${topic} -n 1 2>/dev/null | grep -q '.'" 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} ${topic}"
      else
        echo -e "  ${YELLOW}~${NC} ${topic} (sin datos)"
      fi
    done
  else
    warn "roscore no disponible"
  fi

  echo ""
  echo "noVNC: http://localhost:6080/vnc.html"
  echo "═══════════════════════════════════════════════"
  echo ""
  echo "Para enviar un goal de navegación:"
  echo "  docker compose exec roscore bash -c \\"
  echo "    \"source /app/catkin_ws/devel/setup.bash && \\"
  echo "     rostopic pub /mechdog/goal geometry_msgs/PoseStamped \\"
  echo "     '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \\"
  echo "     --once\""
  echo ""
}

# ─── Acciones ─────────────────────────────────────────────────────────────────
case "${1:-all}" in

  all)
    echo ""
    echo "╔══════════════════════════════════════╗"
    echo "║   MechDog — Arranque Completo        ║"
    echo "╚══════════════════════════════════════╝"
    echo ""
    docker compose up -d roscore
    wait_roscore
    docker compose up -d simulation
    wait_sim
    docker compose up -d navigation
    wait_nav
    docker compose up -d mechdog_viz
    show_status
    ;;

  sim)
    docker compose up -d roscore
    wait_roscore
    docker compose up -d simulation
    wait_sim
    docker compose up -d mechdog_viz
    show_status
    ;;

  nav)
    docker compose up -d roscore
    wait_roscore
    docker compose up -d navigation
    wait_nav
    show_status
    ;;

  stop)
    echo "Deteniendo todos los servicios..."
    docker compose down
    ok "Sistema detenido"
    ;;

  status)
    show_status
    ;;

  *)
    echo "Uso: $0 [all|sim|nav|stop|status]"
    echo ""
    echo "  all    → roscore + simulación + navegación + VNC (default)"
    echo "  sim    → roscore + simulación + VNC"
    echo "  nav    → roscore + navegación"
    echo "  stop   → Docker compose down"
    echo "  status → Ver estado actual"
    exit 1
    ;;
esac
