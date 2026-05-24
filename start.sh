#!/bin/bash
# =============================================================================
# MechDog — Script de Arranque Limpio
# Uso: ./start.sh [sim|nav|all|stop|status]
#
#   ./start.sh sim     → Solo simulacion Gazebo
#   ./start.sh nav     → Solo navegacion (requiere sim corriendo)
#   ./start.sh all     → Simulacion + Navegacion (recomendado)
#   ./start.sh stop    → Matar todo
#   ./start.sh status  → Ver estado
# =============================================================================

CONTAINER="mechdog_viz"
SOURCE="source /app/catkin_ws/devel/setup.bash"

# ─── Colores ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERR]${NC} $1"; }

# ─── Verificar que el contenedor esté corriendo ───────────────────────────────
check_container() {
    if ! docker ps --filter "name=${CONTAINER}" --filter "status=running" \
         --format "{{.Names}}" | grep -q "${CONTAINER}"; then
        warn "Contenedor no está corriendo. Iniciando..."
        docker compose up -d ${CONTAINER}
        sleep 5
    fi
    ok "Contenedor ${CONTAINER} activo"
}

# ─── Limpieza de procesos huérfanos ───────────────────────────────────────────
cleanup() {
    echo "Limpiando procesos anteriores..."
    docker compose exec -T ${CONTAINER} bash -c "
        killall -9 gzserver gzclient rosmaster roslaunch python3 2>/dev/null
        sleep 3
        killall -9 gzserver gzclient rosmaster 2>/dev/null
        sleep 1
        echo done
    " 2>/dev/null
    ok "Limpieza completada"
}

# ─── Lanzar simulacion ────────────────────────────────────────────────────────
start_sim() {
    echo "Iniciando simulacion Gazebo..."
    docker compose exec -d ${CONTAINER} bash -c "
        ${SOURCE}
        roslaunch mechdog_sim simulation.launch gui:=false rviz:=false \
            > /tmp/sim.log 2>&1
    "

    # Esperar /clock (indica que gzserver esta listo)
    echo -n "Esperando Gazebo"
    for i in $(seq 1 45); do
        sleep 2
        if docker compose exec -T ${CONTAINER} bash -c \
           "${SOURCE} && timeout 1 rostopic list 2>/dev/null | grep -q '^/clock$'" 2>/dev/null; then
            echo ""
            ok "Gazebo listo (${i}x2s)"
            break
        fi
        echo -n "."
        if [ $i -eq 45 ]; then
            echo ""
            err "Gazebo no respondio en 90s. Revisa: docker compose exec ${CONTAINER} bash -c 'cat /tmp/sim.log | tail -20'"
            exit 1
        fi
    done

    # Esperar spawn del robot
    echo -n "Esperando spawn del robot"
    for i in $(seq 1 15); do
        sleep 2
        if docker compose exec -T ${CONTAINER} bash -c \
           "${SOURCE} && timeout 2 rostopic echo /mechdog/odom -n 1 2>/dev/null | grep -q 'frame_id'" 2>/dev/null; then
            echo ""
            ok "Robot spawneado, odometria activa"
            return 0
        fi
        echo -n "."
    done
    echo ""
    warn "Odometria no detectada aun (puede tardar unos segundos mas)"
}

# ─── Lanzar navegacion ────────────────────────────────────────────────────────
start_nav() {
    echo "Iniciando stack de navegacion..."
    docker compose exec -d ${CONTAINER} bash -c "
        ${SOURCE}
        roslaunch mechdog_navigation navigation.launch environment:=simulation \
            > /tmp/nav.log 2>&1
    "

    # Esperar inicializacion de los 5 nodos
    echo -n "Esperando nodos de navegacion"
    for i in $(seq 1 20); do
        sleep 2
        COUNT=$(docker compose exec -T ${CONTAINER} bash -c \
            "${SOURCE} && rosnode list 2>/dev/null | grep -cE '(global_planner|local_planner|safe_learning|occupancy_grid|navigation_manager)'" 2>/dev/null)
        if [ "${COUNT}" -ge "5" ] 2>/dev/null; then
            echo ""
            ok "5 nodos de navegacion activos"
            return 0
        fi
        echo -n "."
    done
    echo ""
    warn "No todos los nodos de navegacion arrancaron. Revisa: cat /tmp/nav.log | tail -20"
}

# ─── Mostrar estado ───────────────────────────────────────────────────────────
show_status() {
    echo ""
    echo "═══════════════════════════════════════════════"
    echo "  ESTADO DEL SISTEMA MechDog"
    echo "═══════════════════════════════════════════════"

    # Nodos activos
    NODES=$(docker compose exec -T ${CONTAINER} bash -c \
        "${SOURCE} && rosnode list 2>/dev/null" 2>/dev/null)
    echo ""
    echo "Nodos ROS activos:"
    echo "${NODES}" | sed 's/^/  /' | grep -E '(gazebo|planner|safe|occupancy|manager|noise|sensor)' \
        && echo "" || echo "  (ninguno)"

    # Topics clave
    echo "Topics de datos:"
    for topic in /clock /mechdog/odom /mechdog/scan /mechdog/navigation_status; do
        if docker compose exec -T ${CONTAINER} bash -c \
           "${SOURCE} && timeout 2 rostopic echo ${topic} -n 1 2>/dev/null | grep -q '.'" 2>/dev/null; then
            echo -e "  ${GREEN}✓${NC} ${topic}"
        else
            echo -e "  ${RED}✗${NC} ${topic}"
        fi
    done

    echo ""
    echo "noVNC: http://localhost:6080/vnc.html"
    echo "═══════════════════════════════════════════════"
    echo ""
    echo "Para enviar un goal de navegacion:"
    echo "  docker compose exec ${CONTAINER} bash -c \\"
    echo "    \"source /app/catkin_ws/devel/setup.bash && \\"
    echo "     rostopic pub /mechdog/goal geometry_msgs/PoseStamped \\"
    echo "     '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \\"
    echo "     --once\""
    echo ""
}

# ─── Main ─────────────────────────────────────────────────────────────────────
case "${1:-all}" in

    sim)
        check_container
        cleanup
        start_sim
        show_status
        ;;

    nav)
        check_container
        start_nav
        show_status
        ;;

    all)
        echo ""
        echo "╔══════════════════════════════════════╗"
        echo "║   MechDog — Arranque Completo        ║"
        echo "╚══════════════════════════════════════╝"
        echo ""
        check_container
        cleanup
        start_sim
        start_nav
        show_status
        ;;

    stop)
        echo "Deteniendo todo..."
        cleanup
        ok "Sistema detenido"
        ;;

    status)
        check_container
        show_status
        ;;

    *)
        echo "Uso: $0 [all|sim|nav|stop|status]"
        echo ""
        echo "  all    → Simulacion + Navegacion (default)"
        echo "  sim    → Solo simulacion Gazebo"
        echo "  nav    → Solo navegacion (requiere sim)"
        echo "  stop   → Matar todo"
        echo "  status → Ver estado actual"
        exit 1
        ;;
esac
