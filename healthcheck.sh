#!/bin/bash
# =============================================================================
# MechDog — Script de Validación de Despliegue (Healthcheck)
# Verifica que la infraestructura bridge + ROS esté operativa.
#
# Uso:
#   ./healthcheck.sh               → Validación completa
#   ./healthcheck.sh --quick       → Solo contenedores y roscore
#   ./healthcheck.sh --service <s> → Validar un servicio específico
# =============================================================================

set -e

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
PASS="${GREEN}✓ PASS${NC}"
FAIL="${RED}✗ FAIL${NC}"
WARN="${YELLOW}⚠ WARN${NC}"
INFO="${CYAN}ℹ${NC}"

errors=0
warnings=0

_report() {
  local status=$1; shift
  echo -e "  ${status} $*"
}

# ─── Verificaciones ──────────────────────────────────────────────────────────

check_all_containers() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: Contenedores                   ║"
  echo "╚═══════════════════════════════════════════════╝"
  local required=(roscore simulation navigation mechdog_viz)
  for svc in "${required[@]}"; do
    local cname="mechdog-${svc}-1"
    if docker ps --format "{{.Names}}" 2>/dev/null | grep -q "$cname"; then
      _report "$PASS" "$svc ($cname) — running"
    else
      _report "$FAIL" "$svc ($cname) — NOT running"
      ((errors++))
    fi
  done
}

check_network() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: Red Bridge (mechdog_net)       ║"
  echo "╚═══════════════════════════════════════════════╝"
  local net_name="mechdog_net"
  if docker network ls --format "{{.Name}}" 2>/dev/null | grep -q "$net_name"; then
    _report "$PASS" "Red $net_name existe"
  else
    _report "$FAIL" "Red $net_name NO existe"
    ((errors++))
    return
  fi

  # Verificar que los contenedores están en la red
  for svc in roscore simulation navigation mechdog_viz; do
    local cname="mechdog-${svc}-1"
    if docker inspect "$cname" --format '{{.NetworkSettings.Networks.mechdog_net.IPAddress}}' 2>/dev/null | grep -q '^[0-9]'; then
      _report "$PASS" "$svc conectado a $net_name"
    else
      _report "$FAIL" "$svc NO conectado a $net_name"
      ((errors++))
    fi
  done

  # Verificar resolución DNS entre contenedores
  echo ""
  echo "  ${INFO} Verificando resolución DNS bridge..."
  local resolver_ok=true
  for target in roscore simulation navigation mechdog_viz; do
    if docker compose exec -T roscore bash -c \
       "getent hosts $target >/dev/null 2>&1" 2>/dev/null; then
      _report "$PASS" "roscore → $target (DNS OK)"
    else
      _report "$FAIL" "roscore → $target (DNS FAIL)"
      resolver_ok=false
      ((errors++))
    fi
  done

  if $resolver_ok; then
    echo ""
    _report "$PASS" "Todos los contenedores se resuelven por DNS bridge"
  fi
}

check_roscore() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: ROS Master (roscore)           ║"
  echo "╚═══════════════════════════════════════════════╝"
  if docker compose exec -T roscore bash -c \
     "source /opt/ros/noetic/setup.bash && timeout 5 rostopic list >/dev/null 2>&1" 2>/dev/null; then
    _report "$PASS" "roscore accesible (http://roscore:11311)"
  else
    _report "$FAIL" "roscore NO responde"
    ((errors++))
    return
  fi

  # Verificar ROS param server
  if docker compose exec -T roscore bash -c \
     "source /opt/ros/noetic/setup.bash && timeout 3 rosparam list >/dev/null 2>&1" 2>/dev/null; then
    _report "$PASS" "ROS Parameter Server activo"
  else
    _report "$WARN" "ROS Parameter Server no responde"
    ((warnings++))
  fi
}

check_nodes() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: Nodos ROS                      ║"
  echo "╚═══════════════════════════════════════════════╝"
  local nodes
  nodes=$(docker compose exec -T roscore bash -c \
    "source /opt/ros/noetic/setup.bash && timeout 5 rosnode list 2>/dev/null" 2>/dev/null) || true
  if [ -z "$nodes" ]; then
    _report "$WARN" "No se pudieron listar nodos (ROS puede estar iniciando)"
    ((warnings++))
    return
  fi

  local count=$(echo "$nodes" | wc -l)
  _report "$PASS" "${count} nodos ROS registrados"

  # Verificar nodos esperados
  for name in gazebo global_planner local_planner safe_learning occupancy_grid_mapper; do
    if echo "$nodes" | grep -q "$name"; then
      _report "$PASS" "Nodo /${name} — registrado"
    else
      _report "$WARN" "Nodo /${name} — NO registrado"
      ((warnings++))
    fi
  done
}

check_topics() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: Topics ROS                     ║"
  echo "╚═══════════════════════════════════════════════╝"
  local critical_topics=(
    "/clock"
    "/mechdog/odom"
    "/mechdog/ultrasonic"
    "/mechdog/navigation_status"
  )
  local optional_topics=(
    "/mechdog/scan"
    "/mechdog/global_plan"
    "/mechdog/local_plan"
    "/mechdog/map"
    "/mechdog/safety_status"
  )

  for topic in "${critical_topics[@]}"; do
    if docker compose exec -T roscore bash -c \
       "source /opt/ros/noetic/setup.bash && timeout 3 rostopic info $topic 2>/dev/null | grep -q 'Publishers'" 2>/dev/null; then
      _report "$PASS" "$topic — publishers encontrados"
    else
      _report "$FAIL" "$topic — SIN publishers"
      ((errors++))
    fi
  done

  for topic in "${optional_topics[@]}"; do
    if docker compose exec -T roscore bash -c \
       "source /opt/ros/noetic/setup.bash && timeout 2 rostopic info $topic 2>/dev/null | grep -q 'Publishers'" 2>/dev/null; then
      _report "$PASS" "$topic — publishers encontrados"
    else
      _report "$WARN" "$topic — ausente (no crítico)"
      ((warnings++))
    fi
  done
}

check_cross_container() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: Comunicación Entre Contenedores║"
  echo "╚═══════════════════════════════════════════════╝"

  # publicar y leer desde distintos contenedores
  local test_topic="/mechdog_healthcheck_ping"
  echo "  ${INFO} Publicando en ${test_topic} desde simulation..."
  docker compose exec -T simulation bash -c \
    "source /opt/ros/noetic/setup.bash && \
     rostopic pub ${test_topic} std_msgs/String 'data: ping' --once" 2>/dev/null && \
  _report "$PASS" "simulation publica en roscore"

  echo "  ${INFO} Leyendo ${test_topic} desde navigation..."
  if docker compose exec -T navigation bash -c \
     "source /opt/ros/noetic/setup.bash && timeout 3 rostopic echo ${test_topic} -n 1 2>/dev/null | grep -q 'ping'" 2>/dev/null; then
    _report "$PASS" "navigation recibe topics desde simulation"
  else
    _report "$FAIL" "navigation NO recibe topics (pérdida de comunicación bridge)"
    ((errors++))
  fi

  # Limpiar
  docker compose exec -T roscore bash -c \
    "source /opt/ros/noetic/setup.bash && rostopic pub ${test_topic} std_msgs/String 'data: done' --once" 2>/dev/null || true
}

check_mechdog_viz() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   HEALTHCHECK: mechdog_viz (noVNC + Render)   ║"
  echo "╚═══════════════════════════════════════════════╝"

  # Xvfb
  if docker compose exec -T mechdog_viz bash -c \
     "ps aux | grep -v grep | grep -q Xvfb" 2>/dev/null; then
    _report "$PASS" "Xvfb corriendo (framebuffer virtual)"
  else
    _report "$FAIL" "Xvfb NO está corriendo"
    ((errors++))
  fi

  # x11vnc
  if docker compose exec -T mechdog_viz bash -c \
     "ps aux | grep -v grep | grep -q x11vnc" 2>/dev/null; then
    _report "$PASS" "x11vnc corriendo (servidor VNC)"
  else
    _report "$FAIL" "x11vnc NO está corriendo"
    ((errors++))
  fi

  # websockify (noVNC)
  if docker compose exec -T mechdog_viz bash -c \
     "ps aux | grep -v grep | grep -q websockify" 2>/dev/null; then
    _report "$PASS" "websockify corriendo (noVNC en puerto 6080)"
  else
    _report "$FAIL" "websockify NO está corriendo"
    ((errors++))
  fi

  # Variables de software rendering
  for var in QT_X11_NO_MITSHM LIBGL_ALWAYS_SOFTWARE GAZEBO_HEADLESS; do
    if docker compose exec -T mechdog_viz bash -c \
       "printenv $var 2>/dev/null | grep -q '1'" 2>/dev/null; then
      _report "$PASS" "${var}=1 (software rendering)"
    else
      _report "$FAIL" "${var} NO está configurada"
      ((errors++))
    fi
  done

  # Probar conexión noVNC desde el host
  if command -v curl &>/dev/null; then
    if curl -s http://localhost:6080/vnc.html 2>/dev/null | grep -q 'noVNC'; then
      _report "$PASS" "noVNC responde en http://localhost:6080/vnc.html"
    else
      _report "$WARN" "noVNC no responde (puede estar arrancando)"
      ((warnings++))
    fi
  fi
}

# ─── Resumen ──────────────────────────────────────────────────────────────────

print_summary() {
  echo ""
  echo "╔═══════════════════════════════════════════════╗"
  echo "║   RESUMEN DE VALIDACIÓN                       ║"
  echo "╚═══════════════════════════════════════════════╝"
  if [ $errors -eq 0 ] && [ $warnings -eq 0 ]; then
    echo -e "  ${GREEN}Todas las verificaciones pasaron${NC}"
    echo ""
    echo "  Bridge network:   OK (mechdog_net)"
    echo "  ROS Master:       OK (http://roscore:11311)"
    echo "  Servicios:        4/4 activos"
    echo "  Render:           software (sin GPU)"
    echo "  noVNC:            http://localhost:6080/vnc.html"
    return 0
  else
    echo -e "  ${RED}${errors} errores, ${warnings} advertencias${NC}"
    return 1
  fi
}

# ─── Main ─────────────────────────────────────────────────────────────────────

case "${1:-all}" in
  --quick)
    check_all_containers
    check_network
    check_roscore
    print_summary
    ;;
  --service)
    case "${2:-}" in
      roscore) check_roscore ;;
      simulation) check_cross_container ;;
      navigation) check_nodes ;;
      mechdog_viz) check_mechdog_viz ;;
      *)
        echo "Servicios: roscore, simulation, navigation, mechdog_viz"
        exit 1
        ;;
    esac
    ;;
  all|"")
    check_all_containers
    check_network
    check_roscore
    check_nodes
    check_topics
    check_cross_container
    check_mechdog_viz
    print_summary
    ;;
  *)
    echo "Uso: $0 [--quick|--service <s>|all]"
    exit 1
    ;;
esac
