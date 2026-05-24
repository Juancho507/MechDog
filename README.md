# MechDog — Robot Cuadrúpedo Autónomo

> ROS 1 Noetic · Gazebo Classic · Navegación Autónoma · Visualización Web noVNC

---

## Tabla de Contenidos

1. [Arquitectura del sistema](#1-arquitectura-del-sistema)
2. [Estructura del repositorio](#2-estructura-del-repositorio)
3. [Inicio rápido](#3-inicio-rápido)
4. [Salida esperada al correr el proyecto](#4-salida-esperada-al-correr-el-proyecto)
5. [Enviar un goal de navegación](#5-enviar-un-goal-de-navegación)
6. [Referencia de topics ROS](#6-referencia-de-topics-ros)
7. [Arquitectura de los paquetes](#7-arquitectura-de-los-paquetes)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│  Docker Container  (mechdog_viz)                                │
│                                                                 │
│  ┌────────────┐   ┌──────────────┐   ┌────────────────────┐   │
│  │  Gazebo    │   │  mechdog_sim │   │ mechdog_navigation │   │
│  │  Classic   │──▶│  (sensores)  │──▶│  (5 nodos A*/DWA)  │   │
│  │  (robot    │   │  noise inj.  │   │  safe learning     │   │
│  │  + LIDAR)  │   │  P3D odom    │   │  occupancy grid    │   │
│  └────────────┘   └──────────────┘   └────────────────────┘   │
│         │                                      │                │
│         ▼                                      ▼                │
│  /gazebo/lidar_clean              /mechdog/cmd_vel_raw          │
│  /gazebo/odom_clean               /mechdog/navigation_status    │
│         │                                                       │
│         ▼                                                       │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Xvfb :0 → x11vnc :5900 → websockify :6080              │  │
│  └──────────────────────────────────────────────────────────┘  │
└──────────────────────────┬──────────────────────────────────────┘
                           │ Puerto 6080
                    Browser: http://localhost:6080/vnc.html
```

### Flujo de datos de sensores

```
Gazebo (P3D plugin) ──▶ /gazebo/odom_clean ──▶ noise_injector ──▶ /mechdog/odom
Gazebo (LIDAR plugin) ─▶ /gazebo/lidar_clean ─▶ noise_injector ──▶ /mechdog/scan
```

### Docker Multi-Stage Build

| Stage | Base | Responsabilidad |
|-------|------|-----------------|
| `base` | `osrf/ros:noetic-desktop-full` | ROS Noetic + Gazebo + dependencias |
| `builder` | `base` | Compilación del workspace (`catkin_make`) |
| `visualizer` | `builder` | Stack noVNC (Xvfb + x11vnc + websockify) |

---

## 2. Estructura del Repositorio

```
MechDog/
├── Dockerfile                     # Multi-stage build (3 etapas)
├── docker-compose.yml             # Servicios Docker
├── start.sh                       # Script de arranque desde el host
├── start_vnc.sh                   # Entrypoint del contenedor (VNC)
├── catkin_ws/
│   ├── launch_all.sh              # Arranque completo dentro del contenedor
│   └── src/
│       ├── mechdog_description/   # URDF/Xacro del robot
│       │   └── urdf/mechdog.urdf.xacro
│       ├── mechdog_sim/           # Simulación y sensores
│       │   ├── scripts/
│       │   │   ├── noise_injector_node.py
│       │   │   ├── sensor_simulator_node.py
│       │   │   ├── metrics_collector_node.py
│       │   │   └── spawn_robot.py      ← borra modelo antes de spawn
│       │   ├── launch/
│       │   │   ├── simulation.launch
│       │   │   └── gazebo_world.launch
│       │   ├── config/
│       │   │   ├── simulation.yaml
│       │   │   ├── sensor_params.yaml
│       │   │   └── noise_injection.yaml
│       │   └── worlds/
│       │       └── open_path_world.world
│       └── mechdog_navigation/    # Stack de navegación autónoma
│           ├── scripts/
│           │   ├── global_planner_node.py    (A*)
│           │   ├── local_planner_node.py     (DWA)
│           │   ├── safe_learning_node.py     (frenado activo)
│           │   ├── occupancy_grid_node.py    (mapeo bayesiano)
│           │   └── navigation_manager_node.py (máquina de estados)
│           ├── launch/
│           │   └── navigation.launch
│           └── config/
│               ├── navigation.yaml
│               └── environments/
│                   ├── simulation.yaml
│                   └── real_hardware.yaml
└── docs/
```

---

## 3. Inicio Rápido

### Prerequisito: Docker Desktop instalado y corriendo

### Paso 1 — Construir la imagen (solo la primera vez)

```bash
docker compose build mechdog_viz
```

> Tarda ~5-8 minutos. Descarga ROS Noetic + Gazebo + dependencias.

### Paso 2 — Iniciar el contenedor

```bash
docker compose up -d mechdog_viz
```

### Paso 3 — Lanzar el sistema completo

```bash
docker compose exec mechdog_viz bash /app/catkin_ws/launch_all.sh
```

### Paso 4 — Abrir la interfaz gráfica

En tu navegador: **http://localhost:6080/vnc.html** → click **Connect**

Verás el escritorio virtual con Gazebo y RViz corriendo.

### Paso 5 — Enviar un goal de navegación

```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \
  --once
"
```

### Paso 6 — Parar todo

```bash
docker compose stop mechdog_viz
```

---

## 4. Salida Esperada al Correr el Proyecto

### Al ejecutar `launch_all.sh`

```
[1/5] Limpiando procesos anteriores...
      Listo
[2/5] Iniciando Gazebo (modo headless)...
[3/5] Esperando Gazebo...
      Gazebo listo (Xs)
[4/5] Esperando odometria del robot...
      Robot spawneado OK
[5/5] Iniciando stack de navegacion...

═══════════════════════════════════════════════
  Sistema listo. Nodos activos:
  /gazebo
  /global_planner
  /local_planner
  /navigation_manager
  /noise_injector
  /occupancy_grid_mapper
  /safe_learning
  /sensor_simulator
═══════════════════════════════════════════════
```

> El mensaje `navigation_startup_info process has died` es **normal e inofensivo** — es un publicador informativo de una sola vez cuya cadena con espacios falla en el parser de roslaunch.

### Lista completa de nodos activos (13 nodos)

```
/base_to_laser          ← TF estática base_link → lidar_link
/gazebo                 ← Servidor de simulación Gazebo
/gazebo_gui             ← GUI de Gazebo (solo con gui:=true)
/global_planner         ← Planificador global A*
/joint_state_publisher  ← Publica estados de joints del URDF
/local_planner          ← Planificador local DWA (20 Hz)
/metrics_collector      ← Recolecta métricas de simulación
/navigation_manager     ← Máquina de estados (idle/planning/moving)
/noise_injector         ← Inyecta ruido realista a sensores
/occupancy_grid_mapper  ← Mapa de ocupación bayesiano 1000×1000
/robot_state_publisher  ← Publica TF del URDF (50 Hz)
/rosout                 ← Logger de ROS
/safe_learning          ← Control de seguridad activo (50 Hz)
/sensor_simulator       ← Monitor de salud de sensores
```

---

## 5. Enviar un Goal de Navegación

### Comando

```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \
  --once
"
```

### Salida esperada del comando

```
publishing and latching message for 3.0 seconds
```

Eso es todo lo que imprime el comando — es correcto. El trabajo real ocurre en los nodos de navegación.

### Qué verificar después de enviar el goal

**1. Estado del Navigation Manager** (debe decir `"moving"`):
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/navigation_status -n 1
"
# Salida esperada:
# data: "moving"
```

**2. Plan global generado (A\*)**:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/global_plan -n 1 | head -8
"
# Salida esperada (fragmento):
# header:
#   frame_id: "map"
# poses:
#   - (60 waypoints aprox.)
```

**3. Velocidades del Local Planner (DWA)**:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/cmd_vel_raw -n 1
"
# Salida esperada:
# linear:
#   x: 0.2-0.5  (velocidad hacia adelante en m/s)
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: -0.3 a 0.3  (giro en rad/s)
```

**4. Estado de seguridad (Safe Learning)**:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/safety_status -n 1
"
# Salida esperada (robot libre de obstáculos):
# data: "safe"
```

**5. En el log de navegación** (`/tmp/nav.log` dentro del contenedor):
```
[INFO]: Received global path with 60 waypoints
[INFO]: Path planned in 0.000s, length: 60 waypoints
[INFO]: Received global path with 60 waypoints
...
```

### Secuencia completa de verificación

```bash
# Abrir una sola terminal y monitorear todo:
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/navigation_status &
  rostopic hz /mechdog/cmd_vel_raw &
  wait
"
# Salida esperada:
# data: "moving"
# ---
# subscribed to [/mechdog/cmd_vel_raw]
# average rate: 20.000 Hz
```

---

## 6. Referencia de Topics ROS

### Topics de Entrada (el robot recibe)

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/mechdog/goal` | `geometry_msgs/PoseStamped` | Goal de navegación |
| `/mechdog/emergency_stop` | `std_msgs/Bool` | Parada de emergencia manual |

### Topics de Salida (el robot publica)

| Topic | Tipo | Hz | Descripción |
|-------|------|----|-------------|
| `/mechdog/odom` | `nav_msgs/Odometry` | 50 | Odometría con ruido (frame: `odom`) |
| `/mechdog/scan` | `sensor_msgs/LaserScan` | 20 | LIDAR 360° con ruido |
| `/mechdog/cmd_vel_raw` | `geometry_msgs/Twist` | 20 | Velocidad calculada por DWA |
| `/mechdog/navigation_status` | `std_msgs/String` | 1 | `idle` / `moving` / `replanning` |
| `/mechdog/safety_status` | `std_msgs/String` | 50 | `safe` / `warning` / `emergency_stop` |
| `/mechdog/global_plan` | `nav_msgs/Path` | 1 | Path global del A* |
| `/mechdog/local_plan` | `nav_msgs/Path` | 20 | Path local del DWA |
| `/mechdog/map` | `nav_msgs/OccupancyGrid` | 1 | Mapa de ocupación bayesiano |
| `/mechdog/safety_polygon` | `geometry_msgs/PolygonStamped` | 50 | Polígono de seguridad dinámico |

### Topics internos de Gazebo

| Topic | Descripción |
|-------|-------------|
| `/gazebo/odom_clean` | Odometría ground-truth del plugin P3D |
| `/gazebo/lidar_clean` | LIDAR sin ruido del plugin gazebo_ros_laser |
| `/clock` | Tiempo de simulación (use_sim_time=true) |

---

## 7. Arquitectura de los Paquetes

### mechdog_description

Modelo físico del robot en formato URDF/Xacro.

- **Cuerpo**: Box 0.5×0.3×0.2m, masa 15kg
- **Patas**: 4 cilindros, masa 0.5kg cada uno (joints fijos)
- **LIDAR**: Cilindro en `x=0.28m, z=0.15m` desde base_link
  - Tipo: ray, 360°, 0.1-10m, 20Hz
  - Plugin: `libgazebo_ros_laser.so` → `/gazebo/lidar_clean`
- **Odometría**: Plugin `libgazebo_ros_p3d.so` → `/gazebo/odom_clean`

### mechdog_sim

Capa de simulación que conecta Gazebo con el stack de navegación.

| Nodo | Función |
|------|---------|
| `spawn_robot.py` | Borra modelo anterior y hace spawn limpio en Gazebo |
| `noise_injector_node.py` | Agrega ruido gaussiano + drift a odom y LIDAR (Sim-to-Real) |
| `sensor_simulator_node.py` | Monitorea salud de sensores, logs timeouts |
| `metrics_collector_node.py` | Guarda métricas en `/app/metrics_output/` |

**Parámetros de ruido** (`noise_injection.yaml`):
- LIDAR: σ=0.01m, 1% outliers, 1% dropouts
- Odometría: σ=0.005m, drift 0.001m/s, σ_θ=0.01rad

### mechdog_navigation

Stack de navegación autónoma 100% portable (sin dependencias de Gazebo).

```
/mechdog/goal ──▶ navigation_manager ──▶ global_planner (A*)
                                    ├──▶ local_planner (DWA)
                                    ├──▶ safe_learning (freno activo)
                                    └──▶ occupancy_grid_mapper
```

| Nodo | Algoritmo | Frecuencia |
|------|-----------|------------|
| `global_planner_node.py` | A* en grid 2D | 1 Hz (o al recibir goal) |
| `local_planner_node.py` | DWA (Dynamic Window Approach) | 20 Hz |
| `safe_learning_node.py` | Freno predictivo + polígono dinámico | 50 Hz |
| `occupancy_grid_node.py` | Bayesian log-odds update | Por scan LIDAR |
| `navigation_manager_node.py` | Máquina de estados FSM | Reactivo |

**Fórmula de frenado (Safe Learning)**:
```
d_freno = v² / (2×a) + v × t_reacción   × factor_seguridad
```

---

## 8. Troubleshooting

### Error: `entity already exists` o `Address already in use`

**Causa**: Hay una sesión de Gazebo corriendo de una ejecución anterior.

**Solución**:
```bash
# Opción A (recomendada): reiniciar el contenedor
docker compose stop mechdog_viz
docker compose up -d mechdog_viz
docker compose exec mechdog_viz bash /app/catkin_ws/launch_all.sh

# Opción B: limpiar procesos sin reiniciar
docker compose exec mechdog_viz bash -c "killall -9 gzserver gzclient rosmaster roslaunch python3 2>/dev/null; sleep 3"
```

### Error: `Waiting for odometry...` (timeout)

**Causa**: El nodo `noise_injector` no recibe datos de `/gazebo/odom_clean`.

**Diagnóstico**:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic hz /gazebo/odom_clean
"
```

**Solución**: El plugin P3D de Gazebo tarda ~10s en activarse tras el spawn. Espera un poco más. Si persiste, verifica que el robot fue spawneado:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rosservice call /gazebo/get_model_state '{model_name: mechdog}'
"
```

### Error: `navigation_startup_info process has died`

**Causa**: Error de parsing de espacios en `rostopic pub` dentro del launch file.

**Impacto**: Ninguno. La navegación funciona con normalidad.

### noVNC: `Failed to connect to server`

**Causa**: Docker Desktop en Windows descarta el mapeo de puertos con `network_mode: host`.

**Solución**: Ya corregido en `docker-compose.yml` — `mechdog_viz` usa bridge networking con puertos explícitos.

**Verificación**:
```bash
docker ps --filter "name=mechdog_viz" --format "{{.Ports}}"
# Debe mostrar: 0.0.0.0:6080->6080/tcp
```

### La navegación dice `idle` después del goal

**Causa**: El topic `/mechdog/odom` no tiene datos — el robot no sabe dónde está.

**Verificación rápida**:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  timeout 2 rostopic echo /mechdog/odom -n 1 | head -4
"
```

Si no hay salida, reiniciar la simulación con `launch_all.sh`.

### Comandos de diagnóstico útiles

```bash
# Ver todos los nodos activos
docker compose exec mechdog_viz bash -c "source /app/catkin_ws/devel/setup.bash && rosnode list"

# Verificar frecuencia de sensores
docker compose exec mechdog_viz bash -c "source /app/catkin_ws/devel/setup.bash && rostopic hz /mechdog/odom /mechdog/scan"

# Ver logs de la simulación en tiempo real
docker compose exec mechdog_viz bash -c "tail -f /tmp/sim.log"

# Ver logs de navegación en tiempo real
docker compose exec mechdog_viz bash -c "tail -f /tmp/nav.log"

# Parar el robot inmediatamente
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/emergency_stop std_msgs/Bool 'data: true' --once
"
```

---

## Licencia

Proyecto universitario de investigación en robótica autónoma.
