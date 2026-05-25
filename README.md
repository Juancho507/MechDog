# MechDog — Robot Cuadrúpedo Autónomo

> ROS 1 Noetic · Gazebo Classic · Navegación Autónoma · noVNC · Experimentación

---

## Tabla de Contenidos

1. [Arquitectura del sistema](#1-arquitectura-del-sistema)
2. [Arquitectura Docker](#2-arquitectura-docker)
3. [Estructura del repositorio](#3-estructura-del-repositorio)
4. [Inicio rápido](#4-inicio-rápido)
   - [Opción A: Multi-servicio (docker compose)](#opción-a-multi-servicio-recomendada)
   - [Opción B: Script todo-en-uno (start.sh)](#opción-b-script-todo-en-uno-startsh)
5. [Salida esperada](#5-salida-esperada-al-correr-el-proyecto)
6. [Enviar un goal de navegación](#6-enviar-un-goal-de-navegación)
7. [Visualización y Simulación del Entorno](#7-visualización-y-simulación-del-entorno)
8. [Experimentación](#8-experimentación)
9. [Benchmarking Algorítmico](#9-benchmarking-algorítmico)
10. [Referencia de topics ROS](#10-referencia-de-topics-ros)
11. [Arquitectura de los paquetes](#11-arquitectura-de-los-paquetes)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Arquitectura del Sistema

```
┌──────────────────────────────────────────────────────────────────┐
│                        DOCKER HOST                               │
│                                                                  │
│  ┌──────────┐   ┌────────────┐   ┌──────────────┐               │
│  │ roscore  │   │ simulation │   │ navigation   │               │
│  │ :11311   │◀──│ (Gazebo)   │──▶│ (5 nodos)    │               │
│  │ ROS      │   │ mechdog_sim│   │ mechdog_nav  │               │
│  │ Master   │   │ noise,odom │   │ A*,DWA,safe  │               │
│  └────┬─────┘   └─────┬──────┘   └──────┬───────┘               │
│       │               │                  │                       │
│       │               ▼                  ▼                       │
│       │        ┌──────────────────────────────┐                  │
│       │        │      mechdog_viz (noVNC)     │                  │
│       │        │  Xvfb → x11vnc → websockify  │                  │
│       │        │  Gazebo GUI + RViz visual    │                  │
│       │        └──────────┬───────────────────┘                  │
│       │                   │ Puerto 6080                          │
│       └───────────────────┼──────────────────────────────────────┘
│                           ▼                                       │
│                   Browser: localhost:6080/vnc.html                │
└──────────────────────────────────────────────────────────────────┘
```

### Flujo de datos de sensores

```
Gazebo (P3D plugin) ────▶ /gazebo/odom_clean ──▶ noise_injector ───▶ /mechdog/odom
Gazebo (Range HC-SR04) ─▶ /gazebo/ultrasonic_clean ─▶ noise_injector ──▶ /mechdog/ultrasonic (Range)
                                                              └──▶ /mechdog/scan (1-ray LaserScan)

Pipeline real (HAL):
  I2C Sonar (HC-SR04) ──▶ mechdog_hardware_interface ──▶ /mechdog/ultrasonic
                                                    └──▶ /mechdog/scan (1-ray LaserScan)
```

### Docker Multi-Stage Build

| Stage | Base | Responsabilidad |
|-------|------|-----------------|
| `base` | `osrf/ros:noetic-desktop-full` | ROS Noetic + Gazebo + dependencias |
| `builder` | `base` | Compilación del workspace (`catkin_make`) |
| `visualizer` | `builder` | Stack noVNC (Xvfb + x11vnc + websockify) |

---

## 2. Arquitectura Docker

El sistema se compone de **4 servicios principales** definidos en `docker-compose.yml`:

| Servicio | Imagen (target) | Puerto | Rol | ¿Debe quedar abierto? |
|----------|----------------|--------|-----|-----------------------|
| `roscore` | `builder` | — (host) | ROS Master (`roscore`) | **Sí** — debe estar corriendo siempre |
| `simulation` | `builder` | — (host) | Gazebo + sensores + ruido | **Sí** — debe estar corriendo siempre |
| `navigation` | `builder` | — (host) | Stack navegación (5 nodos) | **Sí** — debe estar corriendo siempre |
| `mechdog_viz` | `visualizer` | `6080` | noVNC + Gazebo GUI + RViz | Opcional — solo para ver la simulación |

Además hay servicios **legacy** (perfil `--profile legacy`) para la arquitectura anterior con PyBullet:
- `solver`, `visualize`, `train` — requieren `Dockerfile.pybullet` (ya no existe)

### Dependencia entre servicios

```
roscore ◀── simulation ◀── navigation
                ▲
          mechdog_viz (visualización web)
```

Los servicios `simulation` y `navigation` dependen de `roscore` y se conectan via `network_mode: host`.

### ¿Qué contenedores deben estar abiertos?

- **Mínimo para que funcione**: `roscore` + `simulation` + `navigation`
- **Para ver la simulación**: agregar `mechdog_viz`
- **Todos deben mantenerse corriendo** — si alguno muere, el sistema deja de funcionar

---

## 3. Estructura del Repositorio

```
MechDog/
├── Dockerfile                     # Multi-stage build (base → builder → visualizer)
├── docker-compose.yml             # 4 servicios ROS + legacy PyBullet
├── start.sh                       # Script de arranque desde el host (all/sim/nav/stop/status)
├── start_vnc.sh                   # Entrypoint del contenedor visualizer (Xvfb + x11vnc + noVNC)
├── requirements.txt               # Solo para referencia legacy (PyBullet)
├── catkin_ws/
│   ├── launch_all.sh              # Arranque todo-en-uno dentro del contenedor
│   ├── metrics_output/            # Métricas generadas por experimentos
│   └── src/
│       ├── mechdog_description/   # URDF/Xacro del robot
│       │   ├── urdf/
│       │   │   ├── mechdog.urdf.xacro       # Modelo físico (SIM + REAL)
│       │   │   └── mechdog_gazebo.xacro     # Plugins Gazebo (P3D, LIDAR)
│       │   ├── launch/display.launch
│       │   ├── config/joint_limits.yaml
│       │   └── rviz/
│       │       ├── display.rviz
│       │       └── display.rviz
│       ├── mechdog_sim/           # Simulación Gazebo + sensores
│       │   ├── scripts/
│       │   │   ├── noise_injector_node.py
│       │   │   ├── sensor_simulator_node.py
│       │   │   ├── metrics_collector_node.py
│       │   │   └── spawn_robot.py
│       │   ├── launch/
│       │   │   ├── simulation.launch
│       │   │   └── gazebo_world.launch
│       │   ├── config/
│       │   │   ├── simulation.yaml
│       │   │   ├── sensor_params.yaml
│       │   │   └── noise_injection.yaml
│       │   └── worlds/
│       │       ├── open_path.world
│       │       └── open_path_world.world
│       ├── mechdog_navigation/    # Stack de navegación (100% portable)
│       │   ├── scripts/
│       │   │   ├── global_planner_node.py    (A*)
│       │   │   ├── local_planner_node.py     (DWA)
│       │   │   ├── safe_learning_node.py     (frenado predictivo)
│       │   │   ├── occupancy_grid_node.py    (mapeo bayesiano)
│       │   │   └── navigation_manager_node.py (máquina de estados)
│       │   ├── src/mechdog_navigation/__init__.py
│       │   ├── setup.py
│       │   ├── launch/
│       │   │   ├── navigation.launch
│       │   │   ├── planning.launch
│       │   │   ├── mapping.launch
│       │   │   └── safe_learning.launch
│       │   └── config/
│       │       ├── navigation.yaml
│       │       ├── global_planner.yaml
│       │       ├── local_planner.yaml
│       │       ├── occupancy_grid.yaml
│       │       ├── safe_learning.yaml
│       │       └── environments/
│       │           ├── simulation.yaml
│       │           └── real_hardware.yaml
│       └── mechdog_experiments/   # Framework de experimentación
│           ├── scripts/
│           │   ├── experiment_runner.py
│           │   ├── scenario_builder.py
│           │   ├── home_base_node.py
│           │   ├── metrics_aggregator.py
│           │   └── report_generator.py
│           ├── launch/
│           │   ├── experiments.launch
│           │   ├── run_experiments.launch
│           │   ├── build_scenarios.launch
│           │   └── generate_report.launch
│           ├── config/
│           │   ├── experiment_config.yaml
│           │   ├── scenarios.yaml
│           │   └── home_base.yaml
│           ├── worlds/
│           │   ├── scenario_simple.world
│           │   ├── scenario_medium.world
│           │   └── scenario_complex.world
│           └── report/
│               ├── index.html
│               └── assets/
│                   ├── charts.js
│                   └── style.css
└── docs/
    ├── 00_PROJECT_STRUCTURE.md
    ├── 01_ARCHITECTURE_ROS_RULES.md
    ├── 02_PROJECT_WORKFLOW.md
    ├── 03_SIM_TO_REAL_STRATEGY.md
    └── AI_CONTEXT.md
```

---

## 4. Inicio Rápido

### Prerequisito: Docker Desktop instalado y corriendo

### Opción A: Multi-servicio (recomendada)

Inicia cada servicio por separado. Todos deben quedar corriendo.

#### Paso 1 — Construir la imagen

```bash
docker compose build mechdog_viz
```

> Tarda ~5-8 minutos. Descarga ROS Noetic + Gazebo + dependencias y compila el workspace.

#### Paso 2 — Iniciar el ROS Master

```bash
docker compose up -d roscore
```

#### Paso 3 — Iniciar la simulación (Gazebo)

```bash
docker compose up -d simulation
```

#### Paso 4 — Iniciar la navegación

```bash
docker compose up -d navigation
```

#### Paso 5 — (Opcional) Abrir la interfaz gráfica

```bash
docker compose up -d mechdog_viz
```

En tu navegador: **http://localhost:6080/vnc.html** → click **Connect**

Verás el escritorio virtual con Gazego y RViz.

#### Paso 6 — Verificar que todo funciona

```bash
docker compose exec roscore bash -c "source /app/catkin_ws/devel/setup.bash && rosnode list"
```

Debes ver ~13 nodos activos.

#### Paso 7 — Enviar un goal de navegación

```bash
docker compose exec roscore bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \
  --once
"
```

#### Paso 8 — Parar todo

```bash
docker compose stop roscore simulation navigation mechdog_viz
```

---

### Opción B: Script todo-en-uno (start.sh)

Usa el contenedor `mechdog_viz` para correr todo internamente (Gazebo + navegación en un solo contenedor).

```bash
# Construir (solo la primera vez)
docker compose build mechdog_viz

# Arrancar todo (simulación + navegación + VNC)
./start.sh all

# O pasos individuales:
./start.sh sim     # Solo simulación
./start.sh nav     # Solo navegación
./start.sh status  # Ver estado del sistema
./start.sh stop    # Detener todo
```

Luego abre **http://localhost:6080/vnc.html** en tu navegador.

> `start.sh` automáticamente levanta el contenedor `mechdog_viz`, limpia procesos anteriores, espera a Gazebo, espera la odometría del robot y lanza la navegación.

---

## 5. Salida Esperada al Correr el Proyecto

### Al ejecutar `./start.sh all`

```
╔══════════════════════════════════════╗
║   MechDog — Arranque Completo        ║
╚══════════════════════════════════════╝

[OK] Contenedor mechdog_viz activo
Limpiando procesos anteriores...
[OK] Limpieza completada
Iniciando simulacion Gazebo...
Esperando Gazebo.....      Gazebo listo (5x2s)
Esperando spawn del robot.....      Robot spawneado, odometria activa
Iniciando stack de navegacion...
Esperando nodos de navegacion......      [OK] 5 nodos de navegacion activos
```

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

## 6. Enviar un Goal de Navegación

### Con el script start.sh

```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \
  --once
"
```

### Con servicios separados

```bash
docker compose exec roscore bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' \
  --once
"
```

### Salida esperada

```
publishing and latching message for 3.0 seconds
```

### Verificar estado después del goal

```bash
# Estado del navigation manager (debe decir "moving")
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/navigation_status -n 1
"

# Velocidades del DWA
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/cmd_vel_raw -n 1
"

# Estado de seguridad
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/safety_status -n 1
"
```

---

## 7. Visualización y Simulación del Entorno

Esta sección detalla cómo lanzar la simulación, abrir la interfaz web noVNC, cargar el mapa probabilístico y observar en tiempo real cómo el robot ejecuta el *Scan Behavior* y traza rutas con distintos algoritmos.

### 7.1 Lanzar simulación completa

```bash
# Construir imagen (solo la primera vez)
docker compose build mechdog_viz

# Iniciar todo (Gazebo + navegación + noVNC)
./start.sh all
```

O con servicios separados:

```bash
docker compose up -d roscore
docker compose up -d simulation
docker compose up -d navigation
docker compose up -d mechdog_viz
```

### 7.2 Acceder a la interfaz web noVNC

En el navegador: **http://localhost:6080/vnc.html** → click **Connect**

Verás un escritorio virtual Linux con:
- **Gazebo GUI**: Mundo de simulación con el robot MechDog
- **RViz**: Visualización del mapa de ocupación, rutas y planificación

### 7.3 Cargar el mapa probabilístico

En RViz (ya configurado en el launch):
- Add → **Map** → Topic: `/mechdog/map`
- Add → **LaserScan** → Topic: `/mechdog/scan`
- Add → **Path** → Topic: `/mechdog/global_plan`
- Add → **Path** → Topic: `/mechdog/local_plan`

Todos estos temas se cargan automáticamente al abrir el archivo RViz configurado.

### 7.4 Observar el Scan Behavior en tiempo real

El *Scan Behavior* se activa automáticamente cuando el planificador global no encuentra una ruta hacia el goal (debido al FOV de 15° del sensor ultrasónico).

Para provocar y observar el Scan Behavior:

```bash
# 1. Enviar un goal detrás de un obstáculo (fuera del FOV inicial)
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}' \
  --once
"

# 2. Monitorear el estado del scan behavior
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/navigation_status
"
```

El robot se detendrá, rotará 360° a 0.3 rad/s (~21 segundos), poblando el mapa probabilístico durante la rotación. Una vez completado, el planificador global recalculará la ruta, y el robot navegará hacia el goal.

### 7.5 Monitorear el mapa en tiempo real

```bash
# Ver el mapa de ocupación actual
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/map -n 1 --noarr | grep -E 'width:|height:|resolution:'
"

# Ver la frecuencia de actualización del mapa
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic hz /mechdog/map
"

# Ver las rutas global y local
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic echo /mechdog/global_plan -n 1 | grep -c 'poses:'
"
```

### 7.6 Cambiar algoritmo de planificación global

```bash
# A* (default)
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rosparam set /global_planner/algorithm astar
"

# Dijkstra
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rosparam set /global_planner/algorithm dijkstra
"

# BFS
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rosparam set /global_planner/algorithm bfs
"
```

---

## 8. Experimentación

El paquete `mechdog_experiments` permite ejecutar baterías de pruebas automatizadas para evaluar el stack de navegación.

### Escenarios disponibles

| Escenario | Obstáculos | Descripción |
|-----------|------------|-------------|
| `simple` | 3 | Curso básico, obstáculos dispersos |
| `medium` | 8 | Curso medio, pasillos estrechos |
| `complex` | 15 | Curso denso, maniobras evasivas |

### Ejecutar experimentos

```bash
# Con start.sh (todo en un contenedor)
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  roslaunch mechdog_experiments run_experiments.launch
"
```

### Generar reporte

```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  roslaunch mechdog_experiments generate_report.launch
"
```

Los reportes se generan en `catkin_ws/src/mechdog_experiments/report/`.

---

## 9. Benchmarking Algorítmico

El proyecto incluye un motor de benchmarking automatizado (`experiment_runner.py`) para comparar los tres algoritmos de planificación global (A*, Dijkstra, BFS) bajo dos escenarios.

### 9.1 Escenarios de benchmark

| Escenario | Descripción |
|-----------|-------------|
| **A (Static)** | Cálculo inmediato de ruta sin rotación previa |
| **B (Active)** | Simula el *Scan Behavior* (barrido 360° de 2m de radio) antes del cálculo |

### 9.2 Ejecutar benchmark

```bash
# Benchmark con mapa sintético (100×100, obstáculos + pasillo)
python experiment_runner.py

# Benchmark con mapa en vivo desde ROS
python experiment_runner.py --live
```

### 9.3 Resultados esperados

El script ejecuta cada algoritmo en 7 pruebas de navegación para ambos escenarios y exporta los resultados a `benchmark_results.csv`.

| Campo | Descripción |
|-------|-------------|
| `scenario` | `Scenario_A` o `Scenario_B` |
| `algorithm` | `A*`, `Dijkstra`, `BFS` |
| `success` | 1 si encontró ruta, 0 si falló |
| `nodes_expanded` | Cantidad de nodos expandidos durante la búsqueda |
| `cpu_time_ms` | Tiempo de CPU del algoritmo en milisegundos |
| `path_length_metres` | Longitud de la ruta en metros |
| `start_x / start_y` | Coordenadas de inicio (mundo) |
| `goal_x / goal_y` | Coordenadas de destino (mundo) |

### 9.4 Análisis comparativo

El benchmark revela las diferencias fundamentales entre los algoritmos:

| Aspecto | A* | Dijkstra | BFS |
|---------|----|----------|-----|
| Heurística | Manhattan | Ninguna (costo uniforme) | Ninguna |
| Garantía | Ruta óptima (si heurística admisible) | Ruta de costo mínimo | Ruta con menos pasos |
| Complejidad | O((V+E) log V) | O((V+E) log V) | O(V+E) |
| Uso de memoria | Prioridad (heap) + visited | Prioridad (heap) + visited | Cola FIFO + visited |
| Mejor para | Mapas grandes con obstáculos dispersos | Mapas con costos variables | Mapas sin costos diferenciales |

---

## 10. Referencia de Topics ROS

### Topics de Entrada (el robot recibe)

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/mechdog/goal` | `geometry_msgs/PoseStamped` | Goal de navegación |
| `/mechdog/emergency_stop` | `std_msgs/Bool` | Parada de emergencia manual |

### Topics de Salida (el robot publica)

| Topic | Tipo | Hz | Descripción |
|-------|------|----|-------------|
| `/mechdog/odom` | `nav_msgs/Odometry` | 50 | Odometría con ruido (frame: `odom`) |
| `/mechdog/ultrasonic` | `sensor_msgs/Range` | 20 | HC-SR04 ultrasonido (FOV 15°, 3.0m) |
| `/mechdog/scan` | `sensor_msgs/LaserScan` | 20 | 1-rayo LaserScan (compatibilidad nav stack) |
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
| `/gazebo/ultrasonic_clean` | HC-SR04 sin ruido (Range) del plugin gazebo_ros_range |
| `/clock` | Tiempo de simulación (use_sim_time=true) |

---

## 11. Arquitectura de los Paquetes

### mechdog_description

Modelo físico del robot en formato URDF/Xacro.

- **Cuerpo**: Box 0.5×0.3×0.2m, masa 15kg
- **Patas**: 4 cilindros, masa 0.5kg cada uno (joints fijos)
- **Ultrasonido HC-SR04**: Caja 0.02×0.02×0.01m en `x=0.28m, z=0.15m` desde base_link
  - Tipo: range, FOV 15° (0.26 rad), 0.02-3.0m, 20Hz
  - Plugin: `libgazebo_ros_range.so` → `/gazebo/ultrasonic_clean`
- **Odometría**: Plugin `libgazebo_ros_p3d.so` → `/gazebo/odom_clean`
- **Archivos**: `mechdog.urdf.xacro` (modelo), `mechdog_gazebo.xacro` (plugins Gazebo)

### mechdog_sim

Capa de simulación que conecta Gazebo con el stack de navegación.

| Nodo | Función |
|------|---------|
| `spawn_robot.py` | Borra modelo anterior y hace spawn limpio en Gazebo |
| `noise_injector_node.py` | Agrega ruido gaussiano + drift a odom y LIDAR (Sim-to-Real) |
| `sensor_simulator_node.py` | Monitorea salud de sensores, logs timeouts |
| `metrics_collector_node.py` | Guarda métricas en `/app/metrics_output/` |

**Parámetros de ruido** (`noise_injection.yaml`):
- Ultrasonido: σ=0.01m, 1% outliers, 2% dropouts
- Odometría: σ=0.005m, drift 0.001m/s, σ_θ=0.002rad/s, drift angular 0.002rad/s
- Actuadores: retardo 50ms, respuesta de primer orden
- Suelo: variación de fricción 0.2 a 2Hz

### mechdog_navigation

Stack de navegación autónoma 100% portable (sin dependencias de Gazebo).

```
/mechdog/goal ──▶ navigation_manager ──▶ global_planner (PlannerStrategy ABC)
                                      │   ┌ A* / Dijkstra / BFS
                                      │   └───▶ /mechdog/global_plan
                                      ├──▶ scan_behavior (rotación 360° si no hay ruta)
                                      ├──▶ local_planner (DWA)
                                      ├──▶ safe_learning (freno activo)
                                      └──▶ occupancy_grid_mapper
```

| Nodo | Algoritmo | Frecuencia |
|------|-----------|------------|
| `planner_strategy.py` | ABC con A*, Dijkstra y BFS (heapq) | — |
| `global_planner_node.py` | Delegación a PlannerStrategy | 1 Hz (o al recibir goal) |
| `scan_behavior_node.py` | Rotación 360° a 0.3 rad/s si no hay plan | Reactivo |
| `local_planner_node.py` | DWA (Dynamic Window Approach) | 20 Hz |
| `safe_learning_node.py` | Freno predictivo + polígono dinámico | 50 Hz |
| `occupancy_grid_node.py` | Bayesian log-odds update | Por scan |
| `navigation_manager_node.py` | Máquina de estados FSM | Reactivo |

**Fórmula de frenado (Safe Learning)**:
```
d_freno = v² / (2×a) + v × t_reacción   × factor_seguridad
```

**Múltiples launch files** para arrancar componentes individuales:

| Launch File | Componentes |
|-------------|-------------|
| `navigation.launch` | Stack completo (5 nodos) |
| `planning.launch` | Solo global + local planner |
| `mapping.launch` | Solo occupancy grid |
| `safe_learning.launch` | Solo capa de seguridad |

### mechdog_experiments

Framework para ejecutar experimentos automatizados de navegación.

| Script | Función |
|--------|---------|
| `experiment_runner.py` | Ejecuta trials (algoritmo × escenario × réplicas) |
| `scenario_builder.py` | Construye escenarios en Gazebo desde YAML |
| `home_base_node.py` | Gestiona punto de origen y recuperación |
| `metrics_aggregator.py` | Agrega métricas multi-trial (éxito, tiempo, colisiones) |
| `report_generator.py` | Genera reporte HTML con gráficas |

**Métricas registradas por trial**:
- Tasa de éxito / fallo
- Tiempo hasta alcanzar goal
- Distancia recorrida vs distancia óptima
- Número de colisiones
- Tiempo en estado `planning` vs `moving`
- Suavidad de trayectoria (jerks)

**Archivos de configuración**:

| Archivo | Propósito |
|---------|-----------|
| `experiment_config.yaml` | Algoritmos (A*, BFS), escenarios, réplicas, timeout |
| `scenarios.yaml` | Definición de obstáculos por escenario |
| `home_base.yaml` | Posición de home y comportamiento de recovery |

---

## 12. Troubleshooting

### Error: `entity already exists` o `Address already in use`

**Causa**: Hay una sesión de Gazebo corriendo de una ejecución anterior.

**Solución**:
```bash
# Opción A (recomendada): reiniciar con start.sh
./start.sh stop
./start.sh all

# Opción B: limpiar procesos en el contenedor
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

**Solución**: El plugin P3D de Gazebo tarda ~10s en activarse tras el spawn. Si persiste:
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

**Solución**: `mechdog_viz` usa bridge networking con puertos explícitos.

**Verificación**:
```bash
docker ps --filter "name=mechdog_viz" --format "{{.Ports}}"
# Debe mostrar: 0.0.0.0:6080->6080/tcp
```

### La navegación dice `idle` después del goal

**Causa**: El topic `/mechdog/odom` no tiene datos — el robot no sabe dónde está.

**Verificación**:
```bash
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  timeout 2 rostopic echo /mechdog/odom -n 1 | head -4
"
```

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

# Ver logs de experimentos
docker compose exec mechdog_viz bash -c "tail -f /tmp/experiments.log"

# Parar el robot inmediatamente
docker compose exec mechdog_viz bash -c "
  source /app/catkin_ws/devel/setup.bash &&
  rostopic pub /mechdog/emergency_stop std_msgs/Bool 'data: true' --once
"
```

---

## Licencia

Proyecto universitario de investigación en robótica autónoma.
