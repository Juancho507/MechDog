# MechDog — Project Workflow

## FLUJO DE EJECUCIÓN COMPLETO (Sim-to-Real Identical)

### Fase de Inicio (t=0 a t=5s)

```
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 1: Launch Simulation/Hardware Layer                            │
└─────────────────────────────────────────────────────────────────────┘

[Simulación]
$ roslaunch mechdog_sim gazebo_pathway.launch seed:=42 add_noise:=true

ACCIONES INTERNAS:
├─ t=0.0s: Gazebo inicia, carga world pathway_obstacles_static.world
├─ t=0.5s: robot_state_publisher lee mechdog.urdf.xacro → publica TF estática
├─ t=1.0s: spawn_model spawnnea robot en (x=0, y=0, z=0.2)
├─ t=1.5s: Plugins Gazebo activan:
│   ├─ gazebo_ros_control → inicializa controladores de articulaciones
│   ├─ gazebo_ros_laser → comienza a publicar /gazebo/lidar_clean (sin ruido)
│   └─ gazebo_ros_p3d → publica ground truth en /gazebo/odom_ground_truth
├─ t=2.0s: noise_injection_node.py inicia:
│   ├─ Subscribe: /gazebo/lidar_clean, /gazebo/odom_ground_truth
│   ├─ Aplica ruido gaussiano según sim_noise_params.yaml
│   └─ Publish: /mechdog/sensor/scan, /mechdog/odom (con ruido)
└─ t=2.5s: TF completo disponible: map→odom→base_footprint→base_link→lidar_link

[Hardware Real]
$ roslaunch mechdog_hw hardware_drivers.launch

ACCIONES INTERNAS:
├─ t=0.0s: lidar_driver_node conecta con hardware LIDAR (vía serie/USB)
├─ t=0.5s: motor_controller_node inicializa CAN bus a motores
├─ t=1.0s: state_estimator_node (EKF) inicia sensor fusion:
│   ├─ Subscribe: /mechdog/sensor/imu, /mechdog/joint_states
│   └─ Publish: /mechdog/odom (estimación filtrada)
├─ t=1.5s: robot_state_publisher carga URDF → publica TF estática
└─ t=2.0s: lidar_driver comienza a publicar /mechdog/sensor/scan (datos reales)

┌─────────────────────────────────────────────────────────────────────┐
│ STEP 2: Launch Navigation Stack (CORE — idéntico en sim y real)    │
└─────────────────────────────────────────────────────────────────────┘

$ roslaunch mechdog_navigation navigation_stack.launch goal_x:=10.0 goal_y:=0.0

ACCIONES INTERNAS:
├─ t=2.5s: map_server inicia (opcional, si hay mapa estático):
│   ├─ Carga mapa pre-construido desde pathway_map.yaml
│   └─ Publica /map (nav_msgs/OccupancyGrid) estático
│
├─ t=3.0s: move_base inicia (nodo principal de navegación):
│   ├─ Carga configuración desde:
│   │   ├─ costmap_common_params.yaml (footprint robot, inflation)
│   │   ├─ global_costmap_params.yaml (usa /map de map_server)
│   │   ├─ local_costmap_params.yaml (rolling window, sensor updates)
│   │   ├─ base_local_planner_params.yaml (DWA con max_vel_y=0.0)
│   │   └─ global_planner_params.yaml (NavfnROS o GlobalPlanner)
│   ├─ Inicializa global_planner (Dijkstra sobre /map)
│   ├─ Inicializa local_planner (DWAPlannerROS plugin)
│   ├─ Costmaps construyen representación interna desde /scan y /odom
│   ├─ Espera recibir goal via /move_base/goal (MoveBaseActionGoal)
│   └─ Publica /move_base/cmd_vel (Twist, 10Hz) → comandos de velocidad
│
└─ t=3.5s: safe_controller_node inicia (CRÍTICO):
    ├─ Subscribe: /move_base/cmd_vel, /mechdog/odom, /mechdog/sensor/scan
    ├─ Verifica collision polygon + brake distance
    ├─ Publica velocidad verificada a /mechdog/cmd_vel_safe
    └─ Robot físico/sim SOLO lee /mechdog/cmd_vel_safe (cierra loop)

VERIFICACIÓN (t=5s):
✓ rostopic hz /mechdog/sensor/scan  → debe ser ~10-30Hz
✓ rostopic hz /mechdog/odom         → debe ser ~50Hz
✓ rostopic hz /move_base/cmd_vel    → debe ser ~10Hz (DWA activo)
✓ rosnode list                       → confirma move_base + safe_controller + hw/sim nodes activos
✓ rosrun tf view_frames              → verifica TF tree completo
```

---

### Fase de Exploración (t=5s a t=T_final)

#### Ciclo de Percepción → Planning → Control (loop a 10Hz)

```
t=5.0s ───────────────────────────────────────────────────────────────
│
├─► PERCEPCIÓN (Pipeline de Sensores)
│   │
│   ├─ (1) LIDAR publish sensor/scan @ 20Hz
│   │       ├─ 360 lecturas, rango [0.1, 10.0]m
│   │       └─ frame_id: "lidar_link"
│   │
│   ├─ (2) move_base/global_costmap actualiza @ 1Hz
│   │       ├─ Lee /map (mapa estático desde map_server)
│   │       ├─ Aplica inflación según robot footprint (0.15m radius)
│   │       └─ Publica /move_base/global_costmap/costmap
│   │
│   ├─ (2b) move_base/local_costmap actualiza @ 5Hz
│   │       ├─ Rolling window 5x5m centrada en robot
│   │       ├─ Integra /mechdog/sensor/scan en tiempo real
│   │       ├─ Marca obstáculos dinámicos (no en mapa estático)
│   │       └─ Publica /move_base/local_costmap/costmap
│   │
│   └─ (3) Odometry publish /mechdog/odom @ 50Hz
│           ├─ Pose: (x, y, θ) en frame "odom"
│           └─ Twist: (vx, vy, ωz) velocidades actuales (NOTE: vy=0 siempre)
│
├─► PLANIFICACIÓN GLOBAL (Trigger: al recibir goal O costmap cambia >10%)
│   │
│   ├─ (4) move_base recibe /move_base/goal (MoveBaseActionGoal)
│   │       ├─ Goal: target_pose=(x, y, θ) en frame "map"
│   │       └─ Activa state machine: PLANNING → CONTROLLING → GOAL
│   │
│   ├─ (5) NavfnROS (global_planner) ejecuta Dijkstra
│   │       ├─ Lee /move_base/global_costmap/costmap
│   │       ├─ Extrae start desde TF transform map→base_link
│   │       ├─ Convierte costmap a graph (8-conectividad, celdas FREE)
│   │       ├─ Ejecuta Dijkstra desde start hasta goal
│   │       ├─ Genera path interpolado (0.2m entre waypoints)
│   │       └─ Publica /move_base/NavfnROS/plan (nav_msgs/Path) @ 1Hz
│   │
│   └─ MANEJO DE ERRORES (move_base internal):
│       ├─ Si planning falla (no path): activa recovery behaviors
│       │   ├─ Clear costmap → retry planning
│       │   ├─ Rotate recovery (360° in-place) → retry
│       │   └─ Si fallan todos: abort goal (status=ABORTED)
│       └─ Si goal en obstáculo: expande búsqueda radius 0.5m
│
├─► PLANIFICACIÓN LOCAL (Loop a 10Hz — cinemática no-holonómica)
│   │
│   ├─ (6) DWAPlannerROS (local_planner) recibe:
│   │       ├─ /move_base/NavfnROS/plan (global path)
│   │       ├─ /mechdog/odom (pose + velocidades actuales)
│   │       ├─ /move_base/local_costmap/costmap (obstáculos dinámicos)
│   │       └─ TF transform odom→base_link
│   │
│   ├─ (7) DWA calcula ventana dinámica (NO-HOLONÓMICA):
│   │       ├─ Genera N=200 trayectorias candidatas:
│   │       │   ├─ vx samples: 10 valores en [min_vel_x, max_vel_x]
│   │       │   ├─ ωz samples: 20 valores en [-max_vel_theta, +max_vel_theta]
│   │       │   └─ vy forzado a 0.0 (restricción cinemática)
│   │       ├─ Simula cada trayectoria 1.5s adelante (dt=0.05s steps)
│   │       ├─ Scoring: f = path_distance_bias·d_path + goal_distance_bias·d_goal - occdist_scale·d_obs
│   │       │   ├─ d_path: distancia a global path
│   │       │   ├─ d_goal: distancia a meta
│   │       │   └─ d_obs: distancia a obstáculos (penalización)
│   │       ├─ Descarta trayectorias que colisionan con local_costmap
│   │       ├─ Selecciona trayectoria con max(score)
│   │       └─ Extrae (vx, ωz) del primer paso (vx, 0.0, ωz)
│   │
│   └─ (8) Publica /move_base/cmd_vel:
│           └─ Twist(linear.x=vx, linear.y=0.0, angular.z=ωz)
│
├─► SAFE CONTROLLER (Loop a 20Hz — seguridad física adicional)
│   │
│   ├─ (9) safe_controller_node recibe:
│   │       ├─ /move_base/cmd_vel (velocidad deseada por DWA)
│   │       ├─ /mechdog/odom (velocidad actual del robot)
│   │       └─ /mechdog/sensor/scan (obstáculos inmediatos)
│   │
│   ├─ (10) Cálculo de Polígono de Colisión Dinámico:
│   │       ├─ Base: dimensiones URDF del robot (largo=0.5m, ancho=0.3m)
│   │       ├─ Expansión frontal: margin = v² / (2 * brake_decel)
│   │       │   └─ Ejemplo: v=1m/s, brake_decel=2m/s² → margin = 0.25m extra
│   │       ├─ Expansión lateral: +10% del ancho (robustez ante desplazamientos)
│   │       └─ Publica /mechdog/collision_polygon (para visualización)
│   │
│   ├─ (11) Verificación de Obstáculos:
│   │       ├─ Proyecta cada lectura de LIDAR en frame base_link
│   │       ├─ Chequea si algún punto está dentro del polígono de colisión
│   │       └─ Si colisión inminente (distancia < margin):
│   │           ├─ EMERGENCY STOP:
│   │           │   ├─ Publica /mechdog/emergency_stop = True
│   │           │   ├─ Envía cmd_vel_safe = (0, 0) → robot frena
│   │           │   ├─ Espera 0.5s (robot se detiene completamente)
│   │           │   ├─ Ejecuta rotación in-place: cmd_vel_safe = (0, +0.5rad/s)
│   │           │   │   └─ Gira hasta que clearance frontal > 1.5m
│   │           │   └─ Retorna control a move_base (emergency_stop = False)
│   │           │
│   │           └─ Si tras 360° no hay clearance (callejón sin salida):
│   │               ├─ Cancela goal actual de move_base
│   │               └─ Retrocede 0.5m (cmd_vel_safe = (-0.2, 0, 0)) y re-evalúa
│   │
│   └─ (12) Ajuste de Velocidad (si no hay emergencia):
│           ├─ Reduce vx si clearance < 2m: vx_safe = vx * (clearance/2)
│           ├─ Limita aceleración: |vx_safe - vx_prev| < a_max * dt
│           └─ Publica /mechdog/cmd_vel_safe → robot ejecuta
│
└─► ACTUACIÓN (Hardware/Sim consume cmd_vel_safe)
    │
    ├─ [Simulación] Gazebo plugin lee /mechdog/cmd_vel_safe @ 20Hz
    │   └─ Aplica fuerzas a articulaciones según cinemática inversa
    │
    └─ [Hardware Real] motor_controller_node lee /mechdog/cmd_vel_safe @ 20Hz
        ├─ Convierte (vx, ωz) a velocidades por pata (IK cuadrúpedo)
        └─ Envía comandos PWM a motores vía CAN bus

───────────────────────────────────────────────────────────────────────

MONITOREO CONTINUO (Paralelo al loop principal):

├─ metrics_recorder (graba en background):
│   ├─ rosbag record /mechdog/odom /move_base/NavfnROS/plan /map \
│   │                /move_base/cmd_vel /mechdog/cmd_vel_safe \
│   │                /mechdog/emergency_stop /move_base/status
│   └─ Archivo: ~/rosbags/run_<timestamp>.bag
│
└─ rviz (visualización en tiempo real — opcional):
    ├─ Display /map → mapa estático
    ├─ Display /move_base/global_costmap/costmap → costmap global inflado
    ├─ Display /move_base/local_costmap/costmap → costmap local dinámico
    ├─ Display /move_base/NavfnROS/plan → plan global (Dijkstra)
    ├─ Display /move_base/DWAPlannerROS/local_plan → trayectoria local
    ├─ Display /mechdog/collision_polygon → polígono de seguridad (safe_controller)
    └─ Display TF tree → verifica frames consistentes (map→odom→base_footprint→base_link)
```

---

### Fase de Convergencia (robot alcanza meta)

```
t=T_goal ─────────────────────────────────────────────────────────────
│
├─► (13) Condición de Llegada:
│       ├─ move_base detecta: dist(robot_pose, goal) < xy_goal_tolerance (0.2m)
│       │                    AND |θ_robot - θ_goal| < yaw_goal_tolerance (0.1rad)
│       ├─ Publica /move_base/status = SUCCEEDED (actionlib status)
│       └─ DWAPlannerROS detiene generación de trayectorias, publica cmd_vel=(0,0,0)
│
├─► (14) Frenado Final:
│       ├─ safe_controller verifica que robot se detuvo:
│       │   └─ Monitorea /mechdog/odom hasta |v| < 0.01m/s
│       └─ Publica cmd_vel_safe = (0, 0, 0) → robot estático
│
└─► (15) Post-Procesamiento:
        ├─ Guardar mapa final (si fue construido dinámicamente):
        │   └─ rosrun map_server map_saver -f ~/maps/map_final map:=/map
        ├─ metrics_recorder detiene grabación rosbag
        └─ Imprime estadísticas:
            ├─ Tiempo total: T_goal - T_start
            ├─ Distancia recorrida: ∫|v(t)|dt desde odom
            ├─ # de emergency stops: count(msg en /emergency_stop == True)
            └─ Ratio explorado: count(FREE)/count(UNKNOWN) en map
```

---

## DIAGRAMA DE SECUENCIA (Caso Nominal)

```
Tiempo  │ Sensor  │ OccGrid │ Global  │ Local   │ Safe    │ Robot
        │         │ Builder │ Planner │ Planner │ Control │
────────┼─────────┼─────────┼─────────┼─────────┼─────────┼──────────
t=0     │ init    │         │         │         │         │
t=1     │ ───scan──────►    │         │         │         │
t=2     │         │ ───map──────────► │         │         │
t=3     │         │         │ ─path───────────► │         │
t=4     │         │         │         │ ─vel_raw────────► │
t=5     │         │         │         │         │ ─vel_safe──────►
t=6     │         │         │         │         │         │ move
t=7     │ ───scan──────►    │         │         │         │
t=8     │         │ update  │         │         │         │
...     │         │         │         │         │         │
t=100   │         │         │ replan  │         │         │
...     │         │         │         │         │ STOP!   │
t=105   │         │         │         │ recalc  │         │
t=106   │         │         │         │         │ ─resume─────────►
...     │         │         │         │         │         │
t=500   │         │         │ goal_reached ──────────────────────►
        │         │         │         │         │         │ stop
```

---

## MANEJO DE CASOS ESPECIALES

### Caso 1: Obstáculo Dinámico Aparece Súbitamente

```
Situación: Robot avanza a 0.8m/s, persona cruza súbitamente a 1m adelante

Timeline:
t=0.00s: LIDAR detecta objeto a 1.0m (scan.ranges[180] = 1.0)
         ├─ move_base/local_costmap actualiza → nueva celda OCCUPIED
         └─ Publica /move_base/local_costmap/costmap @ t=0.02s

t=0.02s: DWAPlannerROS recibe costmap actualizado
         ├─ Recalcula ventana dinámica: trayectorias con vx>0.5 colisionan
         └─ Selecciona trayectoria conservadora: vx=0.2, ωz=-0.3 (giro+freno)
         └─ Publica /move_base/cmd_vel=(0.2, 0.0, -0.3) @ t=0.04s

t=0.04s: safe_controller recibe /move_base/cmd_vel=(0.2, 0.0, -0.3)
         ├─ Verifica: brake_distance = 0.8²/(2*2) = 0.16m
         ├─ Clearance actual (desde LIDAR) = 1.0m → OK (sin colisión)
         ├─ PERO: margen insuficiente para v=0.8 actual
         └─ OVERRIDE: reduce vx_safe = 0.0 (emergency brake)
         └─ Publica cmd_vel_safe=(0.0, 0.0, 0.0) @ t=0.06s

t=0.06s: Robot ejecuta freno → se detiene en ~0.2s (desacelera 2m/s²)

t=0.30s: Robot completamente detenido
         ├─ safe_controller detecta: vx_actual < 0.01m/s
         ├─ Clearance frontal sigue siendo <1.5m (obstáculo no se movió)
         └─ Inicia rotación: cmd_vel_safe=(0.0, 0.0, +0.5rad/s)

t=1.50s: Tras rotar 60°, clearance frontal = 3.0m (obstáculo está a un lado)
         ├─ safe_controller: emergency_stop=False
         └─ Retorna control a move_base (pasa /move_base/cmd_vel sin override)

t=1.60s: DWAPlannerROS replannea con nueva orientación → robot continúa
```

**Métricas registradas**:
- emergency_stop duration: 1.24s
- Desviación del global_path: +0.8m (por el desvío lateral)

---

### Caso 2: Path Bloqueado (Re-Planning Global)

```
Situación: Obstáculo estático bloquea el camino planificado

Timeline:
t=0.0s: move_base/NavfnROS genera plan global con 15 waypoints

t=30.0s: Robot llega a waypoint 8, detecta obstáculo grande (muro) en waypoint 9
         ├─ DWAPlannerROS intenta generar trayectoria durante 5s
         └─ Todas las trayectorias colisionan (score=-inf, ninguna válida)

t=35.0s: DWAPlannerROS timeout → reporta failure a move_base
         └─ move_base activa recovery behaviors automáticamente

t=35.1s: Recovery Behavior #1 — Clear Costmap
         ├─ move_base limpia obstáculos antiguos del local_costmap
         ├─ Re-intenta planning local → sigue fallando (obstáculo real)
         └─ Escala a Recovery #2

t=35.5s: Recovery Behavior #2 — Rotate Recovery
         ├─ move_base ejecuta rotación 360° in-place
         ├─ Actualiza costmaps con nuevas vistas
         └─ Re-intenta planning local → sigue fallando

t=36.5s: Recovery Behavior #3 — Global Replan
         ├─ move_base invalida plan global anterior
         ├─ NavfnROS re-ejecuta Dijkstra con costmap actualizado
         ├─ Encuentra plan alternativo (rodea obstáculo por izquierda)
         └─ Publica nuevo /move_base/NavfnROS/plan con 20 waypoints

t=37.0s: DWAPlannerROS recibe nuevo plan global → continúa normalmente
```

**Métricas registradas**:
- replan_count: +1
- Tiempo de re-planning: 0.9s
- Incremento en longitud de path: +2.5m

---

## SINCRONIZACIÓN CRÍTICA (Timing Constraints)

| Componente | Frecuencia Mínima | Latencia Máxima | Justificación |
|------------|-------------------|-----------------|---------------|
| `/mechdog/sensor/scan` | 10 Hz | 100 ms | Obstáculos a 1m/s se mueven 10cm entre scans |
| `/mechdog/odom` | 50 Hz | 20 ms | Control loop de velocidad requiere feedback rápido |
| `DWAPlannerROS` (local planner) | 10 Hz | 100 ms | Balance entre calidad de planning y carga CPU |
| `safe_controller` | 20 Hz | 50 ms | Tiempo de reacción < brake_time (~200ms @ 2m/s²) |
| `NavfnROS` (global planner) | 1 Hz | 1 s | Re-planning no es crítico, solo cuando costmap cambia |
| `/map` (estático) | 1 Hz | 1 s | Mapa estático, baja frecuencia suficiente |
| `/move_base/local_costmap` | 5 Hz | 200 ms | Integra sensores dinámicos, balance performance |

**Regla crítica**: `freq(safe_controller) ≥ 2 × freq(DWAPlannerROS)` para interceptar comandos peligrosos antes de ejecutarlos.

---

## COMANDOS DE EJECUCIÓN COMPLETOS

### Simulación con Ruido (Entrenamiento)

```bash
# Terminal 1: Lanzar simulación
roslaunch mechdog_sim gazebo_pathway.launch \
    world:=pathway_obstacles_dynamic \
    seed:=42 \
    add_noise:=true \
    gui:=true

# Terminal 2: Lanzar stack de navegación
roslaunch mechdog_navigation navigation_stack.launch \
    goal_x:=10.0 \
    goal_y:=0.0 \
    goal_tolerance:=0.2

# Terminal 3: Grabar métricas
roslaunch mechdog_metrics record_run.launch \
    output_dir:=~/rosbags/sim_noisy/run_001

# Terminal 4 (opcional): Visualizar en RViz
roslaunch mechdog_navigation rviz_navigation.launch
```

### Hardware Real (Despliegue)

```bash
# Terminal 1: Lanzar drivers de hardware
roslaunch mechdog_hw hardware_drivers.launch \
    calibration_file:=~/config/mechdog_real_calib.yaml

# Terminal 2: Lanzar stack de navegación (MISMO comando que simulación)
roslaunch mechdog_navigation navigation_stack.launch \
    goal_x:=10.0 \
    goal_y:=0.0 \
    goal_tolerance:=0.2

# Terminal 3: Grabar métricas
roslaunch mechdog_metrics record_run.launch \
    output_dir:=~/rosbags/real_hw/run_001
```

**NOTA CRÍTICA**: Los comandos de Terminal 2 y 3 son IDÉNTICOS. Solo cambia Terminal 1 (sim vs hw).

---

## ANÁLISIS POST-RUN

```bash
# Comparar mapa simulado vs real
rosrun mechdog_metrics compare_maps.py \
    --sim ~/rosbags/sim_noisy/run_001/final_map.pgm \
    --real ~/rosbags/real_hw/run_001/final_map.pgm \
    --output ~/reports/map_comparison.png

# Calcular desviación de trayectoria
rosrun mechdog_metrics trajectory_deviation.py \
    --bag ~/rosbags/real_hw/run_001/*.bag \
    --output ~/reports/trajectory_metrics.json

# Frecuencia de emergency stops
rosrun mechdog_metrics emergency_frequency.py \
    --bag ~/rosbags/real_hw/run_001/*.bag
```

**Output esperado** (ejemplo):
```json
{
  "map_iou": 0.87,
  "trajectory_rmse_cm": 8.3,
  "emergency_stop_rate": 0.03,
  "total_time_s": 124.5,
  "distance_traveled_m": 10.8,
  "replan_count": 2
}
```

---

## RESUMEN EJECUTIVO DEL FLUJO

**Pipeline Principal**:
```
LIDAR → move_base/local_costmap → NavfnROS (global) → DWAPlannerROS (local) → SafeController → Robot
         (5Hz update)                (1Hz replan)        (10Hz cmd_vel)          (20Hz vel_safe)
                                                         [vy=0 enforced]
```

**Loops Críticos**:
1. **Percepción**: 20Hz (LIDAR → local_costmap @ 5Hz processed)
2. **Planning Global**: 1Hz (Dijkstra sobre global_costmap, trigger cuando cambia >10%)
3. **Planning Local**: 10Hz (DWA con restricción no-holonómica: vx, ωz solamente)
4. **Safety Override**: 20Hz (safe_controller verifica + modifica si necesario)

**Garantías Sim-to-Real**:
- Stack ROS Navigation estándar (move_base + plugins)
- Configuración 100% mediante YAML (sin recompilación)
- Mismo URDF, mismos tópicos ROS
- Solo cambia launch file (sim vs hw) y environment overrides
- Validación previa con ruido en sim
- Restricción cinemática no-holonómica forzada (max_vel_y=0.0)

**Puntos de Control Críticos**:
1. `/move_base/cmd_vel`: Output de DWA (puede ser peligroso)
2. `/mechdog/cmd_vel_safe`: ÚNICO comando ejecutado por robot (post safe_controller)
3. `base_local_planner_params.yaml`: vy_samples=1, max_vel_y=0.0 (enforcement no-holonómico)
