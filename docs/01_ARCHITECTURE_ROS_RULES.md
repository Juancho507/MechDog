# MechDog — Architecture & ROS Rules

## SISTEMA TARGET
**Middleware**: ROS 1 Noetic (Python 3.8+, Catkin build system)  
**Objetivo**: Paridad absoluta Sim-to-Real mediante abstracción estricta hw/sim bajo topología ROS estándar  
**Entorno**: Camino confinado con obstáculos dinámicos  
**Robot físico**: Cuadrúpedo MechDog, sistema no-holonómico (unicycle-like: $v_x$, $\omega_z$ solamente, NO desplazamiento lateral $v_y$)

---

## ARQUITECTURA MODULAR ROS

### Topología de Nodos (Diagrama Lógico)

```
┌─────────────────────────────────────────────────────────────────┐
│                        SIMULATION LAYER                          │
│  ┌────────────────┐         ┌─────────────────────────────┐    │
│  │ Gazebo/PyBullet│ ◄─────► │ mechdog_sim_interface       │    │
│  │ (Physics + Viz)│         │ (Noise Injection + SDF/URDF)│    │
│  └────────────────┘         └─────────────────────────────┘    │
└───────────────────────────────────────┬─────────────────────────┘
                                        │ (ROS Topics/Services)
┌───────────────────────────────────────┴─────────────────────────┐
│                         HARDWARE ABSTRACTION                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  /mechdog/sensor_raw  (sensor_msgs/LaserScan|PointCloud2)│  │
│  │  /mechdog/odom        (nav_msgs/Odometry)                │  │
│  │  /mechdog/cmd_vel     (geometry_msgs/Twist)              │  │
│  │  /mechdog/joint_states(sensor_msgs/JointState, opcional) │  │
│  └──────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────┬─────────────────────────┘
                                        │
┌───────────────────────────────────────┴─────────────────────────┐
│                        PERCEPTION LAYER                          │
│  ┌──────────────────┐       ┌──────────────────────────────┐   │
│  │ occupancy_grid   │◄──────┤ sensor_processor             │   │
│  │ (map_server)     │       │ (filtrado, downsampling)     │   │
│  └──────────────────┘       └──────────────────────────────┘   │
│         │ pub: /map (nav_msgs/OccupancyGrid)                   │
└─────────┼──────────────────────────────────────────────────────┘
          │
┌─────────┴────────────────────────────────────────────────────────┐
│                       PLANNING & CONTROL LAYER (CORE)            │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ move_base (nodo principal)                               │  │
│  │  ├─ global_planner: NavfnROS (Dijkstra) o A*           │  │
│  │  ├─ local_planner: DWAPlannerROS (plugin)               │  │
│  │  ├─ global_costmap (mapa + inflación)                   │  │
│  │  └─ local_costmap (ventana móvil)                       │  │
│  │                                                           │  │
│  │  pub: /move_base/cmd_vel → safe_controller              │  │
│  └──────────────────────────────────────────────────────────┘  │
│                           │                                      │
│                  ┌────────▼────────┐                            │
│                  │ safe_controller │                            │
│                  │ (collision poly,│                            │
│                  │  brake dist,    │                            │
│                  │  emergency stop)│                            │
│                  └────────┬────────┘                            │
│                           │ pub: /mechdog/cmd_vel_safe          │
└───────────────────────────┼──────────────────────────────────────┘
                            │
┌───────────────────────────┴──────────────────────────────────────┐
│                      METRICS & LOGGING LAYER                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ metrics_recorder (rosbag record + analysis scripts)      │  │
│  │  - map_comparison (sim vs real occupancy grids)          │  │
│  │  - trajectory_deviation (global_path vs executed odom)   │  │
│  │  - emergency_frequency (safe_controller activation rate) │  │
│  └──────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

---

## CONVENCIONES ESTRICTAS

### 1. Nomenclatura de Nodos

**Formato**: `<namespace>/<function>_<type>`

Ejemplos válidos:
- `/mechdog/global_planner_node`
- `/mechdog/safe_controller_node`
- `/mechdog/occupancy_grid_builder_node`

**Restricciones**:
- PROHIBIDO: nombres genéricos sin namespace (`planner`, `controller`)
- OBLIGATORIO: sufijo `_node` para ejecutables principales
- Namespaces por robot: `/mechdog/` (simulación), `/mechdog_real/` (hardware real)

### 2. Nomenclatura de Tópicos

**Formato**: `/<namespace>/<sensor|state|cmd>/<data_type>`

| Categoría | Tópico | Tipo ROS | Frecuencia | Descripción |
|-----------|--------|----------|------------|-------------|
| **Sensor Raw** | `/mechdog/sensor/scan` | `sensor_msgs/LaserScan` | 10-30 Hz | LIDAR 2D o proyección 3D |
| | `/mechdog/sensor/cloud` | `sensor_msgs/PointCloud2` | 5-10 Hz | 3D point cloud (alternativa a scan) |
| **Odometría** | `/mechdog/odom` | `nav_msgs/Odometry` | 50 Hz | Pose + velocidad estimada |
| **Estado interno** | `/mechdog/joint_states` | `sensor_msgs/JointState` | 20 Hz | Estados de articulaciones (cuadrúpedo) |
| **Comando Movimiento** | `/move_base/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | Velocidad desde DWA (raw) |
| | `/mechdog/cmd_vel_safe` | `geometry_msgs/Twist` | 20 Hz | Velocidad tras safe_controller → robot |
| **Mapas** | `/map` | `nav_msgs/OccupancyGrid` | 1 Hz | Mapa estático (map_server o SLAM) |
| | `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 1 Hz | Costmap global (inflado) |
| | `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 5 Hz | Costmap local (ventana móvil) |
| **Planificación** | `/move_base/NavfnROS/plan` o `/move_base/GlobalPlanner/plan` | `nav_msgs/Path` | 1 Hz | Ruta global |
| | `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | 10 Hz | Trayectoria local DWA |
| | `/move_base/goal` | `move_base_msgs/MoveBaseActionGoal` | event | Goal para navegación |
| **Seguridad** | `/mechdog/emergency_stop` | `std_msgs/Bool` | event-driven | Trigger de parada de emergencia |
| | `/mechdog/collision_polygon` | `geometry_msgs/PolygonStamped` | 10 Hz | Polígono de seguridad dinámico |

**Restricciones**:
- PROHIBIDO: tópicos sin `/mechdog/` (contaminan namespace global)
- PROHIBIDO: abreviaturas no estándar (`/md/sp` ❌, `/mechdog/sensor/scan` ✅)
- OBLIGATORIO: `frame_id` consistente en headers (ver sección TF)

### 3. Frame Conventions (TF Tree)

**Árbol TF obligatorio**:

```
map
 └─ odom
     └─ base_footprint  (origen cinemático, proyección en suelo)
         └─ base_link   (centro de masa del robot)
             ├─ lidar_link
             ├─ camera_link (opcional)
             ├─ imu_link (opcional)
             └─ [leg_links...] (cinemática cuadrúpedo)
```

**Reglas**:
- `map` → `odom`: publicado por localization node (AMCL/EKF)
- `odom` → `base_footprint`: publicado por odometry source (robot_state_publisher + sensor fusion)
- `base_footprint` → `base_link`: estático (offset Z = altura del robot)
- Sensores SIEMPRE como child de `base_link` (definido en URDF)

**Prohibiciones estrictas**:
- NO publicar `map → base_link` directamente (rompe pipeline estándar de nav)
- NO usar `base_footprint` == `base_link` (ambigüedad en alturas)

### 4. Separación Hardware/Simulación (Abstracción Obligatoria)

#### 4.1. Capa de Simulación

**Archivos aislados en**: `mechdog_sim/` (paquete ROS separado)

```
mechdog_sim/
├── launch/
│   ├── gazebo_world.launch           # Carga mundo + spawn robot
│   ├── pybullet_sim.launch           # Alternativa PyBullet (custom)
│   └── noise_injection.launch        # Wrapper con parámetros de ruido
├── worlds/
│   ├── pathway_obstacles_static.world
│   └── pathway_obstacles_dynamic.world
├── urdf/
│   └── mechdog.urdf.xacro            # Descripción robot (sim + real)
├── config/
│   └── sim_noise_params.yaml         # σ para sensores (ver sección Sim-to-Real)
└── scripts/
    └── obstacle_spawner.py           # Genera obstáculos procedurales
```

**Responsabilidades**:
- Publicar `/mechdog/sensor/*`, `/mechdog/odom`, `/mechdog/joint_states`
- Consumir `/mechdog/cmd_vel_safe`
- NO contener lógica de planning/control (solo física + noise)

#### 4.2. Capa de Hardware Real

**Archivos aislados en**: `mechdog_hw/` (paquete ROS separado)

```
mechdog_hw/
├── launch/
│   └── hardware_drivers.launch       # drivers LIDAR, IMU, motores
├── config/
│   └── hw_calibration.yaml           # offsets de sensores, PID motores
└── src/
    ├── lidar_driver_node.cpp         # publica /mechdog/sensor/scan
    ├── motor_controller_node.cpp     # suscribe /mechdog/cmd_vel_safe
    └── state_estimator_node.cpp      # publica /mechdog/odom (EKF)
```

**Responsabilidades**:
- Mismos tópicos que simulación (contract idéntico)
- NO contener lógica de planning/control

#### 4.3. Código CORE (Hardware-Agnostic)

**Paquete**: `mechdog_navigation/` (100% portable sim↔real)

```
mechdog_navigation/
├── launch/
│   ├── navigation_stack.launch       # lanza TODOS los nodos CORE
│   └── move_base.launch              # configura move_base con DWA plugin
├── config/
│   ├── costmap_common_params.yaml    # Parámetros compartidos costmaps
│   ├── global_costmap_params.yaml    # Costmap para global planner
│   ├── local_costmap_params.yaml     # Costmap para DWA
│   ├── base_local_planner_params.yaml # Configuración DWAPlannerROS
│   ├── global_planner_params.yaml    # NavfnROS o custom global planner
│   └── safe_controller.yaml          # Parámetros safe learning
└── src/
    ├── safe_controller_node.py       # collision polygon + brake logic
    └── occupancy_grid_builder_node.py # sensor fusion → /map (si custom)
```

**RESTRICCIÓN CRÍTICA**: Este paquete NO debe tener dependencias de `mechdog_sim` o `mechdog_hw`. Dependencias permitidas:
- ROS std: `rospy`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf`, `tf2_ros`
- ROS Navigation: `move_base`, `costmap_2d`, `base_local_planner` (DWA plugin), `navfn` o `global_planner`

---

## REGLAS DE DESARROLLO (Enforcement)

### R1: Isolation Principle
**NUNCA** importar desde `mechdog_sim` o `mechdog_hw` dentro de `mechdog_navigation`.

❌ Prohibido:
```python
from mechdog_sim.utils import get_robot_position  # ROMPE PORTABILIDAD
```

✅ Correcto:
```python
import rospy
from nav_msgs.msg import Odometry
# Leer posición desde tópico /mechdog/odom
```

### R2: Configuration via ROS Parameters
Toda variable dependiente de hw/sim debe venir de `rosparam`:

```yaml
# En mechdog_navigation/config/safe_controller.yaml
safe_controller:
  max_linear_velocity: 0.5      # m/s
  max_angular_velocity: 1.0     # rad/s
  collision_margin_base: 0.15   # m (depende del tamaño físico del robot)
  brake_deceleration: 2.0       # m/s² (física del robot)
```

Luego en código:
```python
self.max_vel = rospy.get_param('~max_linear_velocity', 0.5)
```

**Prohibido**: hardcodear valores físicos del robot en `mechdog_navigation/src/*.py`.

### R3: Topic Remapping Only
Diferencias sim/real se resuelven SOLO con remapping en launch files.

Ejemplo (simulación):
```xml
<launch>
  <include file="$(find mechdog_sim)/launch/gazebo_world.launch"/>
  <include file="$(find mechdog_navigation)/launch/navigation_stack.launch">
    <arg name="scan_topic" value="/mechdog/sensor/scan"/>  <!-- Gazebo publica aquí -->
  </include>
</launch>
```

Hardware real:
```xml
<launch>
  <include file="$(find mechdog_hw)/launch/hardware_drivers.launch"/>
  <include file="$(find mechdog_navigation)/launch/navigation_stack.launch">
    <arg name="scan_topic" value="/mechdog/sensor/scan"/>  <!-- Driver LIDAR publica aquí -->
  </include>
</launch>
```

**El código CORE no cambia**. Solo cambian los launch files.

### R4: URDF as Single Source of Truth
El archivo `mechdog.urdf.xacro` define:
- Geometría de colisión (usada por safe_controller para `collision_margin`)
- Posición de sensores (transforms en TF)
- Límites de velocidad de articulaciones (si es cuadrúpedo)

**Simulación** y **hardware** cargan el MISMO URDF. Diferencias menores (gazebo plugins) vía xacro conditionals:

```xml
<xacro:if value="$(arg simulation)">
  <gazebo reference="lidar_link">
    <sensor type="ray" name="lidar_sensor">
      <!-- Gazebo-specific config -->
    </sensor>
  </gazebo>
</xacro:if>
```

### R5: Test Before Real
**Flujo obligatorio**:
1. Desarrollar en simulación sin ruido
2. Agregar ruido (via `sim_noise_params.yaml`)
3. Validar métricas en sim con ruido
4. Desplegar en hardware real (mismo código, launch diferente)
5. Comparar métricas sim_noisy vs real (deben ser < 10% desviación)

---

## PARÁMETROS CRÍTICOS PARA SIM-TO-REAL

**Archivo**: `mechdog_sim/config/sim_noise_params.yaml`

```yaml
sensor_noise:
  lidar:
    range_stddev: 0.02           # m (2 cm std dev en mediciones de rango)
    angle_stddev: 0.005          # rad (~0.3 grados)
    dropout_probability: 0.01    # 1% de lecturas perdidas
  odom:
    linear_drift: 0.05           # m/s acumulación de error lineal
    angular_drift: 0.02          # rad/s acumulación de error angular
    update_rate_jitter: 0.1      # ±10% variación en frecuencia

physics_noise:
  friction_coefficient_stddev: 0.1  # variación en fricción suelo
  mass_uncertainty: 0.05            # ±5% masa del robot (para inercia)
  motor_response_delay: 0.05        # segundos (lag cmd_vel → movimiento real)
```

**Implementación**: Nodo `noise_injection_node.py` subscribe a tópicos limpios de Gazebo/PyBullet, aplica ruido, republica con nombres reales.

```python
# Pseudocódigo (ver sección Workflow para detalles)
sub_clean = rospy.Subscriber('/gazebo/lidar_clean', LaserScan, callback)
pub_noisy = rospy.Publisher('/mechdog/sensor/scan', LaserScan, queue_size=10)

def callback(msg):
    noisy_msg = add_gaussian_noise(msg, params['lidar'])
    pub_noisy.publish(noisy_msg)
```

---

## CONFIGURACIÓN DE NAVEGACIÓN (move_base + DWA)

### Integración con move_base

**Decisión arquitectónica**: MechDog utiliza el stack estándar de ROS Navigation (`move_base`) con el plugin `base_local_planner/DWAPlannerROS` en lugar de implementar DWA desde cero.

**Ventajas**:
- Código battle-tested en producción (miles de robots reales)
- Integración nativa con costmaps y recovery behaviors
- Tuning mediante YAML (no recompilación)
- Compatibilidad con herramientas ROS (rviz plugins, rqt_reconfigure)

**Topología de nodos**:
```
move_base (nodo principal)
  ├─ global_planner: navfn/NavfnROS (Dijkstra) o global_planner/GlobalPlanner (A*)
  ├─ local_planner: base_local_planner/DWAPlannerROS
  ├─ global_costmap: costmap_2d (mapa estático + inflación)
  └─ local_costmap: costmap_2d (ventana móvil + obstáculos dinámicos)
```

### Restricción Cinemática No-Holonómica (CRÍTICO)

**MechDog NO tiene desplazamiento lateral**. Es un sistema no-holonómico tipo unicycle:
- ✅ Velocidad lineal frontal: $v_x$ ∈ [-v_max, +v_max]
- ✅ Velocidad angular (yaw): $\omega_z$ ∈ [-ω_max, +ω_max]
- ❌ Velocidad lateral: $v_y$ = 0 (SIEMPRE)

**Configuración obligatoria en `base_local_planner_params.yaml`**:

```yaml
DWAPlannerROS:
  # ═══════════════════════════════════════════════════════
  # RESTRICCIÓN CINEMÁTICA NO-HOLONÓMICA (CRÍTICO)
  # ═══════════════════════════════════════════════════════
  max_vel_y: 0.0                    # Sin desplazamiento lateral
  min_vel_y: 0.0
  acc_lim_y: 0.0                    # Sin aceleración lateral
  vy_samples: 1                     # Solo sample vy=0 (no explorar lateral)
  
  # Velocidades lineales (eje X - frontal)
  max_vel_trans: 0.5                # m/s (ajustar según hw real)
  min_vel_trans: 0.1                # m/s (velocidad mínima para evitar stall)
  max_vel_x: 0.5
  min_vel_x: -0.2                   # Permitir retroceso lento
  acc_lim_x: 1.5                    # m/s² (aceleración lineal)
  
  # Velocidades angulares (eje Z - rotación yaw)
  max_vel_theta: 1.0                # rad/s
  min_vel_theta: 0.2                # rad/s (evitar oscilaciones)
  acc_lim_theta: 2.0                # rad/s²
  
  # Dynamic Window sampling
  vx_samples: 10                    # Samplear 10 velocidades lineales
  vtheta_samples: 20                # Samplear 20 velocidades angulares
  
  # Funciones de costo (tuning fino)
  path_distance_bias: 32.0          # Prioridad seguir global_path
  goal_distance_bias: 24.0          # Prioridad acercarse a meta
  occdist_scale: 0.01               # Penalización por proximidad a obstáculos
  
  # Horizonte de simulación
  sim_time: 1.5                     # segundos (predecir 1.5s adelante)
  sim_granularity: 0.05             # resolución temporal (50ms steps)
  
  # Tolerancias
  xy_goal_tolerance: 0.2            # metros (radio de aceptación meta)
  yaw_goal_tolerance: 0.1           # radianes (~6 grados)
  latch_xy_goal_tolerance: false    # Re-verificar tolerancia continuamente
```

**Archivo de referencia**: `mechdog_navigation/config/base_local_planner_params.yaml`

### Validación de Restricción No-Holonómica

**Test obligatorio antes de deployment**:
```bash
# Monitorear cmd_vel durante navegación
rostopic echo /move_base/cmd_vel

# Verificar: linear.y SIEMPRE debe ser 0.0
# Si linear.y ≠ 0 → CONFIGURACIÓN INCORRECTA, detener inmediatamente
```

**Herramienta de debug**:
```bash
# Visualizar ventanas dinámicas generadas por DWA
rosrun rqt_reconfigure rqt_reconfigure
# Navegar a /move_base/DWAPlannerROS
# Verificar: vy_samples = 1, max_vel_y = 0.0
```

---

## MÉTRICAS DE VALIDACIÓN SIM-TO-REAL

**Criterios de éxito** (antes de aceptar despliegue real):

| Métrica | Umbral Aceptable | Método de Medición |
|---------|------------------|---------------------|
| Desviación de trayectoria (RMSE) | < 10 cm | Comparar `/mechdog/odom` vs `/mechdog/global_path` |
| Ratio de mapa explorado | > 95% | `count(FREE)/count(TOTAL)` en `/mechdog/map` |
| Frecuencia de emergency_stop | < 5% del tiempo total | Monitor `/mechdog/emergency_stop` (True/False) |
| Tiempo de convergencia a meta | Δ < 15% entre sim y real | Timestamp inicio → timestamp llegada |
| Match de mapa sim vs real | IoU > 0.85 | Intersection over Union de occupancy grids |

**Herramienta**: `metrics_recorder` (ver sección Workflow).

---

## ANTI-PATTERNS (PROHIBIDOS)

1. **Hardcodear paths absolutos**:
   ❌ `map_file = "/home/user/maps/map.yaml"`
   ✅ `map_file = rospy.get_param('~map_file')`

2. **Time.sleep() en control loops**:
   ❌ `time.sleep(0.1)` (no determinista en real-time)
   ✅ `rospy.Rate(10).sleep()` (respeta ROS time)

3. **Asumir frecuencias fijas de sensores**:
   ❌ `assert len(scan.ranges) == 360`
   ✅ `ranges = scan.ranges; angle_increment = scan.angle_increment`

4. **Bypass del safe_controller**:
   ❌ Conectar `/move_base/cmd_vel` directamente al robot
   ✅ `move_base` → `/move_base/cmd_vel` → `safe_controller` → `/mechdog/cmd_vel_safe` → robot
   
5. **Configurar velocidad lateral en robot no-holonómico**:
   ❌ `max_vel_y: 0.5` en base_local_planner_params.yaml
   ✅ `max_vel_y: 0.0, vy_samples: 1` (restricción cinemática estricta)

6. **Lógica de física en CORE**:
   ❌ Simular fricción o fricción en `global_planner_node.py`
   ✅ Física SOLO en simulador o driver de hardware

---

## RESUMEN EJECUTIVO

**Lo que DEBE estar aislado en mechdog_sim/**:
- Carga de Gazebo/PyBullet
- Spawning de obstáculos
- Parámetros de ruido
- Plugins específicos del simulador

**Lo que DEBE estar aislado en mechdog_hw/**:
- Drivers de LIDAR, IMU, motores
- Calibración de sensores
- Lógica de bajo nivel (PWM, comunicación serie)

**Lo que DEBE ser portable (mechdog_navigation/)**:
- Stack ROS Navigation (move_base + DWAPlannerROS plugin)
- Configuración de costmaps y planners (YAML)
- Safe controller (capa adicional de seguridad física)
- Métricas y logging

**IMPORTANTE**: No reimplementar planning/control desde cero. Usar paquetes ROS battle-tested (`move_base`, `navfn`, `base_local_planner`) y tunear via parámetros.

**La interfaz entre capas son SOLO tópicos/servicios ROS estándar**.
