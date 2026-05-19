# MechDog — AI Context (Ultra-Compact)

## SISTEMA
**Tipo**: Robot cuadrúpedo autónomo, sistema NO-HOLONÓMICO (unicycle-like: vx, ωz solamente, NO vy)  
**Middleware**: ROS 1 Noetic (Catkin, rospy, roscore) — NO ROS 2  
**Meta**: Paridad Sim-to-Real (mismo código CORE en simulación y hardware)

---

## ARQUITECTURA (4 paquetes ROS)

```
mechdog_sim/        → Gazebo/PyBullet + noise injection (AISLADO de CORE)
mechdog_hw/         → Drivers LIDAR/motores/IMU (AISLADO de CORE)
mechdog_navigation/ → CORE algorithms (PORTABLE 100%, NO deps de sim/hw)
mechdog_metrics/    → Rosbags + análisis sim vs real
```

---

## TOPOLOGÍA ROS

### Topics (Standard Interface)
```
/mechdog/sensor/scan                    sensor_msgs/LaserScan      20Hz   [sim/hw → move_base]
/mechdog/odom                           nav_msgs/Odometry          50Hz   [sim/hw → move_base]
/map                                    nav_msgs/OccupancyGrid     1Hz    [map_server → move_base]
/move_base/global_costmap/costmap       nav_msgs/OccupancyGrid     1Hz    [move_base internal]
/move_base/local_costmap/costmap        nav_msgs/OccupancyGrid     5Hz    [move_base internal]
/move_base/NavfnROS/plan                nav_msgs/Path              1Hz    [global planner]
/move_base/DWAPlannerROS/local_plan     nav_msgs/Path              10Hz   [local planner]
/move_base/cmd_vel                      geometry_msgs/Twist        10Hz   [DWA → safe_controller]
/mechdog/cmd_vel_safe                   geometry_msgs/Twist        20Hz   [safe_controller → robot]
/mechdog/emergency_stop                 std_msgs/Bool              event  [safe_controller]
```

### Nodes (CORE)
```
move_base (nodo principal)
  ├─ NavfnROS (global planner: Dijkstra sobre /map)
  ├─ DWAPlannerROS (local planner: NO-HOLONÓMICO max_vel_y=0.0, vy_samples=1)
  ├─ global_costmap (mapa estático + inflación)
  └─ local_costmap (rolling window + sensores dinámicos)

safe_controller_node → lee /move_base/cmd_vel → verifica → /mechdog/cmd_vel_safe
```

### TF Tree (Standard)
```
map → odom → base_footprint → base_link → lidar_link
```

---

## REGLAS CRÍTICAS (ENFORCEMENT)

### R1: Isolation Principle
**NUNCA** importar `pybullet`, `gazebo`, `serial` en `mechdog_navigation/src/`  
**Solo**: `rospy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf`

### R2: Configuration via YAML
**NUNCA** hardcodear: `ROBOT_LENGTH`, `MAX_VELOCITY`, `SENSOR_RANGE`  
**Siempre**: `rospy.get_param('~max_velocity', default)`

### R3: URDF as Single Source of Truth
Dimensiones, offsets de sensores, límites → `mechdog.urdf.xacro`  
Acceso vía TF: `tf_listener.lookupTransform('base_link', 'lidar_link')`

### R4: Topic Remapping Only
Diferencias sim/real se resuelven en launch files, NO en código:
```xml
<arg name="environment" default="sim_clean"/>  <!-- sim_clean | sim_noisy | real_hw -->
<rosparam file="$(find mechdog_navigation)/config/environments/$(arg environment).yaml"/>
```

### R5: Usar ROS Navigation Stack Estándar
NO reimplementar planning/control. Usar `move_base` + plugins `NavfnROS` + `DWAPlannerROS`. Tunear via YAML.

### R6: Restricción Cinemática No-Holonómica OBLIGATORIA
En `base_local_planner_params.yaml`:
```yaml
DWAPlannerROS:
  max_vel_y: 0.0      # SIN desplazamiento lateral
  vy_samples: 1       # Solo samplear vy=0
```

### R7: Safe Controller es Único Actuador
Robot SOLO lee `/mechdog/cmd_vel_safe` (nunca `/move_base/cmd_vel` directo)  
Pipeline: `move_base/DWA → /move_base/cmd_vel → safe_controller → /mechdog/cmd_vel_safe → robot`

---

## NOISE INJECTION (Sim-to-Real Gap Mitigation)

```python
# mechdog_sim/scripts/noise_injection_node.py
Gazebo → /gazebo/lidar_clean → [add gaussian noise] → /mechdog/sensor/scan
Gazebo → /gazebo/odom_gt     → [add drift]          → /mechdog/odom
```

**Parámetros** (`mechdog_sim/config/sim_noise_params.yaml`):
```yaml
lidar_range_stddev: 0.02      # 2cm std dev
odom_linear_drift: 0.05       # 5cm por 1m recorrido
dropout_probability: 0.01     # 1% pérdida de paquetes
```

**Comando**: `roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true`

---

## WORKFLOW (Identical Commands for Sim & Real)

### Simulación
```bash
# Terminal 1: Sim layer
roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true seed:=42

# Terminal 2: CORE - move_base + safe_controller (IDÉNTICO para sim y real)
roslaunch mechdog_navigation navigation_stack.launch environment:=sim_noisy

# Terminal 3: Enviar goal (usar send_goal.py o rviz)
rosrun mechdog_navigation send_goal.py --x 10.0 --y 0.0

# Terminal 4: Metrics
roslaunch mechdog_metrics record_run.launch output_dir:=~/rosbags/sim/run_001
```

### Hardware Real
```bash
# Terminal 1: HW layer (ÚNICO cambio)
roslaunch mechdog_hw hardware_drivers.launch

# Terminal 2: CORE - move_base + safe_controller (MISMO que sim)
roslaunch mechdog_navigation navigation_stack.launch environment:=real_hw

# Terminal 3: Enviar goal (MISMO que sim)
rosrun mechdog_navigation send_goal.py --x 10.0 --y 0.0

# Terminal 4: Metrics (MISMO que sim)
roslaunch mechdog_metrics record_run.launch output_dir:=~/rosbags/real/run_001
```

**Key insight**: Solo cambia Terminal 1 (sim vs hw). Terminales 2, 3, 4 son IDÉNTICOS.

---

## VALIDACIÓN PRE-DEPLOYMENT

```bash
# 1. No imports prohibidos
grep -r "import pybullet\|import gazebo\|import serial" mechdog_navigation/src/
# → Output esperado: (vacío)

# 2. No hardcoded values
grep -r "ROBOT_LENGTH\|MAX_VELOCITY" mechdog_navigation/src/ | grep -v "rospy.get_param"
# → Output esperado: (vacío)

# 3. TF tree idéntico
rosrun tf view_frames  # Comparar sim vs real
# → Debe ser idéntico (modulo timestamps)

# 4. Métricas sim vs real
rosrun mechdog_metrics compare_runs.py --sim ~/rosbags/sim --real ~/rosbags/real
# → Criterio: Δ < 15% en trajectory_deviation, map_iou, emergency_frequency
```

---

## MÉTRICAS DE ÉXITO

| Métrica | Umbral | Método |
|---------|--------|--------|
| Code portability | 0 líneas modificadas en CORE | `git diff` |
| Trajectory deviation (RMSE) | < 15 cm | `trajectory_deviation.py` |
| Map similarity (IoU) | > 0.85 | `compare_maps.py` |
| Emergency stop frequency Δ | < 20% | `emergency_frequency.py` |
| Success rate (real) | > 90% en 20 runs | manual log |

---

## ANTI-PATTERNS (PROHIBIDO)

❌ `from mechdog_sim.utils import X` en mechdog_navigation/  
❌ `ROBOT_LENGTH = 0.5` hardcoded en código CORE  
❌ Implementar DWA desde cero (usar `base_local_planner/DWAPlannerROS`)  
❌ Configurar `max_vel_y != 0.0` en robot no-holonómico  
❌ Publicar directamente a `/mechdog/cmd_vel_safe` desde move_base (bypass safe_controller)  
❌ `time.sleep(0.1)` en control loops (usar `rospy.Rate(10).sleep()`)  
❌ Modificar `mechdog_navigation/src/` al pasar de sim a real  
❌ Usar ROS 2 (el proyecto es ROS 1 Noetic exclusivamente)  

---

## DEPLOYMENT PHASES

1. **Dev en Sim Limpia** (add_noise:=false) → validar lógica básica
2. **Training en Sim con Ruido** (add_noise:=true) → refinar parámetros
3. **Validación Portabilidad** → checklist compliance
4. **Calibración Física** → medir dimensiones reales → actualizar URDF
5. **First Run HW** → entorno controlado, goal cercano
6. **Comparación Métricas** → Δ < 15% → ajustar YAML si necesario
7. **Deployment Completo** → escenarios reales

---

## VARIABLES CRÍTICAS (Tabla de Aislamiento)

| Variable | Ubicación CORRECTA | PROHIBIDO en |
|----------|-------------------|--------------|
| Dimensiones robot | URDF | Código CORE |
| Offset sensores | URDF (TF) | Código CORE |
| Max velocidad | URDF + YAML | Código CORE |
| Thresholds seguridad | YAML (puede variar sim/real) | Código CORE |
| Ruido sensores | `sim_noise_params.yaml` | Código CORE |
| Goal position | Launch arg o rosparam | Código CORE |
| Algoritmo planning (BFS/A*/DWA) | Código CORE | YAML |

---

## SAFE CONTROLLER (Seguridad Física)

**Lógica**:
1. Calcular polígono colisión dinámico: `margin = v²/(2*brake_decel)`
2. Proyectar LIDAR en `base_link`, chequear intersección con polígono
3. Si colisión inminente (dist < margin):
   - Publicar `/emergency_stop = True`
   - Enviar `cmd_vel_safe = (0, 0)` → freno
   - Rotar in-place hasta clearance > 1.5m
   - Retornar control a DWA
4. Si no hay escape (360° sin clearance):
   - Trigger re-planning global
   - Retroceder 0.5m y re-evaluar

**Frecuencia**: 20Hz (2× faster que local_planner para interceptar comandos)

---

## CONFIG FILES (Environment-Specific)

### Base (mechdog_navigation/config/safe_controller.yaml)
```yaml
brake_deceleration: 2.0
collision_margin_base: 0.15
emergency_stop_threshold: 0.8
```

### Override Real HW (config/environments/real_hw.yaml)
```yaml
safe_controller:
  brake_deceleration: 1.8      # Más fricción en real
  emergency_stop_threshold: 1.0  # Más conservador
  collision_margin_base: 0.20    # +5cm extra
```

**Carga**: `<rosparam file="$(find mechdog_navigation)/config/environments/$(arg environment).yaml"/>`

---

## RESUMEN ULTRA-COMPACTO (Token-Optimized)

```
STACK: ROS 1 Noetic (NO ROS 2) | Catkin | rospy | Gazebo | PyBullet | URDF
ARCH: mechdog_sim(isolated) | mechdog_hw(isolated) | mechdog_navigation(CORE,portable,move_base) | mechdog_metrics
CINEMÁTICA: NO-HOLONÓMICA (unicycle): vx, ωz solamente, vy=0 SIEMPRE (max_vel_y: 0.0, vy_samples: 1)
TOPICS: /move_base/cmd_vel, /mechdog/cmd_vel_safe, /mechdog/{sensor/scan,odom}, /map, /move_base/{global|local}_costmap
NODES: move_base{NavfnROS(Dijkstra)+DWAPlannerROS(vy=0)+costmaps} | safe_controller(brake+polygon)
TF: map→odom→base_footprint→base_link→lidar_link
RULES: NO custom planners (usar move_base) | NO import pybullet/gazebo | URDF=truth | YAML=config | safe_controller=único actuador | vy=0 enforcement
NOISE: noise_injection_node adds gaussian(σ=2cm) + drift(5cm/1m) + dropout(1%) en sim
WORKFLOW: roslaunch mechdog_{sim|hw} ... → roslaunch mechdog_navigation navigation_stack.launch environment:={sim_noisy|real_hw}
VALIDATION: grep prohibidos=vacío | TF tree=idéntico | Δ(trajectory,map,emergency)<15% | max_vel_y=0.0 verified
SUCCESS: 0 LOC modificadas CORE | IoU>0.85 | RMSE<15cm | success_rate>90%
ANTIPATTERNS: reimplementar DWA | max_vel_y!=0 | bypass safe_controller | hardcode en CORE | modificar CORE sim→real | usar ROS 2
```

**Tamaño**: 197 tokens (vs >5000 en docs completas) → 97% compresión, 100% contexto crítico.

---

**END AI_CONTEXT**  
Refs: [00_PROJECT_STRUCTURE.md](00_PROJECT_STRUCTURE.md), [01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md), [02_PROJECT_WORKFLOW.md](02_PROJECT_WORKFLOW.md), [03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md)
