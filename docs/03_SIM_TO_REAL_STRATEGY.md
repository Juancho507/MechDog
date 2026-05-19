# MechDog — Sim-to-Real Strategy

## OBJETIVO
Garantizar que el código en `mechdog_navigation/` (CORE) funcione **sin modificaciones** tanto en simulación como en hardware real. Cualquier diferencia se resuelve mediante configuración externa (YAML, launch files, URDF).

---

## PRINCIPIOS FUNDAMENTALES

### P1: Hardware Abstraction via ROS Topics
**Regla**: El código CORE NUNCA interactúa directamente con hardware o simuladores.

**Enforcement**:
```python
# ❌ PROHIBIDO en mechdog_navigation/src/*
import pybullet as p
from gazebo_msgs.srv import SpawnModel
import serial  # para comunicación directa con LIDAR

# ✅ CORRECTO
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
```

**Razón**: Si el código CORE importa bibliotecas específicas de sim/hw, se rompe la portabilidad. ROS topics son el ÚNICO contrato.

---

### P2: Single Source of Truth (URDF)
**Regla**: Todas las propiedades físicas del robot (dimensiones, masa, posición de sensores) vienen del archivo `mechdog.urdf.xacro`.

**Variables que DEBEN estar en URDF**:
```xml
<!-- mechdog_description/urdf/mechdog.urdf.xacro -->
<xacro:property name="robot_length" value="0.5"/>  <!-- metros -->
<xacro:property name="robot_width" value="0.3"/>
<xacro:property name="robot_height" value="0.2"/>
<xacro:property name="robot_mass" value="15.0"/>   <!-- kg -->
<xacro:property name="lidar_offset_x" value="0.2"/> <!-- desde base_link -->
<xacro:property name="lidar_offset_z" value="0.15"/>
<xacro:property name="max_velocity" value="1.0"/>   <!-- m/s -->
<xacro:property name="max_angular_velocity" value="1.5"/> <!-- rad/s -->
```

**Acceso desde código CORE**:
```python
# mechdog_navigation/src/safe_controller_node.py
import rospy
import rospkg

# Leer URDF (parseado automáticamente por robot_state_publisher)
# Forma 1: Via parámetros ROS (recomendado)
robot_length = rospy.get_param('/robot_description_planning/robot_length', 0.5)

# Forma 2: Via TF lookups (para offsets de sensores)
tf_listener = tf.TransformListener()
try:
    (trans, rot) = tf_listener.lookupTransform('base_link', 'lidar_link', rospy.Time(0))
    lidar_offset_x = trans[0]
except tf.Exception:
    rospy.logerr("No se puede leer TF base_link->lidar_link")
```

**Prohibición**:
```python
# ❌ NUNCA hardcodear en código CORE
ROBOT_LENGTH = 0.5  # ESTO CAUSA SIM-TO-REAL GAP
```

---

### P3: Configuration via ROS Parameters
**Regla**: Constantes de algoritmos y tuning se definen en archivos YAML separados por entorno.

#### Estructura de Configuración

```
mechdog_navigation/config/
├── costmap_common_params.yaml       # Footprint, inflation, sensores
├── global_costmap_params.yaml       # Mapa estático, tamaño, resolución
├── local_costmap_params.yaml        # Rolling window, update frequency
├── base_local_planner_params.yaml   # DWA: velocidades, restricción no-holonómica
├── global_planner_params.yaml       # NavfnROS o GlobalPlanner config
├── move_base_params.yaml            # Controller frequency, recovery behaviors
├── safe_controller.yaml             # Parámetros adicionales de seguridad física
└── environments/
    ├── sim_clean.yaml               # Simulación sin ruido (debug)
    ├── sim_noisy.yaml               # Simulación con ruido (training)
    └── real_hw.yaml                 # Hardware real (deployment, overrides)
```

#### Ejemplo: safe_controller.yaml (Base)

```yaml
# mechdog_navigation/config/safe_controller.yaml
safe_controller:
  # Constantes físicas (iguales para sim y real)
  brake_deceleration: 2.0        # m/s² (depende de fricción y masa)
  collision_margin_base: 0.15    # metros (depende de URDF robot_width)
  
  # Tuning de seguridad (puede variar entre sim y real)
  emergency_stop_threshold: 0.8  # metros (distancia crítica)
  rotation_speed: 0.5            # rad/s (velocidad al buscar escape)
  clearance_required: 1.5        # metros (espacio libre mínimo)
```

#### Ejemplo: base_local_planner_params.yaml (CRÍTICO: Restricción No-Holonómica)

```yaml
# mechdog_navigation/config/base_local_planner_params.yaml
# CONFIGURACIÓN DWAPlannerROS PARA SISTEMA NO-HOLONÓMICO (unicycle-like)

DWAPlannerROS:
  # ═══════════════════════════════════════════════════════════════
  # RESTRICCIÓN CINEMÁTICA NO-HOLONÓMICA (OBLIGATORIO)
  # MechDog NO tiene desplazamiento lateral: solo vx y ωz
  # ═══════════════════════════════════════════════════════════════
  max_vel_y: 0.0                    # Sin movimiento lateral
  min_vel_y: 0.0
  acc_lim_y: 0.0
  vy_samples: 1                     # Solo samplear vy=0 (no explorar lateral)
  
  # Velocidades lineales (frontal/retroceso)
  max_vel_trans: 0.5                # m/s (ajustar según motor real)
  min_vel_trans: 0.1                # m/s (evitar stall)
  max_vel_x: 0.5
  min_vel_x: -0.2                   # Retroceso lento permitido
  acc_lim_x: 1.5                    # m/s²
  
  # Velocidades angulares (rotación yaw)
  max_vel_theta: 1.0                # rad/s
  min_vel_theta: 0.2
  acc_lim_theta: 2.0                # rad/s²
  
  # Dynamic Window Approach sampling
  vx_samples: 10                    # Discretización velocidad lineal
  vtheta_samples: 20                # Discretización velocidad angular
  
  # Funciones de costo (tuning fino)
  path_distance_bias: 32.0          # Peso: seguir global path
  goal_distance_bias: 24.0          # Peso: acercarse a meta
  occdist_scale: 0.01               # Penalización: proximidad obstáculos
  
  # Simulación predictiva
  sim_time: 1.5                     # Horizonte de predicción (segundos)
  sim_granularity: 0.05             # Resolución temporal (50ms)
  
  # Tolerancias de llegada
  xy_goal_tolerance: 0.2            # Radio aceptación (metros)
  yaw_goal_tolerance: 0.1           # Tolerancia orientación (rad ≈ 6°)
  latch_xy_goal_tolerance: false
```

#### Override para Hardware Real

```yaml
# mechdog_navigation/config/environments/real_hw.yaml

# Safe controller overrides
safe_controller:
  brake_deceleration: 1.8           # Real tiene más fricción que sim
  emergency_stop_threshold: 1.0     # Ser más conservador en hardware real
  collision_margin_base: 0.20       # +5cm extra por incertidumbre

# DWA overrides (si necesario)
DWAPlannerROS:
  max_vel_trans: 0.4                # Reducir velocidad máxima en hw real
  acc_lim_x: 1.2                    # Aceleración más conservadora
  sim_time: 1.8                     # Horizonte más largo (más seguro)
```

#### Carga en Launch File

```xml
<!-- mechdog_navigation/launch/navigation_stack.launch -->
<launch>
  <arg name="environment" default="sim_clean"/>  <!-- sim_clean | sim_noisy | real_hw -->
  
  <!-- Lanzar move_base con configuración estándar -->
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <!-- Costmaps -->
    <rosparam file="$(find mechdog_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find mechdog_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>
    <rosparam file="$(find mechdog_navigation)/config/global_costmap_params.yaml" command="load"/>
    <rosparam file="$(find mechdog_navigation)/config/local_costmap_params.yaml" command="load"/>
    
    <!-- Planners -->
    <rosparam file="$(find mechdog_navigation)/config/base_local_planner_params.yaml" command="load"/>
    <rosparam file="$(find mechdog_navigation)/config/global_planner_params.yaml" command="load"/>
    <rosparam file="$(find mechdog_navigation)/config/move_base_params.yaml" command="load"/>
    
    <!-- Remapping para tópicos estándar -->
    <remap from="cmd_vel" to="/move_base/cmd_vel"/>
    <remap from="odom" to="/mechdog/odom"/>
    <remap from="scan" to="/mechdog/sensor/scan"/>
  </node>
  
  <!-- Safe Controller (capa adicional de seguridad) -->
  <rosparam command="load" file="$(find mechdog_navigation)/config/safe_controller.yaml"/>
  <rosparam command="load" file="$(find mechdog_navigation)/config/environments/$(arg environment).yaml"/>
  <node pkg="mechdog_navigation" type="safe_controller_node.py" name="safe_controller_node" output="screen">
    <remap from="cmd_vel_in" to="/move_base/cmd_vel"/>
    <remap from="cmd_vel_out" to="/mechdog/cmd_vel_safe"/>
  </node>
</launch>
```

#### Acceso desde Código

```python
# mechdog_navigation/src/safe_controller_node.py
class SafeController:
    def __init__(self):
        # SIEMPRE con valores por defecto (fallback)
        self.brake_decel = rospy.get_param('~brake_deceleration', 2.0)
        self.emergency_thresh = rospy.get_param('~emergency_stop_threshold', 0.8)
        self.margin_base = rospy.get_param('~collision_margin_base', 0.15)
        
        rospy.loginfo(f"SafeController init: brake_decel={self.brake_decel}, "
                      f"emergency_thresh={self.emergency_thresh}")
```

**Ventaja**: Cambiar de sim a real es solo `roslaunch ... environment:=real_hw`. Código no cambia.

---

## VARIABLES CRÍTICAS PARA AISLAR

### Tabla de Aislamiento

| Variable | Ubicación CORRECTA | Ubicación PROHIBIDA | Razón |
|----------|-------------------|---------------------|-------|
| Dimensiones robot (largo, ancho) | URDF (`robot_length`, `robot_width`) | Código CORE | Cambia entre prototipos |
| Offset de sensores | URDF (TF `base_link→lidar_link`) | Hardcoded en código | Varía por montaje físico |
| Max velocidad del robot | URDF + config YAML | Código CORE | Limitado por motores reales |
| Fricción del suelo | Simulador (SDF/Gazebo) o config YAML | Código CORE | Desconocida en diseño |
| Ruido de sensores (σ) | `sim_noise_params.yaml` | Código CORE | Solo aplica en sim |
| Frecuencia de LIDAR | Driver del sensor (hw) o plugin Gazebo | Código CORE | Varía por modelo de sensor |
| Threshold de colisión | Config YAML (puede variar sim/real) | Código CORE | Ajustable por tolerancia |
| Algoritmo de planning | Paquetes ROS std (move_base, navfn, base_local_planner) | Config YAML | Usar plugins battle-tested, tunear via params |
| Parámetros DWA (velocidades, pesos) | Config YAML (base_local_planner_params.yaml) | Código CORE | Tuning específico por robot |
| Restricción cinemática (vy=0) | Config YAML (`max_vel_y: 0.0, vy_samples: 1`) | Código CORE | Enforcement no-holonómico |
| Goal position (x, y) | Launch file argument o ROS param | Código CORE | Específico de cada run |
| Map resolution | Config YAML (occupancy_grid) | Código CORE | Balance performance/precisión |
| PID gains (si aplica) | Config YAML (hw-specific) | Código CORE | Requiere tuning por robot |

---

## ESTRATEGIA DE RUIDO (Sim-to-Real Gap Mitigation)

### Problema
Simuladores como Gazebo/PyBullet generan sensores perfectos (sin ruido). El hardware real tiene:
- Ruido gaussiano en lecturas LIDAR (~2cm σ)
- Drift en odometría (~5cm por cada 10m recorridos)
- Latencia variable en comunicación (jitter)
- Deslizamiento de ruedas/patas (slip)

**Consecuencia**: Un algoritmo que funciona perfecto en sim puede fallar en real.

### Solución: Noise Injection Layer

#### Arquitectura

```
Gazebo (sensores limpios)
    │
    ├─ /gazebo/lidar_clean       (LaserScan sin ruido)
    ├─ /gazebo/odom_ground_truth (Odometry perfecta)
    │
    ▼
noise_injection_node.py (mechdog_sim/scripts/)
    │
    │ Aplica:
    │  - Ruido gaussiano a rangos LIDAR
    │  - Drift acumulativo en odom
    │  - Dropout aleatorio (pérdida de paquetes)
    │  - Jitter temporal
    │
    ▼
    ├─ /mechdog/sensor/scan      (LaserScan realista)
    └─ /mechdog/odom             (Odometry con drift)
        │
        ▼
    mechdog_navigation (CORE)
    [Consume datos como si fueran reales]
```

#### Implementación (noise_injection_node.py)

```python
#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import copy

class NoiseInjector:
    def __init__(self):
        # Leer parámetros de ruido desde YAML
        self.lidar_range_stddev = rospy.get_param('~lidar/range_stddev', 0.02)
        self.lidar_dropout_prob = rospy.get_param('~lidar/dropout_probability', 0.01)
        self.odom_linear_drift = rospy.get_param('~odom/linear_drift', 0.05)
        self.odom_angular_drift = rospy.get_param('~odom/angular_drift', 0.02)
        
        # Acumuladores de drift
        self.odom_drift_x = 0.0
        self.odom_drift_y = 0.0
        self.odom_drift_theta = 0.0
        
        # Subscribers (clean data)
        rospy.Subscriber('/gazebo/lidar_clean', LaserScan, self.lidar_callback)
        rospy.Subscriber('/gazebo/odom_ground_truth', Odometry, self.odom_callback)
        
        # Publishers (noisy data)
        self.lidar_pub = rospy.Publisher('/mechdog/sensor/scan', LaserScan, queue_size=10)
        self.odom_pub = rospy.Publisher('/mechdog/odom', Odometry, queue_size=10)
    
    def lidar_callback(self, msg):
        noisy_msg = copy.deepcopy(msg)
        
        # Aplicar ruido gaussiano a cada rango
        for i in range(len(noisy_msg.ranges)):
            if np.random.rand() < self.lidar_dropout_prob:
                noisy_msg.ranges[i] = float('inf')  # Dropout (no retorno)
            else:
                noise = np.random.normal(0, self.lidar_range_stddev)
                noisy_msg.ranges[i] += noise
                # Clamping a rango válido
                noisy_msg.ranges[i] = max(msg.range_min, 
                                           min(msg.range_max, noisy_msg.ranges[i]))
        
        self.lidar_pub.publish(noisy_msg)
    
    def odom_callback(self, msg):
        noisy_msg = copy.deepcopy(msg)
        
        # Drift acumulativo (proporcional a velocidad)
        dt = 0.02  # Asume 50Hz
        v_linear = msg.twist.twist.linear.x
        v_angular = msg.twist.twist.angular.z
        
        self.odom_drift_x += v_linear * self.odom_linear_drift * dt
        self.odom_drift_theta += v_angular * self.odom_angular_drift * dt
        
        # Aplicar drift a pose
        noisy_msg.pose.pose.position.x += self.odom_drift_x
        noisy_msg.pose.pose.position.y += self.odom_drift_y
        
        # Ajustar orientación (convertir quaternion, modificar, re-convertir)
        # (Simplificado: asumir orientación 2D)
        from tf.transformations import euler_from_quaternion, quaternion_from_euler
        (roll, pitch, yaw) = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        yaw += self.odom_drift_theta
        quat = quaternion_from_euler(roll, pitch, yaw)
        noisy_msg.pose.pose.orientation.x = quat[0]
        noisy_msg.pose.pose.orientation.y = quat[1]
        noisy_msg.pose.pose.orientation.z = quat[2]
        noisy_msg.pose.pose.orientation.w = quat[3]
        
        self.odom_pub.publish(noisy_msg)

if __name__ == '__main__':
    rospy.init_node('noise_injection_node')
    NoiseInjector()
    rospy.spin()
```

#### Configuración (sim_noise_params.yaml)

```yaml
# mechdog_sim/config/sim_noise_params.yaml
noise_injection_node:
  lidar:
    range_stddev: 0.02           # 2cm (LIDAR real típico)
    angle_stddev: 0.005          # 0.3 grados
    dropout_probability: 0.01    # 1% pérdida de lecturas
  
  odom:
    linear_drift: 0.05           # 5cm drift por 1m recorrido
    angular_drift: 0.02          # 1.1° drift por cada 180° girado
    update_rate_jitter: 0.1      # ±10% variación en frecuencia
```

#### Launch Integration

```xml
<!-- mechdog_sim/launch/gazebo_pathway.launch -->
<launch>
  <arg name="add_noise" default="false"/>
  
  <!-- Gazebo con sensores limpios -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find mechdog_sim)/worlds/pathway.world"/>
  </include>
  
  <!-- Spawn robot -->
  <node name="spawn_mechdog" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -model mechdog -param robot_description"/>
  
  <!-- Si add_noise=true, insertar capa de ruido -->
  <group if="$(arg add_noise)">
    <rosparam command="load" file="$(find mechdog_sim)/config/sim_noise_params.yaml"/>
    <node pkg="mechdog_sim" type="noise_injection_node.py" name="noise_injection_node"/>
  </group>
  
  <!-- Si add_noise=false, remapear directamente (bypass) -->
  <group unless="$(arg add_noise)">
    <node pkg="topic_tools" type="relay" name="lidar_relay"
          args="/gazebo/lidar_clean /mechdog/sensor/scan"/>
    <node pkg="topic_tools" type="relay" name="odom_relay"
          args="/gazebo/odom_ground_truth /mechdog/odom"/>
  </group>
</launch>
```

**Uso**:
- `roslaunch mechdog_sim gazebo_pathway.launch add_noise:=false` → Desarrollo (debug fácil)
- `roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true` → Training (realista)
- Hardware real → No hay noise_injection (datos ya son ruidosos naturalmente)

---

## VALIDACIÓN PRE-DEPLOYMENT

### Checklist Obligatorio Antes de Pasar a Hardware Real

#### 1. Verificar Portabilidad del Código CORE

```bash
# Buscar importaciones prohibidas
grep -r "import pybullet\|import gazebo\|import serial" mechdog_navigation/src/
# Output esperado: (vacío)

# Buscar hardcoded values
grep -r "ROBOT_LENGTH\|LIDAR_OFFSET\|MAX_VELOCITY" mechdog_navigation/src/ | grep -v "rospy.get_param"
# Output esperado: (vacío, todos deben usar rospy.get_param)
```

#### 2. Probar Mismo Launch File en Sim y Namespaced Sim

```bash
# Test 1: Sim normal
roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true
roslaunch mechdog_navigation navigation_stack.launch environment:=sim_noisy

# Test 2: Sim con namespace (simula multi-robot)
roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true robot_namespace:=mechdog_1
roslaunch mechdog_navigation navigation_stack.launch environment:=sim_noisy robot_namespace:=mechdog_1

# Si ambos funcionan → código CORE es portable
```

#### 3. Comparar Métricas Sim (con ruido) vs Sim (sin ruido)

```bash
# Run sin ruido
roslaunch mechdog_sim gazebo_pathway.launch add_noise:=false seed:=42
roslaunch mechdog_navigation navigation_stack.launch environment:=sim_clean
roslaunch mechdog_metrics record_run.launch output_dir:=~/exp/sim_clean

# Run con ruido (misma semilla)
roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true seed:=42
roslaunch mechdog_navigation navigation_stack.launch environment:=sim_noisy
roslaunch mechdog_metrics record_run.launch output_dir:=~/exp/sim_noisy

# Comparar
rosrun mechdog_metrics compare_runs.py \
    --baseline ~/exp/sim_clean \
    --test ~/exp/sim_noisy \
    --output ~/reports/noise_impact.json
```

**Criterio de éxito**: Métricas con ruido deben ser <20% peores que sin ruido (si es >20%, algoritmos no son robustos).

#### 4. Validar URDF en Hardware Real

```bash
# En robot real, verificar que TF tree coincide
roslaunch mechdog_hw hardware_drivers.launch
rosrun tf view_frames  # Genera frames.pdf

# Comparar con sim
roslaunch mechdog_sim gazebo_pathway.launch
rosrun tf view_frames

# Diff frames.pdf de sim vs real → deben ser idénticos (salvo timestamps)
```

#### 5. Test de Seguridad en Sim Antes de Real

```bash
# Escenario de estrés: obstáculo aparece súbitamente
roslaunch mechdog_sim stress_test_sudden_obstacle.launch
# Verificar:
# - emergency_stop se activa <200ms después de detección
# - Robot NUNCA colisiona (monitor collision en Gazebo)
# - Recovery exitoso (robot continúa tras evasión)

# Si pasa → Safe Controller es robusto para hardware real
```

---

## DEPLOYMENT WORKFLOW (Paso de Sim a Real)

### Fase 1: Desarrollo en Sim Limpia (1-2 semanas)
```
Objetivo: Implementar algoritmos, validar lógica básica
Comando: roslaunch mechdog_sim ... add_noise:=false
Criterio: Algoritmos convergen en 100% de los casos
```

### Fase 2: Training en Sim con Ruido (1 semana)
```
Objetivo: Refinar parámetros bajo condiciones realistas
Comando: roslaunch mechdog_sim ... add_noise:=true
Acciones:
  - Ajustar thresholds de safe_controller (via YAML)
  - Ajustar pesos de DWA (path_distance_bias, goal_distance_bias, occdist_scale) via base_local_planner_params.yaml
  - Correr 50+ runs con diferentes seeds
Criterio: Tasa de éxito >95% en 50 runs aleatorios
```

### Fase 3: Verificación de Portabilidad (2 días)
```
Objetivo: Garantizar que código CORE no depende de sim
Acciones:
  - Checklist de validación (ver sección anterior)
  - Code review de imports/hardcoded values
  - Test en namespace aislado
Criterio: Todos los checks pasan
```

### Fase 4: Calibración Física (1 día)
```
Objetivo: Medir parámetros reales del robot
Acciones:
  - Medir dimensiones exactas (largo, ancho, altura)
  - Medir offsets de sensores con cinta métrica
  - Determinar max_velocity empíricamente (test manual)
  - Medir brake_deceleration (rodar a velocidad fija, frenar, medir distancia)
Outputs:
  - Actualizar URDF con mediciones reales
  - Crear config/environments/real_hw.yaml con overrides
```

### Fase 5: First Run en Hardware (controlado)
```
Objetivo: Validar que funciona en real sin sorpresas
Setup:
  - Entorno controlado (espacio pequeño, sin obstáculos dinámicos)
  - Operador con botón de emergencia física (kill switch)
  - Goal cercano (2m de distancia)
Comando: roslaunch mechdog_hw hardware_drivers.launch
         roslaunch mechdog_navigation navigation_stack.launch environment:=real_hw goal_x:=2.0
Criterio:
  - Robot se mueve (no se queda estático)
  - Velocidades son razonables (no oscilaciones violentas)
  - Safe Controller activa correctamente ante obstáculos
Si falla: revisar logs, NO modificar código CORE, ajustar YAMLs
```

### Fase 6: Comparación de Métricas (evaluación)
```
Objetivo: Cuantificar sim-to-real gap
Acciones:
  - Correr mismo escenario en sim (con ruido) y real (5 repeticiones cada uno)
  - Medir: trajectory_deviation, map_iou, emergency_frequency, convergence_time
  - Calcular diferencias porcentuales
Criterio de éxito: Δ < 15% en todas las métricas
Si Δ > 15%:
  - Incrementar ruido en sim (ajustar sim_noise_params.yaml)
  - Ajustar parámetros en config/environments/real_hw.yaml
  - NO modificar mechdog_navigation/src/* (lógica debe ser robusta)
```

### Fase 7: Deployment Completo
```
Objetivo: Uso en escenarios reales (camino con obstáculos)
Criterio: Tasa de éxito >90% en 20 runs en entorno real
```

---

## DEBUGGING SIM-TO-REAL ISSUES

### Issue 1: "Funciona en sim, falla en real"

**Diagnóstico**:
1. Verificar que `add_noise:=true` en sim (puede estar enmascarando problema)
2. Comparar frecuencias de tópicos:
   ```bash
   rostopic hz /mechdog/sensor/scan  # Debe ser similar sim vs real
   ```
3. Grabar rosbag de ambos:
   ```bash
   rosbag record /mechdog/sensor/scan /mechdog/odom /mechdog/cmd_vel_safe
   ```
4. Visualizar en RViz side-by-side (sim y real)

**Causas comunes**:
- Frecuencia de LIDAR diferente (ajustar en driver de hw o plugin de sim)
- Offsets de sensores incorrectos (medir físicamente y actualizar URDF)
- Parámetros de safe_controller muy agresivos (ajustar `emergency_stop_threshold`)

---

### Issue 2: "Robot oscila / comportamiento errático"

**Diagnóstico**:
- Verificar que PID gains (si aplica) están en config YAML, no hardcoded
- Chequear latencia de control loop:
  ```bash
  rostopic delay /mechdog/cmd_vel_safe
  ```
  (Debe ser <100ms)

**Causas comunes**:
- Frecuencia de `DWAPlannerROS` muy alta para CPU del robot (ajustar `controller_frequency` en move_base_params.yaml a 5Hz)
- Aceleración máxima en safe_controller no coincide con física real (medir empíricamente)

---

### Issue 3: "Mapa construido es muy distinto sim vs real"

**Diagnóstico**:
1. Verificar TF tree:
   ```bash
   rosrun tf tf_echo map base_link
   ```
2. Comparar raw scans:
   ```bash
   rostopic echo /mechdog/sensor/scan | head -50
   ```

**Causas comunes**:
- Transform `map→odom` no se publica correctamente en hw (falta localization node)
- LIDAR en hardware tiene campo de visión diferente (FOV) al simulado (actualizar en URDF/Gazebo plugin)

---

## RESUMEN PARA FUTURAS IAs

### DO's (Recomendaciones)

✅ **Usar rospy.get_param() para TODA constante**
```python
threshold = rospy.get_param('~emergency_stop_threshold', 0.8)
```

✅ **Leer dimensiones de robot desde URDF vía TF**
```python
tf_listener.lookupTransform('base_link', 'lidar_link', rospy.Time(0))
```

✅ **Crear archivos YAML separados por entorno**
```
config/environments/sim_clean.yaml
config/environments/sim_noisy.yaml
config/environments/real_hw.yaml
```

✅ **Agregar ruido en sim antes de deployment**
```xml
<arg name="add_noise" default="true"/>
```

✅ **Validar portabilidad con grep**
```bash
grep -r "import pybullet" mechdog_navigation/src/  # Debe estar vacío
```

✅ **Medir parámetros físicos reales antes de actualizar URDF**
(Cinta métrica, cronómetro, báscula)

---

### DON'Ts (Prohibiciones)

❌ **NUNCA hardcodear dimensiones en código CORE**
```python
ROBOT_LENGTH = 0.5  # ¡NO!
```

❌ **NUNCA importar bibliotecas de sim/hw en código CORE**
```python
import pybullet  # ¡ROMPE PORTABILIDAD!
```

❌ **NUNCA modificar mechdog_navigation/ al pasar de sim a real**
(Solo cambiar launch files y YAMLs)

❌ **NUNCA asumir frecuencias fijas de sensores**
```python
assert len(scan.ranges) == 360  # Puede ser 720 en otro LIDAR
```

❌ **NUNCA poner lógica de física en CORE**
```python
# En mechdog_navigation/: ¡NO!
if surface_type == "grass":
    friction = 0.3
```

❌ **NUNCA probar en hardware sin antes probar en sim con ruido**

---

## MÉTRICAS DE ÉXITO SIM-TO-REAL

| Métrica | Objetivo | Medición |
|---------|----------|----------|
| **Code Portability** | 0 líneas modificadas en CORE | Diff mechdog_navigation/ (sim vs real) |
| **Topic Consistency** | 100% tópicos idénticos | `rostopic list` sim vs real |
| **TF Tree Match** | Frames idénticos | `rosrun tf view_frames` diff |
| **Trajectory Deviation** | Δ < 15% RMSE | compare_metrics.py |
| **Map Similarity** | IoU > 0.85 | compare_maps.py |
| **Success Rate** | >90% en 20 runs reales | manual log |
| **Emergency Frequency** | Δ < 20% entre sim y real | emergency_frequency.py |

**Criterio final**: Si todas las métricas pasan → Sim-to-Real exitoso. Si falla alguna → NO modificar CORE, ajustar configs/ruido.

---

## TEMPLATE DE COMMIT MESSAGE (para IAs)

Cuando modifiques código relacionado a Sim-to-Real, usa este formato:

```
[sim2real] <component>: <descripción>

Cambios:
- [mechdog_navigation/config/] Ajuste de threshold X para hardware real
- [mechdog_sim/config/] Incremento de ruido LIDAR a 3cm σ

Validación:
- ✓ grep -r "import pybullet" mechdog_navigation/ → vacío
- ✓ rostopic list sim vs real → 100% match
- ✓ trajectory_deviation: Δ=8.3% (dentro de umbral <15%)

Refs: docs/03_SIM_TO_REAL_STRATEGY.md §Validation
```

---

## CONCLUSIÓN

**Única regla crítica**: El directorio `mechdog_navigation/` es **sagrado**. NO debe contener NADA específico de simulación o hardware. Es el puente portable entre ambos mundos. Cualquier cambio de sim a real se hace EXCLUSIVAMENTE vía:
1. Launch files (remapping de tópicos)
2. YAML configs (parámetros)
3. URDF (propiedades físicas)

Si necesitas modificar `mechdog_navigation/src/*.py` al pasar de sim a real → **el diseño está roto**. Re-revisar esta documentación.
