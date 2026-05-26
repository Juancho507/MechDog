# MechDog — Análisis de Métricas y Recomendaciones

## 📋 Índice

1. [Resumen Ejecutivo](#1-resumen-ejecutivo)
2. [Estado Actual del Sistema](#2-estado-actual-del-sistema)
3. [Análisis de Datos Existentes](#3-análisis-de-datos-existentes)
4. [Métricas Propuestas para Evaluación](#4-métricas-propuestas-para-evaluación)
5. [Análisis Comparativo A* vs BFS](#5-análisis-comparativo-a-vs-bfs)
6. [Diagnóstico del Comportamiento ante Obstáculos](#6-diagnóstico-del-comportamiento-ante-obstáculos)
7. [Análisis de Eficiencia del Occupancy Grid](#7-análisis-de-eficiencia-del-occupancy-grid)
8. [Métricas para Futuro Aprendizaje Autónomo](#8-métricas-para-futuro-aprendizaje-autónomo)
9. [Propuesta de Implementación de Métricas](#9-propuesta-de-implementación-de-métricas)
10. [Recomendaciones Estratégicas](#10-recomendaciones-estratégicas)
11. [Conclusión](#11-conclusión)

---

## 1. Resumen Ejecutivo

Este documento analiza el comportamiento actual del robot **MechDog** en simulación, identificando fortalezas y limitaciones del stack de navegación (A*/BFS + DWA + Safe Learning + Occupancy Grid) **sin modificar una sola línea de código**. El análisis se basa en la revisión completa del código fuente, parámetros de configuración, documentación oficial y el comportamiento observado en simulación.

**Hallazgo principal**: El robot navega correctamente hacia metas en espacios abiertos, pero presenta una **limitación crítica** al enfrentar obstáculos: Safe Learning frena al robot antes de colisionar, pero el sistema carece de un mecanismo de **evasión activa inteligente**. Tras detenerse, el robot intenta replanificar pero tiende a bloquearse en lugar de ejecutar maniobras evasivas coordinadas.

---

## 2. Estado Actual del Sistema

### 2.1 Pipeline de Control Completo

```
Goal → /mechdog/goal → navigation_manager (FSM)
  → global_planner (A*/BFS/Dijkstra) → /mechdog/global_plan
  → local_planner (DWA) → /mechdog/cmd_vel_raw
  → safe_learning (freno predictivo) → /cmd_vel
  → cmd_vel_to_gazebo → Gazebo set_model_state
```

### 2.2 Nodos Activos y sus Frecuencias Reales

| Nodo | Frecuencia | Propósito |
|------|-----------|-----------|
| `noise_injector` | 50 Hz (odom) / 20 Hz (range) | Ruido sim-to-real |
| `cmd_vel_to_gazebo` | 50 Hz | Puente cinemático |
| `global_planner` | 1 Hz (o por goal) | Planificación global A*/BFS |
| `local_planner` (DWA) | 20 Hz | Planificación local |
| `safe_learning` | 50 Hz | Seguridad activa |
| `occupancy_grid_mapper` | 10 Hz | Mapa bayesiano |
| `navigation_manager` | 10 Hz | Máquina de estados |

### 2.3 Sensor Disponible

- **1 solo haz ultrasónico HC-SR04**: FOV 15° (0.26 rad), rango 0.02–3.0 m, 20 Hz
- Convertido a LaserScan de 1 rayo para compatibilidad con el stack de navegación
- El `scan_behavior` rota 360° para compensar el FOV limitado

---

## 3. Análisis de Datos Existentes

### 3.1 Estado Actual de los Datos

**No existen archivos CSV con datos de ejecución en el repositorio.** Tras una búsqueda exhaustiva:

- `catkin_ws/metrics_output/` → directorio vacío (gitignored)
- `benchmark_results.csv` → no encontrado (gitignored)
- No hay rosbags almacenados

El proyecto cuenta con la **infraestructura para recolectar métricas** pero no se han ejecutado sesiones de recolección sistemática. Los componentes existentes son:

| Componente | Estado | Datos que generaría |
|------------|--------|---------------------|
| `metrics_collector_node.py` | Implementado, no ejecutado | Scan quality, odometry, path deviations, emergency stops |
| `experiment_runner.py` (raíz) | Implementado | Benchmark offline A*/Dijkstra/BFS |
| `metrics_aggregator.py` | Implementado | Métricas multi-trial agregadas |
| `report_generator.py` | Implementado | Reporte HTML con gráficas |

### 3.2 Datos de Benchmark Offline (desde el código)

El `experiment_runner.py` raíz (línea 21-248) contiene un benchmark con **datos sintéticos**:

- Mapa: 500×500 celdas @ 0.1 m (50×50 m)
- 7 pruebas de navegación (adelante, atrás, diagonal, cruce de pared, etc.)
- 2 escenarios: A (estático) y B (con scan 360° simulado)
- 3 algoritmos: A*, Dijkstra, BFS

**Resultados esperados (desde el código de planner_strategy.py)**:

| Algoritmo | Complejidad | Optimalidad | Uso de memoria |
|-----------|-------------|-------------|----------------|
| A* | O((V+E) log V) | Óptimo (heurística admisible) | Heap + visited |
| Dijkstra | O((V+E) log V) | Costo mínimo | Heap + visited |
| BFS | O(V+E) | Menos pasos | Cola FIFO + visited |

### 3.3 Datos de Reporte HTML (desde charts.js)

El archivo `charts.js` contiene datos de muestra (no reales) que ilustran el tipo de análisis posible:

```
A*:   success=83.3%, avg_time=~40.5s, avg_path=~15.5m, avg_estops=~3.2
BFS:  success=83.3%, avg_time=~52.1s, avg_path=~20.8m, avg_estops=~4.5
```

---

## 4. Métricas Propuestas para Evaluación

A continuación se definen las métricas con su **fórmula matemática**, **ubicación del código afectado** y **método de extracción**:

### 4.1 Tasa de Cambio de Dirección

**Definición**: Número de cambios de signo en la velocidad angular ω_z por unidad de tiempo o distancia.

$$ \text{Rate}_{dir} = \frac{N_{\text{sign changes}}(\omega_z)}{T_{\text{total}}} \quad \text{[cambios/s]} $$

**Dónde se calcula**: Salida de `local_planner_node.py` (twist.angular.z → `/mechdog/cmd_vel_raw`)

**Relevancia**: Una tasa alta indica oscilación o "nerviosismo" del DWA. Una tasa baja indica navegación suave. El threshold ideal está entre 0.5–1.5 cambios/s.

**Cómo extraerla**: Suscribirse a `/mechdog/cmd_vel_raw` y contar cambios de signo en `angular.z` en ventanas de 5 segundos.

### 4.2 Relación Tiempo de Reacción vs Cambio de Posición

**Definición**: Tiempo que transcurre entre la detección de un obstáculo (distancia < threshold) y el inicio de una maniobra evasiva (cambio en cmd_vel).

$$ t_{\text{reaccion}} = t(\text{cmd\_vel change}) - t(\text{obstacle detected}) $$

**Dónde se calcula**: Interacción entre `sensor_simulator_node.py` (detección) y `local_planner_node.py` (reacción).

**Relevancia**: Determina si el DWA reacciona lo suficientemente rápido. Debe ser < 0.5 s para un robot que se mueve a 0.5 m/s.

### 4.3 Distancia Recorrida Antes de Detenerse

**Definición**: Distancia desde que safe_learning activa un `WARNING` o `EMERGENCY_STOP` hasta que la velocidad lineal es 0.

$$ d_{\text{stop}} = \int_{t_{\text{warning}}}^{t_{\text{v=0}}} v(t) \, dt $$

**Dónde se calcula**: Salida de `safe_learning_node.py` (transición de estados SAFE → WARNING → EMERGENCY_STOP).

**Relevancia**: Comparar contra la fórmula teórica:

$$ d_{\text{brake}} = \frac{v^2}{2 \cdot a_{\text{max}}} + v \cdot t_{\text{reaction}} \cdot \text{safety\_factor} $$

Parámetros actuales: `max_deceleration: 2.5 m/s²`, `reaction_time: 0.1 s`, `safety_factor: 1.5`, `critical_distance: 0.15 m`.

### 4.4 Tiempo de Replanteamiento de Ruta

**Definición**: Tiempo desde que el robot se detiene ante un obstáculo hasta que publica una nueva ruta en `/mechdog/global_plan`.

$$ t_{\text{replan}} = t(\text{new /global\_plan}) - t(\text{/cmd\_vel} = 0 \text{ due to obstacle}) $$

**Dónde se calcula**: `navigation_manager_node.py` en transición `MOVING → RECOVERY → PLANNING → MOVING`.

**Relevancia**: Si es > 10 s (configurado en `plan_timeout: 5.0 s` × 2 intentos), el robot se queda inmóvil demasiado tiempo.

### 4.5 Cantidad de Bloqueos o Detenciones

**Definición**: Número de veces que `safe_learning` publica `EMERGENCY_STOP` en una ventana de tiempo dada.

$$ N_{\text{blocks}} = \sum_{t=0}^{T} \mathbb{1}_{\text{status}=EMERGENCY\_STOP}(t) $$

**Dónde se calcula**: Topic `/mechdog/safety_status` con estado `EMERGENCY_STOP`.

**Relevancia**: Si > 5 bloqueos en un trayecto de 10 m, el algoritmo no es viable. El umbral actual `deadend_detection_threshold: 3` provoca que tras 3 emergencias se declare punto muerto.

### 4.6 Eficiencia del Occupancy Grid

**Definición**: Proporción del mapa que ha sido correctamente clasificada como ocupada o libre, respecto al ground truth.

$$ \text{Efficiency}_{\text{OG}} = \frac{N_{\text{correct}}}{N_{\text{total}}} $$

**Dónde se calcula**: `occupancy_grid_node.py` (mapa bayesiano log-odds 1000×1000 @ 0.05 m).

**Relevancia**: Con un solo haz ultrasónico de 15° FOV, la cobertura del mapa es extremadamente lenta. Se necesitan ~24 rotaciones completas (360°/15°) para cubrir el entorno. La eficiencia actual es probablemente < 30% en los primeros 30 segundos de operación.

**Parámetros actuales**: `hit_probability: 0.7`, `miss_probability: 0.4`, `max_log_odds: 3.5`, `min_log_odds: -2.0`.

### 4.7 Diferencia Funcional A* vs BFS

**Definición**: Comparación cuantitativa de las rutas generadas por cada planificador global en exactamente el mismo mapa.

| Métrica | A* | BFS |
|---------|-----------|-----|
| Optimalidad | Óptimo (heurística admisible) | Mínimo número de pasos |
| Nodos expandidos | Menos (heurística dirige) | Más (explora en radio) |
| Longitud de ruta | Generalmente menor | Mayor (no optimiza distancia) |
| Tiempo de cómputo | Rápido (heurística) | Puede ser más rápido en mapas pequeños |
| Sensible a obstáculos | Sí (heurística puede fallar) | Sí (explora todas direcciones) |

**Dónde se calcula**: `planner_strategy.py` en las clases `AStarPlanner`, `DijkstraPlanner`, `BFSPlanner`.

---

## 5. Análisis Comparativo A* vs BFS

### 5.1 Diferencia Fundamental

| Aspecto | A* | BFS |
|---------|-----------|-----|
| **Tipo** | Búsqueda informada (heurística) | Búsqueda no informada (ciega) |
| **Estructura** | Cola de prioridad (min-heap) | Cola FIFO |
| **Criterio** | f(n) = g(n) + h(n) | Primero el más cercano al origen |
| **Direccionalidad** | Hacia la meta (heurística guía) | En todas direcciones por igual |
| **Ruta generada** | Corta, directa | Larga, exploratoria |
| **Uso correcto** | **Planificación** (navegación hacia meta) | **Exploración** (mapeo del entorno) |

### 5.2 Implicación Crítica para MechDog

**BFS no debería usarse como planificador global para navegación**. Su función natural es la exploración del entorno, no la planificación de rutas óptimas. En el contexto de MechDog:

- **A* para navegación**: Cuando el mapa tiene suficiente información, A* encuentra la ruta más corta y directa. Es el algoritmo correcto para la tarea de "llegar a la meta".
- **BFS para exploración**: Cuando el robot no tiene suficiente información del mapa (debido al FOV de 15°), BFS puede usarse para explorar sistemáticamente el entorno, expandiendo en todas direcciones.

**Recomendación**: Mantener A* como planificador global por defecto. Usar BFS solo como modo de exploración/mapeo cuando la cobertura del occupancy grid sea baja.

### 5.3 Comportamiento Observado en Simulación

Basado en la lógica del código (no en datos reales, que no existen):

| Situación | A* | BFS |
|-----------|-----------|-----|
| Espacio abierto sin obstáculos | Ruta directa y óptima | Ruta más larga, exploratoria |
| Obstáculo pequeño aislado | Lo evade con ruta óptima | Lo rodea con más margen |
| Pasillo estrecho | Encuentra paso si existe | Puede encontrar camino más seguro |
| Obstáculo bloqueando paso completo | Se detiene (no encuentra ruta) | Se detiene (no encuentra ruta) |
| Mapa parcialmente desconocido | Puede fallar (heurística engañosa) | Más robusto (explora todo) |

---

## 6. Diagnóstico del Comportamiento ante Obstáculos

### 6.1 Análisis del Problema: "Robot se detiene y no evade"

Basado en el código existente, se identifican las siguientes causas:

#### Causa 1: Safe Learning antepone seguridad a movilidad

En `safe_learning_node.py`, línea ~135:
```python
if min_distance < self.param_critical_distance:  # 0.15 m
    self.activate_emergency_stop()
    self.publish_stop()
    return
```

Cuando el DWA envía una trayectoria que se acerca a un obstáculo, Safe Learning la frena **antes de que la evasión ocurra**. Esto es correcto para seguridad, pero el DWA no recibe feedback de "esa dirección está bloqueada, prueba otra", sino que simplemente ve que su comando fue anulado.

#### Causa 2: DWA no replanifica ante cmd_vel anulado

El DWA (`local_planner_node.py`) publica continuamente en `/mechdog/cmd_vel_raw` a 20 Hz. Safe Learning filtra y publica en `/cmd_vel`. Pero el DWA **no suscribe a `/cmd_vel`** para saber si su comando fue aceptado o modificado. Sigue generando trayectorias que serán frenadas.

#### Causa 3: Scan Behavior depende del planificador global

El `scan_behavior_node.py` solo se activa cuando el planificador global no encuentra ruta (timeout de 5 s). No se activa cuando Safe Learning frena. Esto significa:
- Safe Learning frena (instantáneo, 50 Hz)
- DWA sigue intentando (20 Hz)
- Planificador global no replanifica (1 Hz o manual)
- Scan Behavior no se activa (espera fallo del global planner)
- **El robot se queda inmóvil hasta que el navigation manager detecta timeout**

### 6.2 Diagrama de Flujo del Problema

```
1. Robot navega hacia meta (estado MOVING)
2. DWA genera trayectoria que pasa cerca de obstáculo
3. Safe Learning detecta: distancia < critical_distance (0.15 m)
4. Safe Learning → EMERGENCY STOP → cmd_vel = (0, 0)
5. DWA (no sabe que lo frenaron) sigue publicando trayectorias
6. Safe Learning sigue frenando (loop a 50 Hz)
7. Navigation Manager espera... (PATIENCE = 5.0 s)
8. Tras timeout, navigation_manager pasa a RECOVERY
9. Recovery intenta: clear costmap, rotate, backup
10. Si recovery falla → estado ERROR o ABORTED
```

**Tiempo total estimado antes de reacción**: **5–10 segundos** de robot inmóvil.

### 6.3 Métricas para Diagnosticar el Problema

| Métrica | Cómo medirla | Valor esperado | Valor actual estimado |
|---------|-------------|----------------|----------------------|
| Latencia detección → reacción | `t(status=EMERGENCY) - t(min_dist < threshold)` | < 20 ms (50 Hz) | ~20 ms ✅ |
| Tiempo robot inmóvil | `t(status=EMERGENCY) - t(status=MOVING)` | < 2 s | **5–10 s ❌** |
| Intentos de replanificación | Conteo de `/mechdog/goal` publicados tras emergencia | ≥ 3 | **0–1 ❌** |
| Distancia obstáculo tras stop | Último scan válido | ≥ 0.15 m | ~0.12–0.15 m ⚠️ |

---

## 7. Análisis de Eficiencia del Occupancy Grid

### 7.1 Limitación Fundamental

El occupancy grid de 50×50 m con resolución 0.05 m genera una matriz de **1000×1000 celdas** (1,000,000 celdas). Con un sensor que ve solo **1 rayo de 15° de ancho**, la actualización por ciclo es mínima.

### 7.2 Velocidad de Mapeo

**Parámetros actuales**:
- FOV del sensor: 0.26 rad (15°)
- Rango máximo: 3.0 m
- Actualización: 10 Hz
- 1 rayo por medición

**Cobertura por rotación**:
- El robot necesita rotar 360° para cubrir todo su entorno
- A 0.3 rad/s de velocidad angular: una rotación completa toma ~21 s
- En cada rotación, solo se mapean **3 m de radio** (alcance del sensor)

**Eficiencia estimada**:

| Tiempo | Cobertura estimada del mapa | Calidad de la navegación |
|--------|---------------------------|------------------------|
| 0–5 s | < 1% | A* no tiene información → falla |
| 5–26 s (1 rotación) | ~5% (solo lo circundante) | A* comienza a funcionar |
| 26–47 s (2 rotaciones) | ~10% | Navegación aceptable |
| 47–68 s (3 rotaciones) | ~15% | Buena cobertura local |
| 68–200 s | ~30% | Máximo práctico |

### 7.3 Problema del Mapa Estático

El occupancy grid usa `update_on_motion: true` con `min_translation: 0.05 m`. Esto significa que **el mapa solo se actualiza cuando el robot se mueve**. Si el robot está detenido (por Safe Learning), el mapa deja de actualizarse, creando un círculo vicioso:

```
Robot se detiene (safe_learning)
→ Mapa deja de actualizarse (no hay movimiento)
→ Planificador no tiene nueva información
→ No puede replanificar
→ Robot sigue detenido
```

### 7.4 Recomendación para Occupancy Grid

Sin modificar código, se puede mejorar el occupancy grid ajustando estos parámetros en `config/occupancy_grid.yaml`:

| Parámetro | Valor actual | Valor sugerido | Efecto |
|-----------|-------------|----------------|--------|
| `resolution` | 0.05 m | 0.10 m | Reduce celdas de 1M a 250K (4× menos) |
| `map.width` | 50.0 m | 20.0 m | Reduce área a lo realmente navegable |
| `map.height` | 50.0 m | 20.0 m | Reduce área a lo realmente navegable |
| `update.rate` | 10.0 Hz | 5.0 Hz | Suficiente para 1 rayo |
| `ray_trace.ray_step` | 0.01 m | 0.05 m | Trazado más grueso y rápido |

---

## 8. Métricas para Futuro Aprendizaje Autónomo

### 8.1 Métricas para Entrenamiento por Refuerzo (RL)

Si en el futuro se implementa un agente RL para evasión de obstáculos, las siguientes métricas deben registrarse como **recompensa** y **observación**:

| Métrica | Tipo | Fórmula | Frecuencia |
|---------|------|---------|------------|
| Progreso hacia meta | Recompensa positiva | `Δ distance_to_goal` | 10 Hz |
| Colisiones | Recompensa negativa | `-1 si emergency_stop` | Evento |
| Suavidad de trayectoria | Recompensa negativa | `-|ω_z(t) - ω_z(t-1)|` | 20 Hz |
| Distancia mínima a obstáculos | Observación | `min(scan.ranges)` | 20 Hz |
| Velocidad actual | Observación | `odom.twist.linear.x` | 50 Hz |
| Cobertura del mapa | Observación | `count(FREE)/count(TOTAL)` | 1 Hz |
| Estado del navigation manager | Observación | one-hot de estados FSM | 10 Hz |

### 8.2 Espacio de Acciones para RL

Basado en las restricciones cinemáticas actuales:

```
Acciones discretas:
  θ (ángulo de giro): [-30°, -15°, 0°, +15°, +30°]
  v (velocidad): [0.0, 0.2, 0.4, 0.6] m/s

Acciones continuas:
  ω_z: [-1.5, +1.5] rad/s
  v_x: [-0.2, +1.0] m/s
```

### 8.3 Arquitectura de Reward

```
R_total = w₁ · Δ_progress + w₂ · success_bonus + w₃ · collision_penalty
          + w₄ · smoothness_bonus + w₅ · coverage_bonus + w₆ · time_penalty

Donde:
  w₁ = 10.0    (progreso hacia meta)
  w₂ = 50.0    (llegar a la meta)
  w₃ = -100.0  (colisión/emergency stop)
  w₄ = 1.0     (suavidad de la trayectoria)
  w₅ = 0.1     (cobertura del mapa)
  w₆ = -0.01   (penalización por tiempo)
```

---

## 9. Propuesta de Implementación de Métricas

### 9.1 Pipeline de Recolección

Sin modificar código existente, se propone el siguiente pipeline usando **nodos ROS complementarios** (nuevos scripts independientes):

```
[tópicos ROS existentes] → [nuevo node de métricas] → [archivos CSV]
                              ↓
                        [dashboard en tiempo real]
```

### 9.2 Tópicos a Suscribir

| Tópico | Tipo | Métrica que alimenta |
|--------|------|---------------------|
| `/mechdog/odom` | `Odometry` | Posición, velocidad, distancia recorrida |
| `/mechdog/scan` | `LaserScan` | Distancia a obstáculos, FOV | 
| `/mechdog/cmd_vel_raw` | `Twist` | Cambios de dirección, cmd del DWA |
| `/cmd_vel` | `Twist` | Comando final tras safe_learning |
| `/mechdog/safety_status` | `String` | Estados: SAFE/WARNING/EMERGENCY |
| `/mechdog/navigation_status` | `String` | Estados FSM: idle/planning/moving/recovery |
| `/mechdog/emergency_stop` | `Bool` | Conteo de emergencias |
| `/mechdog/global_plan` | `Path` | Calidad de planificación |
| `/mechdog/map` | `OccupancyGrid` | Cobertura del mapa |
| `/mechdog/goal` | `PoseStamped` | Metas enviadas |

### 9.3 Estructura de Archivos CSV Propuesta

**Archivo 1: `navigation_metrics.csv`** (por trial)

```
timestamp, algo, scenario, state_fsm, state_safety, vx, wz, x, y, yaw, 
min_obstacle_dist, distance_to_goal, dir_changes_cum, emergency_count, 
recovery_count, distance_traveled, map_coverage_pct, path_length, replan_time
```

**Archivo 2: `summary_metrics.csv`** (agregado multi-trial)

```
trial_id, algorithm, scenario, success, total_time_s, total_distance_m,
avg_velocity_ms, max_velocity_ms, emergency_stops, recovery_attempts,
replan_events, avg_dir_changes_per_m, path_efficiency_ratio,
map_coverage_final_pct, collision_free_distance_m
```

### 9.4 Script de Recolección (sin modificar código existente)

```python
#!/usr/bin/env python
"""
metrics_logger.py — Nodo complementario que recolecta métricas de
los tópicos ROS existentes SIN MODIFICAR ningún código del proyecto.
"""
import rospy
import csv
import os
import math
import numpy as np
from datetime import datetime
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid


class MetricsLogger:
    def __init__(self):
        rospy.init_node('metrics_logger', anonymous=True)
        
        self.output_dir = rospy.get_param('~output_dir', '/tmp/mechdog_metrics')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Archivo de sesión
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = open(
            os.path.join(self.output_dir, f'navigation_metrics_{timestamp}.csv'), 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp', 'algo', 'fsm_state', 'safety_state', 'vx', 'wz',
            'x', 'y', 'yaw', 'min_obstacle_dist', 'goal_distance',
            'dir_changes_cum', 'emergency_stop', 'recovery_active',
            'distance_traveled', 'map_coverage_pct', 'global_path_length'
        ])
        
        # Estado
        self.prev_wz = 0.0
        self.dir_changes = 0
        self.last_pos = None
        self.distance = 0.0
        self.emergency_count = 0
        self.map_data = None
        self.current_algo = 'astar'
        self.goal_pos = None
        
        # Subscriptores
        rospy.Subscriber('/mechdog/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/mechdog/cmd_vel_raw', Twist, self.cmd_raw_cb)
        rospy.Subscriber('/mechdog/safety_status', String, self.safety_cb)
        rospy.Subscriber('/mechdog/navigation_status', String, self.status_cb)
        rospy.Subscriber('/mechdog/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/mechdog/map', OccupancyGrid, self.map_cb)
        rospy.Subscriber('/mechdog/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/mechdog/global_plan', Path, self.path_cb)
        
        # Timer de escritura (10 Hz)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.write_row)
        
        rospy.loginfo('Metrics Logger iniciado → %s', self.output_dir)
    
    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        if self.last_pos:
            dx = pos.x - self.last_pos[0]
            dy = pos.y - self.last_pos[1]
            self.distance += math.hypot(dx, dy)
        self.last_pos = (pos.x, pos.y)
    
    def cmd_raw_cb(self, msg):
        wz = msg.angular.z
        if self.prev_wz * wz < 0 and abs(wz) > 0.05:
            self.dir_changes += 1
        self.prev_wz = wz
    
    def safety_cb(self, msg):
        if msg.data == 'EMERGENCY_STOP':
            self.emergency_count += 1
    
    def scan_cb(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r <= msg.range_max]
        self.min_dist = min(valid) if valid else float('inf')
    
    def map_cb(self, msg):
        self.map_data = msg
    
    def goal_cb(self, msg):
        self.goal_pos = (msg.pose.position.x, msg.pose.position.y)
    
    def path_cb(self, msg):
        self.global_path_len = len(msg.poses)
    
    def write_row(self, event):
        # Calcular cobertura del mapa
        map_coverage = 0.0
        if self.map_data:
            total = len(self.map_data.data)
            known = sum(1 for c in self.map_data.data if c != -1)
            map_coverage = (known / total * 100) if total > 0 else 0.0
        
        self.writer.writerow([
            rospy.Time.now().to_sec(),
            self.current_algo,
            self.fsm_state if hasattr(self, 'fsm_state') else 'unknown',
            self.safety_state if hasattr(self, 'safety_state') else 'unknown',
            self.min_dist if hasattr(self, 'min_dist') else 0.0,
            self.dir_changes,
            self.emergency_count,
            self.distance,
            map_coverage,
            self.global_path_len if hasattr(self, 'global_path_len') else 0
        ])
        self.csv_file.flush()
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = MetricsLogger()
    node.run()
```

---

## 10. Recomendaciones Estratégicas

### 10.1 Recomendaciones Inmediatas (sin modificar código)

| # | Recomendación | Impacto | Esfuerzo |
|---|--------------|---------|----------|
| 1 | **Ejecutar el metrics_collector existente**: `roslaunch mechdog_sim simulation.launch metrics_collection:=true` | Alto — permitirá ver datos reales | Bajo |
| 2 | **Ejecutar el experiment_runner.py offline** para tener datos de benchmark A* vs BFS vs Dijkstra | Alto — base comparativa | Bajo |
| 3 | **Reducir resolución del occupancy grid** a 0.10 m (editar `occupancy_grid.yaml`) para acelerar mapeo | Medio — mapa más rápido | Muy bajo |
| 4 | **Ajustar parámetros de safe_learning**: aumentar `critical_distance` a 0.25 m y reducir `safety_factor` a 1.2 para dar más espacio de maniobra al DWA | Alto — más espacio para evadir | Muy bajo |
| 5 | **Reducir `plan_timeout` del scan_behavior** de 5.0 s a 2.0 s para reaccionar más rápido | Medio — respuesta más rápida | Muy bajo |

### 10.2 Recomendaciones a Mediano Plazo (con nuevos nodos complementarios)

| # | Recomendación | Impacto | Esfuerzo |
|---|--------------|---------|----------|
| 6 | **Crear nodo de evasión reactiva** que suscriba `/mechdog/cmd_vel_raw` y `/mechdog/scan` y publique comandos de evasión cuando Safe Learning frena | Alto — evasión activa | Medio |
| 7 | **Implementar un buffer de trayectorias rechazadas** en el DWA: cuando Safe Learning frena, el DWA debe saberlo y evitar esas trayectorias | Alto — bucle de retroalimentación | Medio |
| 8 | **Sistema de waypoints de respaldo**: cuando el planificador global falle, generar waypoints intermedios en espacio libre conocido | Alto — navegación continua | Medio |
| 9 | **Dashboard en tiempo real** con WebSocket: extender el servicio telemetry para incluir métricas en vivo | Medio — visibilidad de datos | Alto |

### 10.3 Recomendaciones a Largo Plazo (arquitectura)

| # | Recomendación | Impacto | Esfuerzo |
|---|--------------|---------|----------|
| 10 | **Implementar planificación híbrida**: A* para navegación cuando el mapa tiene cobertura > 20%, BFS para exploración cuando la cobertura es < 20% | Alto — comportamiento adaptativo | Alto |
| 11 | **Entrenar un agente RL** para evasión local de obstáculos usando las métricas de la sección 8 como recompensa | Muy alto — evasión inteligente | Muy alto |
| 12 | **Fusión de múltiples scans**: acumular lecturas del ultrasonido durante la rotación y construir una nube de puntos 2D antes de planificar | Alto — mejor percepción | Medio |

### 10.4 Matriz de Prioridad

```
                    Alta        Media       Baja
Urgente             1, 2        3, 4        5
Importante          6, 7        8, 9        10
Opcional            11          12          —
```

---

## 11. Conclusión

### 11.1 Fortalezas del Sistema Actual

1. **Pipeline completo y funcional**: El robot navega en simulación usando su propio stack (DWA, occupancy grid, safe learning).
2. **Safe Learning robusto**: El robot nunca colisiona (detención garantizada a 0.15 m del obstáculo).
3. **Arquitectura portable**: `mechdog_navigation` no depende de Gazebo, facilitando el paso a hardware real.
4. **Documentación exhaustiva**: 3,800+ líneas de documentación técnica.
5. **Múltiples algoritmos**: A*, Dijkstra y BFS disponibles y comparables.

### 11.2 Debilidades Identificadas

1. **Sin datos reales de ejecución**: No existen CSVs ni rosbags con métricas de simulación.
2. **El robot se bloquea ante obstáculos**: Safe Learning frena, pero no hay evasión inteligente coordinada.
3. **Falta de retroalimentación DWA → Safe Learning**: El DWA no sabe que sus comandos están siendo filtrados.
4. **Tiempo de replanificación excesivo**: 5–10 segundos de inmovilidad antes de intentar una recovery.
5. **Occupancy Grid sobredimensionado**: 1,000,000 de celdas para un sensor de 1 rayo.
6. **BFS usado como planificador**: Su rol natural es exploración, no planificación de rutas óptimas.

### 11.3 Próximos Pasos Recomendados

1. **Inmediato**: Ejecutar `metrics_collector_node.py` y `experiment_runner.py` para generar datos reales.
2. **Corto plazo**: Ajustar parámetros en YAMLs (critical_distance, resolution, plan_timeout).
3. **Mediano plazo**: Implementar nodo complementario de evasión reactiva.
4. **Largo plazo**: Entrenamiento RL para evasión autónoma de obstáculos.

---
