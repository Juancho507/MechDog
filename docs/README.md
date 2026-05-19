# MechDog ROS Architecture — Documentation Index

## RESUMEN EJECUTIVO

Este conjunto de documentos define la arquitectura ROS modular para el proyecto MechDog, diseñada específicamente para garantizar **paridad Sim-to-Real**: el mismo código CORE funciona sin modificaciones en simulación y hardware físico.

### Objetivo Principal
Transformar el proyecto actual (standalone PyBullet) en un sistema ROS 1 Noetic profesional que:
- Separe completamente simulación, hardware y lógica de control
- Utilice ROS Navigation Stack estándar (`move_base` + plugins) en lugar de implementaciones custom
- Implemente restricción cinemática no-holonómica (unicycle-like: vx, ωz solamente, NO vy)
- Permita validación exhaustiva en entorno simulado con ruido realista
- Facilite transición a robot cuadrúpedo físico sin reescribir algoritmos
- Proporcione métricas cuantitativas para evaluar Sim-to-Real gap

### Componentes Clave
1. **mechdog_sim**: Capa de simulación (Gazebo/PyBullet) con inyección de ruido
2. **mechdog_hw**: Drivers de hardware real (LIDAR, motores, IMU)
3. **mechdog_navigation**: Stack ROS Navigation (move_base + DWA + safe_controller) — 100% portable
4. **mechdog_metrics**: Análisis y comparación de performance sim vs real

### Garantías de Diseño
✅ ROS 1 Noetic exclusivamente (Catkin, rospy, roscore) — NO ROS 2  
✅ Uso de ROS Navigation Stack battle-tested (move_base, navfn, base_local_planner)  
✅ Restricción cinemática no-holonómica forzada (max_vel_y: 0.0, vy_samples: 1)  
✅ Código CORE no contiene imports de `pybullet`, `gazebo` ni drivers de hw  
✅ Todas las propiedades físicas vienen de URDF (single source of truth)  
✅ Configuración mediante YAML (no hardcoded values, no recompilación)  
✅ Topología de tópicos ROS estándar del Navigation Stack  
✅ Safe Controller como único punto de actuación (seguridad garantizada)  

---

## ESTRUCTURA DE DOCUMENTOS

### [00_PROJECT_STRUCTURE.md](00_PROJECT_STRUCTURE.md)
**Propósito**: Estructura de directorios y organización de paquetes ROS.

**Contenido**:
- Layout completo del catkin workspace
- Dependencias entre paquetes
- Convenciones de build system (CMakeLists.txt)
- Patrones de ejecución (roslaunch commands)
- Mapeo de código legacy → nueva estructura

**Audiencia**: Desarrolladores implementando la migración a ROS.

---

### [01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md)
**Propósito**: Reglas arquitectónicas estrictas y convenciones de nomenclatura.

**Contenido**:
- Diagrama de topología de nodos y tópicos
- Convenciones obligatorias para nombres de nodos/tópicos/frames
- Separación estricta hardware/simulación/CORE
- Frame conventions (TF tree)
- Parámetros críticos para Sim-to-Real
- Anti-patterns prohibidos

**Audiencia**: Arquitectos, code reviewers, IAs generadoras de código.

---

### [02_PROJECT_WORKFLOW.md](02_PROJECT_WORKFLOW.md)
**Propósito**: Flujo de ejecución temporal detallado del sistema completo.

**Contenido**:
- Timeline de inicio (fase de inicialización 0-5s)
- Ciclo de percepción → planning → control (10Hz loop)
- Fase de convergencia (llegada a meta)
- Diagrama de secuencia entre nodos
- Manejo de casos especiales (obstáculos dinámicos, re-planning)
- Sincronización crítica (timing constraints)
- Comandos completos de ejecución (sim y real)
- Pipeline de análisis post-run

**Audiencia**: Desarrolladores debuggeando comportamiento en tiempo real, operadores del sistema.

---

### [03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md)
**Propósito**: Instrucciones para futuras IAs sobre cómo mantener portabilidad del código CORE.

**Contenido**:
- Principios fundamentales (abstracción vía topics, URDF as truth, config via YAML)
- Variables críticas que DEBEN aislarse (tabla de enforcement)
- Estrategia de inyección de ruido (noise_injection_node)
- Checklist de validación pre-deployment
- Workflow de deployment (7 fases: sim limpia → sim con ruido → calibración → hw real)
- Debugging de issues Sim-to-Real comunes
- DO's and DON'Ts para IAs
- Métricas de éxito (code portability, topic consistency, trajectory deviation, etc.)

**Audiencia**: IAs generadoras de código, desarrolladores realizando transición sim→real.

---

## FLUJO DE LECTURA RECOMENDADO

### Para Desarrolladores Implementando Migración
1. **[00_PROJECT_STRUCTURE.md](00_PROJECT_STRUCTURE.md)** → entender organización de paquetes
2. **[01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md)** → aprender convenciones y restricciones
3. **[02_PROJECT_WORKFLOW.md](02_PROJECT_WORKFLOW.md)** → entender flujo de ejecución
4. **[03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md)** → garantizar portabilidad

### Para IAs Generadoras de Código
1. **[01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md)** → reglas de enforcement (CRÍTICO)
2. **[03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md)** → restricciones de portabilidad
3. **[02_PROJECT_WORKFLOW.md](02_PROJECT_WORKFLOW.md)** → contexto de sincronización temporal
4. **[00_PROJECT_STRUCTURE.md](00_PROJECT_STRUCTURE.md)** → dónde ubicar nuevos archivos

### Para Arquitectos/Revisores
1. **[01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md)** → validar compliance
2. **[03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md)** → verificar aislamiento correcto
3. **[00_PROJECT_STRUCTURE.md](00_PROJECT_STRUCTURE.md)** → revisar estructura de paquetes
4. **[02_PROJECT_WORKFLOW.md](02_PROJECT_WORKFLOW.md)** → validar pipeline completo

---

## QUICK REFERENCE

### Comandos Críticos

#### Simulación con Ruido (Training)
```bash
# Terminal 1: Simulación
roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true seed:=42

# Terminal 2: Navigation stack (move_base + safe_controller)
roslaunch mechdog_navigation navigation_stack.launch environment:=sim_noisy

# Terminal 3: Enviar goal
rosrun mechdog_navigation send_goal.py --x 10.0 --y 0.0

# Terminal 4: Métricas
roslaunch mechdog_metrics record_run.launch output_dir:=~/rosbags/sim/run_001
```

#### Hardware Real (Deployment)
```bash
# Terminal 1: Drivers de hardware (ÚNICO cambio)
roslaunch mechdog_hw hardware_drivers.launch

# Terminal 2: Navigation stack (IDÉNTICO a sim)
roslaunch mechdog_navigation navigation_stack.launch environment:=real_hw

# Terminal 3: Enviar goal (IDÉNTICO a sim)
rosrun mechdog_navigation send_goal.py --x 10.0 --y 0.0

# Terminal 4: Métricas (IDÉNTICO a sim)
roslaunch mechdog_metrics record_run.launch output_dir:=~/rosbags/real/run_001
```

**NOTA**: Terminales 2, 3, 4 son IDÉNTICOS. Solo cambia Terminal 1 (sim vs hw).

### Tópicos Clave

| Tópico | Tipo | Frecuencia | Descripción |
|--------|------|------------|-------------|
| `/mechdog/sensor/scan` | `sensor_msgs/LaserScan` | 20 Hz | LIDAR 2D |
| `/mechdog/odom` | `nav_msgs/Odometry` | 50 Hz | Pose + velocidad (vy=0 siempre) |
| `/map` | `nav_msgs/OccupancyGrid` | 1 Hz | Mapa estático (map_server) |
| `/move_base/NavfnROS/plan` | `nav_msgs/Path` | 1 Hz | Plan global (Dijkstra) |
| `/move_base/DWAPlannerROS/local_plan` | `nav_msgs/Path` | 10 Hz | Trayectoria local DWA |
| `/move_base/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | Comando DWA (pre-validación) |
| `/mechdog/cmd_vel_safe` | `geometry_msgs/Twist` | 20 Hz | Comando validado (post safe_controller) |
| `/mechdog/emergency_stop` | `std_msgs/Bool` | event | Trigger de parada |

### Archivos de Configuración Críticos

| Archivo | Propósito | Modificar en Sim-to-Real |
|---------|-----------|---------------------------|
| `mechdog_description/urdf/mechdog.urdf.xacro` | Definición física del robot | Solo si cambia hardware |
| `mechdog_navigation/config/base_local_planner_params.yaml` | **DWA: max_vel_y=0.0 (CRÍTICO)** | Ajustar velocidades, NO cambiar vy |
| `mechdog_navigation/config/costmap_common_params.yaml` | Footprint robot, inflation | Actualizar si cambian dimensiones |
| `mechdog_navigation/config/safe_controller.yaml` | Parámetros de seguridad | Base (heredado) |
| `mechdog_navigation/config/environments/sim_noisy.yaml` | Overrides simulación | Ajustar ruido si gap >15% |
| `mechdog_navigation/config/environments/real_hw.yaml` | Overrides hardware real | Ajustar velocidades, brake_decel post-calibración |
| `mechdog_sim/config/sim_noise_params.yaml` | Ruido de sensores | Calibrar vs datos reales |

---

## VALIDACIÓN DE COMPLIANCE

### Checklist para Code Review

Antes de aceptar un PR que modifique `mechdog_navigation/`:

```bash
# 1. Verificar no hay imports prohibidos
grep -r "import pybullet\|import gazebo\|import serial" mechdog_navigation/src/
# Output esperado: (vacío)

# 2. Verificar no hay hardcoded values
grep -r "ROBOT_LENGTH\|MAX_VELOCITY\|SENSOR_RANGE" mechdog_navigation/src/ | grep -v "rospy.get_param"
# Output esperado: (vacío)

# 3. Verificar archivos de config existen
ls mechdog_navigation/config/base_local_planner_params.yaml
ls mechdog_navigation/config/environments/sim_noisy.yaml
ls mechdog_navigation/config/environments/real_hw.yaml

# 4. Verificar restricción no-holonómica (CRÍTICO)
grep "max_vel_y:" mechdog_navigation/config/base_local_planner_params.yaml
grep "vy_samples:" mechdog_navigation/config/base_local_planner_params.yaml
# Output esperado: max_vel_y: 0.0 y vy_samples: 1

# 5. Verificar TF frames son estándar
grep -r "base_link\|base_footprint\|map\|odom" mechdog_navigation/src/
# Verificar que NO hay frames custom tipo "robot_base" o "my_odom"

# 6. Verificar que se usa move_base (NO custom planners)
grep -r "class.*Planner\|def.*plan" mechdog_navigation/src/
# Output esperado: (vacío, excepto safe_controller)
```

### Criterios de Rechazo Automático

❌ PR que modifica `mechdog_navigation/` y:
- Agrega dependencia de `mechdog_sim` o `mechdog_hw` en `package.xml`
- Contiene `import pybullet`, `import gazebo_msgs`, `import serial`
- Implementa planners custom (BFS, A*, DWA) en lugar de usar move_base
- Configura `max_vel_y != 0.0` o `vy_samples != 1` (rompe restricción no-holonómica)
- Hardcodea valores físicos (dimensiones, velocidades, offsets)
- Crea frames TF no estándar (fuera de `map`, `odom`, `base_link`, `base_footprint`)
- Bypasea `safe_controller` publicando directamente a `/mechdog/cmd_vel_safe`
- Usa ROS 2 (el proyecto es ROS 1 Noetic exclusivamente)

---

## MÉTRICAS DE ÉXITO DEL PROYECTO

### Fase 1: Implementación ROS (Simulación)
- [x] Estructura de paquetes creada según 00_PROJECT_STRUCTURE.md
- [ ] Topología de nodos funcional (todos los tópicos fluyen)
- [ ] Noise injection layer operativa
- [ ] Pipeline de métricas funcional (rosbags + scripts de análisis)
- [ ] Tasa de éxito >95% en simulación con ruido (50 runs)

### Fase 2: Validación de Portabilidad
- [ ] 0 líneas modificadas en `mechdog_navigation/src/` entre runs de sim y real
- [ ] 100% de tópicos idénticos (`rostopic list` sim vs real)
- [ ] TF tree idéntico (modulo timestamps)
- [ ] Checklist de compliance pasa sin warnings

### Fase 3: Deployment Hardware Real
- [ ] First run exitoso en hardware (robot se mueve sin crashes)
- [ ] Trajectory deviation (RMSE) < 15cm
- [ ] Map IoU (sim vs real) > 0.85
- [ ] Emergency stop frequency Δ < 20% (sim vs real)
- [ ] Tasa de éxito >90% en hardware real (20 runs)

---

## CONTACTO Y CONTRIBUCIONES

**Mantenedor**: Sistema MechDog (Proyecto Universitario de Robótica)  
**Última actualización**: 2026-05-19  
**Versión de docs**: 1.0.0  

**Para reportar issues**:
- Arquitectura: Revisar [01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md) primero
- Sim-to-Real gap: Consultar [03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md) sección Debugging
- Problemas de sincronización: Ver [02_PROJECT_WORKFLOW.md](02_PROJECT_WORKFLOW.md) timing constraints

**Para contribuir nuevo código**:
1. Leer TODOS los documentos en orden
2. Verificar compliance con checklist
3. Testear en sim con ruido antes de PR
4. Documentar overrides en `config/environments/` si necesario

---

## LICENCIA Y USO

Esta documentación está optimizada para consumo por:
- Desarrolladores humanos (estructurada, ejemplos claros)
- Modelos de IA (lenguaje técnico preciso, sin ambigüedades, enforcement explícito)

**Restricciones de uso**:
- NO modificar `mechdog_navigation/` sin leer [03_SIM_TO_REAL_STRATEGY.md](03_SIM_TO_REAL_STRATEGY.md)
- NO cambiar nombres de tópicos sin actualizar [01_ARCHITECTURE_ROS_RULES.md](01_ARCHITECTURE_ROS_RULES.md)
- NO saltarse validación de portabilidad antes de deployment

---

## APÉNDICES

### A. Comparación con Estado Actual del Proyecto

| Aspecto | Estado Actual | Estado Target (ROS) |
|---------|---------------|---------------------|
| Arquitectura | Monolito PyBullet | ROS modular (4 paquetes) |
| Portabilidad | Sim only | Sim + HW real (sin cambios en CORE) |
| Configuración | Hardcoded en código | YAML + URDF |
| Sensores | Raycast perfecto | Ruido gaussiano realista |
| Seguridad | Ad-hoc en algoritmos | Safe Controller centralizado |
| Métricas | Print statements | Rosbags + análisis automatizado |
| Validación | Manual/visual | Cuantitativa (IoU, RMSE, etc.) |
| CI/CD | Sin tests | ROS standard testing framework |

### B. Roadmap de Migración (Estimación)

| Fase | Duración | Entregables |
|------|----------|-------------|
| 1. Setup ROS workspace | 1 día | Catkin workspace + packages vacíos |
| 2. URDF del robot | 2 días | mechdog.urdf.xacro funcional |
| 3. Simulación Gazebo | 3 días | Mundo + spawn robot + noise injection |
| 4. Migración algoritmos CORE | 5 días | Global/local planners + safe controller |
| 5. Occupancy grid builder | 2 días | Sensor fusion → map |
| 6. Métricas y análisis | 2 días | Scripts de comparación sim/real |
| 7. Testing y validación | 3 días | 50 runs en sim con ruido |
| 8. Documentación código | 2 días | Docstrings, READMEs |
| **TOTAL** | **20 días** | Sistema ROS completo en simulación |

Post-implementación ROS:
- Calibración física: 1 día
- First run hardware: 1 día
- Ajuste fino: 3-5 días
- **TOTAL end-to-end**: ~25 días

### C. Recursos Externos

**ROS Tutorials**:
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [TF Conventions (REP-105)](https://www.ros.org/reps/rep-0105.html)
- [Coordinate Frames (REP-103)](https://www.ros.org/reps/rep-0103.html)

**Papers Relevantes**:
- "Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics" (OpenAI)
- "Learning Dexterous In-Hand Manipulation" (Domain Randomization)

**Herramientas**:
- [plotjuggler](https://github.com/facontidavide/PlotJuggler) - Visualización de rosbags
- [rqt_graph](http://wiki.ros.org/rqt_graph) - Visualización de topología de nodos
- [rviz](http://wiki.ros.org/rviz) - Visualización 3D de TF, mapas, trayectorias

---

**FIN DE DOCUMENTACIÓN**

Para comenzar implementación, iniciar con [00_PROJECT_STRUCTURE.md](00_PROJECT_STRUCTURE.md) y crear workspace.
