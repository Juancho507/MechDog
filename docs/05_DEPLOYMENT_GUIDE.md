# MechDog — Guía de Deployment en Hardware Real

## Arquitectura

```
LAPTOP CON DOCKER (Windows)
│
├── Docker containers (roscore + navigation + telemetry)
│   ├── global_planner  → A*, BFS o Dijkstra (configurable)
│   ├── local_planner   → DWA (800 trayectorias, 4s sim)
│   ├── safe_learning   → capa de seguridad (0.30m crítico)
│   └── occupancy_grid  → mapa de ocupación
│
└── ROS master en roscore:11311
         │
         │ WiFi (red local, misma subred)
         │
         ▼
MechDog con ESP32
    ├── Firmware Hiwonder ORIGINAL (sin cambios)
    ├── Recibe set_velocity(vx, wz) desde mechdog_hardware_interface
    └── Lee HC-SR04 por I2C (bus 1, addr 0x77)
```

## Hardware necesario

- Laptop con Windows (Docker Desktop instalado)
- MechDog con ESP32
- Cable USB (para primera prueba) o WiFi (para operación inalámbrica)

## ¿Qué es el ESP32 y qué NO es?

| El ESP32... | NO puede... |
|------------|-------------|
| Ejecuta firmware Hiwonder de fábrica | Correr ROS Noetic |
| Recibe comandos set_velocity(vx, wz) | Ejecutar Python del proyecto |
| Lee el sensor HC-SR04 por I2C | Hacer planificación A* o DWA |
| Se conecta por USB o WiFi | Ejecutar el archivo .py del proyecto |

**El ESP32 NO se programa. No se le "pasa" ningún archivo.** Solo recibe órdenes simples en tiempo real desde la laptop.

## Preparación de la laptop

### 1. Instalar Docker Desktop

```powershell
# Descargar de https://docs.docker.com/desktop/setup/install/windows-install/
# WSL2 backend recomendado (Ubuntu 22.04 LTS)
```

### 2. Clonar el proyecto

```powershell
git clone <url-del-repo> MechDog
cd MechDog
```

### 3. Compilar los contenedores

```powershell
docker compose build
```

(Esto construye la imagen multi-stage con ROS Noetic + todo el workspace)

### 4. Construir el workspace (una sola vez)

```powershell
docker compose run --rm builder bash -c "source /opt/ros/noetic/setup.bash && cd /app/catkin_ws && catkin_make"
```

## Día del deployment

### Opción A: Conexión por USB (prueba inicial)

```powershell
# 1. Conectar MechDog por USB a la laptop
# 2. Identificar el puerto COM del ESP32
#    (En Administrador de Dispositivos → Puertos COM)
# 3. Arrancar servicios ROS sin simulación
docker compose up -d roscore navigation telemetry
```

El stack de navegación corre igual que en simulación, pero sin Gazebo. En vez de recibir datos de Gazebo, espera los sensores reales del MechDog.

Falta configurar el `mechdog_hardware_interface` para que se conecte al ESP32.

### Opción B: Conexión por WiFi (inalámbrica, recomendada)

```powershell
# 1. Conectar el ESP32 del MechDog a la misma red WiFi que la laptop
# 2. Identificar la IP del ESP32 (ej: 192.168.1.100)
# 3. Configurar ROS_IP de la laptop para que coincida con su IP en esa red
$env:ROS_IP = "192.168.1.x"   # IP de la laptop en la red WiFi
# 4. Arrancar servicios
docker compose up -d roscore navigation telemetry
```

El ROS master corre en `roscore:11311`. El ESP32 necesita conectarse a `http://<laptop-ip>:11311` desde su red WiFi.

## Cómo cambiar entre algoritmos (A*, BFS, Dijkstra)

Los tres algoritmos ya están implementados y probados (42/42 tests):

```yaml
# catkin_ws/src/mechdog_navigation/config/global_planner.yaml
global_planner:
  algorithm: "astar"    # Cambiar a: "bfs", "dijkstra", "astar"
```

O en vivo desde la terminal del contenedor:

```powershell
docker compose exec navigation bash
# Dentro del contenedor:
rosparam set /global_planner/algorithm bfs
```

Para aplicar el cambio, el global_planner necesita recibir un nuevo goal o reiniciarse:

```powershell
docker compose restart navigation
```

### Diferencia entre algoritmos

| Algoritmo | Estrategia | Línea clave en planner_strategy.py |
|-----------|-----------|-----------------------------------|
| **A*** | `f = g + h` (heurística Manhattan). Va directo a la meta. | `planner_strategy.py:138` |
| **Dijkstra** | `f = g` (costo real). Explora en círculos. | `planner_strategy.py:184` |
| **BFS** | Cola FIFO. Explora por niveles. | `planner_strategy.py:228` |

## Parámetros para hardware real

Ya existen en `config/environments/real_hardware.yaml`:

```yaml
velocity:
  max_linear: 0.3       # Mitad que en sim (0.5 → más seguro)
  max_angular: 0.5
safety:
  critical_distance: 0.30   # Más conservador que sim (0.25)
  warning_distance: 0.60
```

## Para monitorear el robot

```powershell
# Ver qué nodos están corriendo
docker compose exec navigation rosnode list

# Ver los tópicos activos
docker compose exec navigation rostopic list

# Ver en Foxglove Studio (conexión local)
# Abrir Foxglove → conexión a ws://localhost:9090
```

## Pipeline de datos (hardware real)

```
MechDog (ESP32)
  │ sonar I2C (HC-SR04, 20 Hz)
  ▼
mechdog_hardware_interface (HAL)
  │ publica /mechdog/ultrasonic (Range)
  │ publica /mechdog/scan (LaserScan 1 rayo)
  ▼
occupancy_grid_node
  │ construye /mechdog/map (OccupancyGrid)
  ▼
global_planner_node (A* / BFS / Dijkstra)
  │ publica /mechdog/global_plan (Path)
  ▼
local_planner_node (DWA)
  │ publica /mechdog/cmd_vel_raw (Twist)
  ▼
safe_learning_node
  │ verifica distancia, frena si < 0.30m
  │ publica /cmd_vel (Twist)
  ▼
mechdog_hardware_interface
  │ set_velocity(vx, wz) por WiFi/USB
  ▼
ESP32 → motores
```

## Notas importantes

- Los nodos de navegación (`mechdog_navigation/`) son **100% portables** — no importan Gazebo, pybullet ni serial.
- El ESP32 no necesita flasheo ni cambios de firmware.
- WiFi es preferible a USB para libertad de movimiento.
- La Raspberry Pi Zero 2W (~$15) es una mejora futura para hacer el robot completamente autónomo sin depender de la laptop.
