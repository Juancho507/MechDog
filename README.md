# 🐕 MechDog — Exploración Autónoma 3D

> PyBullet + Docker | Robótica Universitaria

---

## 📁 Estructura

```
MechDog/
├── main.py               ← Exploración headless (entrena/corre sin GUI)
├── visualize.py          ← Visualización 3D por navegador (noVNC)
├── start_vnc.sh          ← Arranca display virtual + noVNC + visualize.py
├── Dockerfile
├── docker-compose.yml
├── requirements.txt
├── maps/                 ← Mapas generados (output)
└── exploration/
    ├── maze.py           ← Laberinto 3D en PyBullet
    ├── robot.py          ← MechDog simulado + sensor ultrasónico (raycast)
    ├── occupancy_grid.py ← Mapa de ocupación 2D
    └── algorithms.py     ← BFS, DFS, A*, Random
```

---

## 🚀 Comandos

### Construir la imagen (solo la primera vez)
```bash
docker compose build
```

### Correr exploración headless (sin ventana)
```bash
docker compose run train
```
Con algoritmo específico:
```bash
docker compose run train python main.py bfs 42
docker compose run train python main.py dfs 99
docker compose run train python main.py astar 7
docker compose run train python main.py random 1
```

### Ver la simulación en el navegador
```bash
docker compose up visualize
```
Luego abrir en Chrome/Edge:
```
http://localhost:6080/vnc.html
```
Con algoritmo específico:
```bash
docker compose run --service-ports visualize /start_vnc.sh bfs 42
```

---

## 🧠 Algoritmos

| Nombre   | Estrategia                        | Uso                   |
|----------|-----------------------------------|-----------------------|
| `bfs`    | Amplitud — explora por capas      | Camino mínimo         |
| `dfs`    | Profundidad — va hasta el fondo   | Menos memoria         |
| `astar`  | Heurística Manhattan              | Más eficiente con meta|
| `random` | Movimiento aleatorio              | Baseline comparación  |

---

## ⚙️ Parámetros (en main.py y visualize.py)

```python
MAZE_SIZE    = 10     # Tamaño laberinto NxN
SENSOR_RANGE = 1.5    # Rango ultrasónico (metros)
MAX_STEPS    = 3000   # Límite de pasos
STEP_DELAY   = 0.03   # Velocidad animación (solo visualize.py)
```

---

## 📅 Cronograma

| Fecha    | Actividad                          |
|----------|------------------------------------|
| 13 Mayo  | Experimento con robot físico       |
| 25 Mayo  | Experimento con robot físico       |
| **27 Mayo** | **Demostración final**          |
