# MechDog — Exploración Autónoma 3D

> PyBullet + Docker | Robótica Universitaria

---

## Estructura

```
MechDog/
├── main.py                    Entry point unificado
├── simulation/
│   ├── maze.py                Laberinto 3D (PyBullet)
│   └── robot.py               Robot simulado + sensor ultrasónico
├── exploration/
│   ├── algorithms.py          BFS, DFS, A*, Random
│   ├── occupancy_grid.py      Mapa de ocupación 2D
│   ├── solver.py              Ejecutor headless (solo algoritmos)
│   └── visualizer.py          Visualización 3D con GUI
├── visualize.py               Legacy (visualizador directo)
├── start_vnc.sh               Arranca display virtual + noVNC
├── Dockerfile
├── docker-compose.yml
├── requirements.txt
└── maps/                      Mapas generados (output)
```

---

## Comandos Docker

### Construir imagen
```bash
docker compose build
```

### PARTE 1 — Algoritmos de exploración (headless)

Ejecuta BFS, DFS, A\* o Random sin ventana gráfica y guarda el mapa:

```bash
docker compose run solver bfs 42
docker compose run solver dfs 99
docker compose run solver astar 7
docker compose run solver random 1
```

O con el entry point unificado:

```bash
docker compose run train python main.py bfs 42
```

### PARTE 2 — Simulación 3D (laberinto visible)

Abre el laberinto 3D en el navegador con el robot explorando:

```bash
docker compose up visualize
```

Luego abrir en el navegador:

```
http://localhost:6080/vnc.html
```

Con algoritmo específico:

```bash
docker compose run --service-ports visualize /start_vnc.sh bfs 42
```

---

## Algoritmos

| Nombre   | Estrategia                        | Uso                   |
|----------|-----------------------------------|-----------------------|
| `bfs`    | Amplitud — explora por capas      | Camino mínimo         |
| `dfs`    | Profundidad — va hasta el fondo   | Menos memoria         |
| `astar`  | Heurística Manhattan              | Más eficiente con meta|
| `random` | Movimiento aleatorio              | Baseline comparación  |

---

## Parámetros (en solver.py y visualizer.py)

```python
MAZE_SIZE    = 10     # Tamaño laberinto NxN
SENSOR_RANGE = 1.5    # Rango ultrasónico (metros)
MAX_STEPS    = 3000   # Límite de pasos
STEP_DELAY   = 0.03   # Velocidad animación (solo visualizer.py)
```

---

## Separación del proyecto

- **`simulation/`** — genera el laberinto 3D y el robot simulado (PyBullet)
- **`exploration/`** — algoritmos de exploración puros, independientes de la simulación
