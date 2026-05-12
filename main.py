import sys
from exploration.maze import MazeEnvironment
from exploration.robot import SimulatedMechDog
from exploration.occupancy_grid import OccupancyGrid
from exploration.algorithms import get_algorithm

# ── CONFIG ────────────────────────────────────────────────────────────────────
MAZE_SIZE    = 10       # celdas del laberinto (NxN)
SENSOR_RANGE = 1.5      # metros, rango del ultrasónico simulado
MAX_STEPS    = 3000     # límite de pasos por episodio
# ─────────────────────────────────────────────────────────────────────────────


def main():
    # Algoritmo por argumento o BFS por defecto
    algo = sys.argv[1] if len(sys.argv) > 1 else "bfs"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 42

    print(f"\n{'='*50}")
    print(f"  MechDog Explorer — algoritmo: {algo.upper()}")
    print(f"  Laberinto {MAZE_SIZE}x{MAZE_SIZE} | seed={seed}")
    print(f"{'='*50}\n")

    # 1. Entorno PyBullet (sin GUI para el servicio train)
    env = MazeEnvironment(size=MAZE_SIZE, seed=seed, gui=False)

    # 2. Robot simulado con sensor ultrasónico
    robot = SimulatedMechDog(env=env, sensor_range=SENSOR_RANGE)

    # 3. Mapa de ocupación
    grid = OccupancyGrid(width=MAZE_SIZE, height=MAZE_SIZE)

    # 4. Algoritmo de exploración
    explorer = get_algorithm(algo, robot, grid)

    print("Iniciando exploración (modo headless)...\n")
    step = 0
    while step < MAX_STEPS and not explorer.is_done():
        explorer.step()
        grid.update(robot.get_position(), robot.sense())
        step += 1

    print(f"\n✅ Exploración terminada en {step} pasos.")
    print(f"   Celdas descubiertas: {grid.explored_ratio():.1%}")

    # Guardar mapa
    grid.save_image("/app/maps/final_map.png")
    print("   Mapa guardado en /app/maps/final_map.png")

    env.close()


if __name__ == "__main__":
    main()
