"""
Solver headless — ejecuta un algoritmo de exploración sobre el laberinto
sin ventana gráfica (modo DIRECT de PyBullet).
Uso: python -m exploration.solver [algo] [seed]
"""

import sys
from simulation.maze import MazeEnvironment
from simulation.robot import SimulatedMechDog
from exploration.occupancy_grid import OccupancyGrid
from exploration.algorithms import get_algorithm

MAZE_SIZE    = 10
SENSOR_RANGE = 1.5
MAX_STEPS    = 3000


def main():
    algo = sys.argv[1] if len(sys.argv) > 1 else "bfs"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 42

    print(f"\n{'='*50}")
    print(f"  MechDog Solver — algoritmo: {algo.upper()}")
    print(f"  Laberinto {MAZE_SIZE}x{MAZE_SIZE} | seed={seed}")
    print(f"{'='*50}\n")

    env = MazeEnvironment(size=MAZE_SIZE, seed=seed, gui=False)
    robot = SimulatedMechDog(env=env, sensor_range=SENSOR_RANGE)
    grid = OccupancyGrid(width=MAZE_SIZE, height=MAZE_SIZE)

    explorer = get_algorithm(algo, robot, grid)

    print("Iniciando exploración (modo headless)...\n")
    step = 0
    while step < MAX_STEPS and not explorer.is_done():
        pos_before = robot.get_position()
        explorer.step()
        pos_after = robot.get_position()
        grid.update(pos_after, robot.sense(), robot.get_heading())
        step += 1
        if step <= 6 or (step <= 100 and step % 20 == 0):
            print(f"  step {step}: {pos_before}->{pos_after}, explored={grid.explored_ratio():.1%}")

    print(f"\nExploración terminada en {step} pasos.")
    print(f"   Celdas descubiertas: {grid.explored_ratio():.1%}")

    grid.save_image(f"/app/maps/{algo}_map.png")
    print(f"   Mapa guardado en /app/maps/{algo}_map.png")

    env.close()


if __name__ == "__main__":
    main()
