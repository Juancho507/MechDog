"""
Visualizador del laberinto 3D — muestra la exploración en tiempo real
con la GUI de PyBullet.
Uso: python -m exploration.visualizer [algo] [seed]
"""

import sys
import time

from simulation.maze import MazeEnvironment
from simulation.robot import SimulatedMechDog
from exploration.occupancy_grid import OccupancyGrid
from exploration.algorithms import get_algorithm

MAZE_SIZE    = 10
SENSOR_RANGE = 1.5
MAX_STEPS    = 3000
STEP_DELAY   = 0.03


def main():
    algo = sys.argv[1] if len(sys.argv) > 1 else "bfs"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 42

    print(f"\n{'='*50}")
    print(f"  MechDog Visualizer — algoritmo: {algo.upper()}")
    print(f"  Laberinto {MAZE_SIZE}x{MAZE_SIZE} | seed={seed}")
    print(f"{'='*50}\n")

    env = MazeEnvironment(size=MAZE_SIZE, seed=seed, gui=True)
    robot = SimulatedMechDog(env=env, sensor_range=SENSOR_RANGE)
    grid = OccupancyGrid(width=MAZE_SIZE, height=MAZE_SIZE)
    explorer = get_algorithm(algo, robot, grid)

    print("Corriendo visualización... Ctrl+C para detener.\n")

    try:
        step = 0
        while step < MAX_STEPS and not explorer.is_done():
            explorer.step()
            grid.update(robot.get_position(), robot.sense(), robot.get_heading())
            time.sleep(STEP_DELAY)
            step += 1

        print(f"\nExploración terminada en {step} pasos.")
        print(f"   Celdas descubiertas: {grid.explored_ratio():.1%}")
        grid.save_image("/app/maps/final_viz.png")
        print("   Mapa guardado en /app/maps/final_viz.png")
        print("\n   Visualización completa. Ctrl+C para salir.")

        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nVisualización detenida.")
    finally:
        env.close()


if __name__ == "__main__":
    main()
