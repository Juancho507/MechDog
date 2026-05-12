import sys
import time
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from exploration.maze import MazeEnvironment
from exploration.robot import SimulatedMechDog
from exploration.occupancy_grid import OccupancyGrid
from exploration.algorithms import get_algorithm

# ── CONFIG ────────────────────────────────────────────────────────────────────
MAZE_SIZE    = 10
SENSOR_RANGE = 1.5
MAX_STEPS    = 3000
STEP_DELAY   = 0.03    # segundos entre pasos (velocidad de animación)
# ─────────────────────────────────────────────────────────────────────────────


def build_rgb(grid) -> np.ndarray:
    """Convierte el grid a imagen RGB para matplotlib."""
    colors = {-1: [0.50, 0.50, 0.50],   # desconocido → gris
               0: [1.00, 1.00, 1.00],   # libre       → blanco
               1: [0.15, 0.15, 0.20]}   # obstáculo   → negro
    rgb = np.zeros((grid.height, grid.width, 3))
    for val, color in colors.items():
        rgb[grid.grid == val] = color
    rgb[grid.visited] = [0.70, 0.85, 1.00]   # recorrido → azul claro
    return rgb


def main():
    algo = sys.argv[1] if len(sys.argv) > 1 else "bfs"
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 42

    print(f"\n{'='*50}")
    print(f"  MechDog Visualizer — algoritmo: {algo.upper()}")
    print(f"  Laberinto {MAZE_SIZE}x{MAZE_SIZE} | seed={seed}")
    print(f"{'='*50}\n")

    # Entorno con GUI PyBullet
    env   = MazeEnvironment(size=MAZE_SIZE, seed=seed, gui=True)
    robot = SimulatedMechDog(env=env, sensor_range=SENSOR_RANGE)
    grid  = OccupancyGrid(width=MAZE_SIZE, height=MAZE_SIZE)
    explorer = get_algorithm(algo, robot, grid)

    # ── Ventana matplotlib del mapa 2D ────────────────────────────────────
    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    ax.set_title(f"MechDog — {algo.upper()} | Mapa de Ocupación",
                 color="white", fontsize=12, pad=10)
    ax.set_xticks([])
    ax.set_yticks([])

    im = ax.imshow(build_rgb(grid), origin="upper",
                   interpolation="nearest", vmin=0, vmax=1)
    robot_dot, = ax.plot([], [], "o", color="#3399FF",
                         markersize=14, zorder=5)

    legend_elements = [
        mpatches.Patch(color="white",     label="Libre"),
        mpatches.Patch(color="#26263a",   label="Obstáculo"),
        mpatches.Patch(color="#b2d8ff",   label="Recorrido"),
        mpatches.Patch(color="gray",      label="Desconocido"),
        mpatches.Patch(color="#3399FF",   label="Robot"),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=7,
              facecolor="#2a2a3e", labelcolor="white", framealpha=0.9)
    plt.tight_layout()
    plt.pause(0.001)
    # ──────────────────────────────────────────────────────────────────────

    print("Corriendo visualización... Ctrl+C para detener.\n")

    try:
        step = 0
        while step < MAX_STEPS and not explorer.is_done():
            explorer.step()
            grid.update(robot.get_position(), robot.sense())

            # Actualizar imagen
            r, c = robot.get_position()
            im.set_data(build_rgb(grid))
            robot_dot.set_data([c], [r])
            fig.canvas.draw_idle()
            plt.pause(0.001)

            time.sleep(STEP_DELAY)
            step += 1

        print(f"\n✅ Exploración terminada en {step} pasos.")
        print(f"   Celdas descubiertas: {grid.explored_ratio():.1%}")
        grid.save_image("/app/maps/final_map.png")
        fig.savefig("/app/maps/final_viz.png", dpi=120,
                    bbox_inches="tight", facecolor=fig.get_facecolor())
        print("   Mapas guardados en /app/maps/")
        print("\n   Visualización completa. Ctrl+C para salir.")
        # Mantener ventana abierta (sin input() que rompe Docker)
        while True:
            plt.pause(1.0)

    except KeyboardInterrupt:
        print("\nVisualización detenida.")
    finally:
        env.close()
        plt.close()


if __name__ == "__main__":
    main()
