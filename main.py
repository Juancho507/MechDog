"""
MechDog Explorer — entry point unificado.
Usa:  python main.py <algo> <seed>      → modo headless (solver)
      python main.py --vis <algo> <seed> → modo visual (GUI con PyBullet)
"""

import sys


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    algo = args[0] if args else "bfs"
    seed = int(args[1]) if len(args) > 1 else 42

    if "--vis" in sys.argv:
        from exploration.visualizer import main as vis_main
        sys.argv = [sys.argv[0], algo, str(seed)]
        vis_main()
    else:
        from exploration.solver import main as solver_main
        sys.argv = [sys.argv[0], algo, str(seed)]
        solver_main()


if __name__ == "__main__":
    main()
