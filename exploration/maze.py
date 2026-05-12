"""
MazeEnvironment — genera el laberinto 3D en PyBullet.
Algoritmo: Recursive Backtracker (genera laberintos perfectos).
"""

import random
import math
import numpy as np
import pybullet as p
import pybullet_data


class MazeEnvironment:

    WALL_HEIGHT = 0.5

    def __init__(self, size: int = 10, seed: int = 42, gui: bool = True):
        self.size = size
        self.seed = seed

        mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        if gui:
            total = size * 0.5   # cell_size / 2 = 0.5
            p.resetDebugVisualizerCamera(
                cameraDistance=size * 0.5 * 1.8,
                cameraYaw=45,
                cameraPitch=-55,
                cameraTargetPosition=[total * size / size,
                                      total * size / size, 0],
            )

        self._maze: np.ndarray = None   # grilla expandida (2*size+1)
        self.start = (0, 0)
        self.goal  = (size - 1, size - 1)
        self._build()

    # ── CONSTRUCCIÓN ─────────────────────────────────────────────────────────

    def _build(self):
        p.loadURDF("plane.urdf")
        self._maze = self._generate()

        N  = self._maze.shape[0]
        cs = 0.5   # mitad del tamaño de celda expandida (cell_size=1/2)

        for row in range(N):
            for col in range(N):
                if self._maze[row, col] == 1:
                    self._add_wall(row, col, cs)

        self._add_marker(*self.start, color=[0, 1, 0, 0.9])   # verde = inicio
        self._add_marker(*self.goal,  color=[1, 0, 0, 0.9])   # rojo  = meta

        walls = int(self._maze.sum())
        print(f"Laberinto {self.size}x{self.size} listo ({walls} bloques de pared).")

    def _generate(self) -> np.ndarray:
        """Recursive Backtracker → grilla expandida (2*size+1 x 2*size+1)."""
        rng  = random.Random(self.seed)
        size = self.size
        N    = 2 * size + 1
        mg   = np.ones((N, N), dtype=int)

        visited = [[False] * size for _ in range(size)]
        stack   = [(0, 0)]
        visited[0][0] = True
        mg[1][1] = 0

        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while stack:
            r, c = stack[-1]
            neighbors = [(r+dr, c+dc, dr, dc)
                         for dr, dc in dirs
                         if 0 <= r+dr < size and 0 <= c+dc < size
                         and not visited[r+dr][c+dc]]
            if neighbors:
                nr, nc, dr, dc = rng.choice(neighbors)
                mg[2*r+1+dr][2*c+1+dc] = 0   # abrir pared
                mg[2*nr+1][2*nc+1]     = 0   # abrir celda destino
                visited[nr][nc] = True
                stack.append((nr, nc))
            else:
                stack.pop()

        return mg

    def _add_wall(self, row: int, col: int, cs: float):
        col_sh = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[cs/2, cs/2, self.WALL_HEIGHT/2])
        vis_sh = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[cs/2, cs/2, self.WALL_HEIGHT/2],
            rgbaColor=[0.35, 0.35, 0.45, 1.0])
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col_sh,
            baseVisualShapeIndex=vis_sh,
            basePosition=[col * cs, row * cs, self.WALL_HEIGHT / 2],
        )

    def _add_marker(self, row: int, col: int, color: list):
        er, ec = 2*row+1, 2*col+1
        cs = 0.5
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.12, rgbaColor=color)
        p.createMultiBody(baseVisualShapeIndex=vis,
                          basePosition=[ec*cs, er*cs, 0.15])

    # ── UTILIDADES ───────────────────────────────────────────────────────────

    def cell_to_world(self, row: int, col: int):
        """Celda de paso (row, col) → coordenadas mundo (x, y)."""
        cs = 0.5
        return (2*col+1) * cs, (2*row+1) * cs

    def is_wall(self, exp_row: int, exp_col: int) -> bool:
        N = self._maze.shape[0]
        if 0 <= exp_row < N and 0 <= exp_col < N:
            return bool(self._maze[exp_row, exp_col] == 1)
        return True

    def close(self):
        p.disconnect(self.client)
