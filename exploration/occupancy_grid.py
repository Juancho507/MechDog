"""
OccupancyGrid — mapa 2D actualizado en tiempo real con lecturas del sensor.
Valores: -1=desconocido, 0=libre, 1=obstáculo
"""

import numpy as np
from PIL import Image


class OccupancyGrid:
    UNKNOWN  = -1
    FREE     =  0
    OBSTACLE =  1

    def __init__(self, width: int, height: int):
        self.width   = width
        self.height  = height
        self.grid    = np.full((height, width), self.UNKNOWN, dtype=int)
        self.visited = np.zeros((height, width), dtype=bool)

    def update(self, position: tuple, sensor: dict):
        row, col = position
        if not self._ok(row, col):
            return
        self.grid[row, col]    = self.FREE
        self.visited[row, col] = True

        WALL_THRESH = 0.6   # menos de 60 cm → hay pared

        dir_map = {"front": (-1, 0), "right": (0, 1),
                   "back":  ( 1, 0), "left":  (0, -1)}
        for direction, dist in sensor.items():
            dr, dc = dir_map.get(direction, (0, 0))
            nr, nc = row+dr, col+dc
            if not self._ok(nr, nc):
                continue
            if dist < WALL_THRESH:
                self.grid[nr, nc] = self.OBSTACLE
            # Si no hay pared, la celda es potencialmente libre PERO
            # la marcamos FREE solo si aún es UNKNOWN (preservamos info previa)
            elif self.grid[nr, nc] == self.UNKNOWN:
                self.grid[nr, nc] = self.FREE

    def is_free(self, r, c) -> bool:
        return self._ok(r, c) and self.grid[r, c] == self.FREE

    def is_unknown(self, r, c) -> bool:
        return self._ok(r, c) and self.grid[r, c] == self.UNKNOWN

    def get_neighbors(self, r, c) -> list:
        return [(r+dr, c+dc)
                for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
                if self.is_free(r+dr, c+dc)]

    def frontier_cells(self) -> list:
        """Celdas libres adyacentes a celdas desconocidas."""
        fronts = []
        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r, c] == self.FREE:
                    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                        if self.is_unknown(r+dr, c+dc):
                            fronts.append((r, c))
                            break
        return fronts

    def explored_ratio(self) -> float:
        return np.sum(self.grid != self.UNKNOWN) / (self.width * self.height)

    def save_image(self, path: str):
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[self.grid == self.UNKNOWN]  = [128, 128, 128]
        rgb[self.grid == self.FREE]     = [255, 255, 255]
        rgb[self.grid == self.OBSTACLE] = [30,  30,  30]
        rgb[self.visited]               = [180, 210, 255]
        img = Image.fromarray(rgb).resize(
            (self.width*40, self.height*40), Image.NEAREST)
        img.save(path)
        print(f"Mapa guardado: {path}")

    def _ok(self, r, c) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width
