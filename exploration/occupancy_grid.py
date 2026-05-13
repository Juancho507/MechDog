import numpy as np
from PIL import Image


class OccupancyGrid:
    UNKNOWN = -1
    FREE = 0

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = np.full((height, width), self.UNKNOWN, dtype=int)
        self.visited = np.zeros((height, width), dtype=bool)
        self.wall_edges: dict[tuple, set] = {}

    def update(self, position: tuple, sensor: dict, heading: int = 0):
        row, col = position
        if not self._ok(row, col):
            return
        self.grid[row, col] = self.FREE
        self.visited[row, col] = True

        WALL_THRESH = 0.6

        dirs = {0: (-1, 0), 1: (0, 1), 2: (1, 0), 3: (0, -1)}
        dir_map = {"front": dirs[heading],
                   "right": dirs[(heading + 1) % 4],
                   "back":  dirs[(heading + 2) % 4],
                   "left":  dirs[(heading + 3) % 4]}
        for direction, dist in sensor.items():
            dr, dc = dir_map.get(direction, (0, 0))
            nr, nc = row + dr, col + dc
            if not self._ok(nr, nc):
                continue
            if dist < WALL_THRESH:
                self.wall_edges.setdefault((row, col), set()).add((dr, dc))
                self.wall_edges.setdefault((nr, nc), set()).add((-dr, -dc))
            else:
                self.grid[nr, nc] = self.FREE

    def is_free(self, r, c) -> bool:
        return self._ok(r, c) and self.grid[r, c] != self.UNKNOWN

    def is_unknown(self, r, c) -> bool:
        return self._ok(r, c) and self.grid[r, c] == self.UNKNOWN

    def has_wall(self, r, c, dr, dc) -> bool:
        return (dr, dc) in self.wall_edges.get((r, c), set())

    def get_neighbors(self, r, c) -> list:
        result = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if not self._ok(nr, nc):
                continue
            if self.grid[nr, nc] == self.UNKNOWN:
                continue
            if self.has_wall(r, c, dr, dc):
                continue
            result.append((nr, nc))
        return result

    def frontier_cells(self) -> list:
        fronts = []
        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r, c] != self.FREE:
                    continue
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if not self._ok(nr, nc):
                        continue
                    if self.is_unknown(nr, nc) and not self.has_wall(r, c, dr, dc):
                        fronts.append((r, c))
                        break
        return fronts

    def explored_ratio(self) -> float:
        return np.sum(self.grid != self.UNKNOWN) / (self.width * self.height)

    def save_image(self, path: str):
        rgb = np.full((self.height, self.width, 3), 128, dtype=np.uint8)
        rgb[self.grid == self.FREE] = [255, 255, 255]
        rgb[self.visited] = [180, 210, 255]
        for (r, c), edges in self.wall_edges.items():
            for dr, dc in edges:
                nr, nc = r + dr, c + dc
                if self._ok(nr, nc):
                    rgb[nr, nc] = [30, 30, 30]
        img = Image.fromarray(rgb).resize(
            (self.width * 40, self.height * 40), Image.NEAREST)
        img.save(path)

    def _ok(self, r, c) -> bool:
        return 0 <= r < self.height and 0 <= c < self.width
