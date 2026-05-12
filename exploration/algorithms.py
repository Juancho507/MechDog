"""
Algoritmos de exploración autónoma del MechDog.
Cada uno expone .step() e .is_done().
"""

from collections import deque
import heapq
import random


def get_algorithm(name: str, robot, grid):
    opts = {"bfs": BFSExplorer, "dfs": DFSExplorer,
            "astar": AStarExplorer, "random": RandomExplorer}
    cls = opts.get(name.lower())
    if cls is None:
        raise ValueError(f"Algoritmo '{name}' no existe. Opciones: {list(opts)}")
    alg = cls(robot, grid)
    # ── Inicializar el grid con la posición y lectura inicial ──────────────
    grid.update(robot.get_position(), robot.sense())
    return alg


# ── BASE ──────────────────────────────────────────────────────────────────────

class BaseExplorer:
    def __init__(self, robot, grid):
        self.robot = robot
        self.grid  = grid
        self._done = False

    def step(self): raise NotImplementedError
    def is_done(self) -> bool: return self._done

    def _move_to(self, tr: int, tc: int) -> bool:
        """Gira y avanza un paso hacia la celda (tr, tc)."""
        r, c = self.robot.get_position()
        dr, dc = tr - r, tc - c
        needed = {(-1,0): 0, (0,1): 1, (1,0): 2, (0,-1): 3}.get((dr, dc))
        if needed is None:
            return False
        for _ in range(4):
            if self.robot.get_heading() == needed:
                break
            self.robot.turn_right()
        return self.robot.move_forward()

    def _bfs_path(self, start: tuple, goal: tuple) -> list:
        """BFS para encontrar camino entre dos celdas libres."""
        q = deque([[start]])
        seen = {start}
        while q:
            path = q.popleft()
            if path[-1] == goal:
                return path[1:]
            for nb in self.grid.get_neighbors(*path[-1]):
                if nb not in seen:
                    seen.add(nb)
                    q.append(path + [nb])
        return []


# ── BFS ───────────────────────────────────────────────────────────────────────

class BFSExplorer(BaseExplorer):
    """
    Exploración BFS: visita todas las celdas FREE no visitadas.
    Prioriza fronteras (FREE adyacente a UNKNOWN), si no hay, va a FREE no visitada.
    """
    def __init__(self, robot, grid):
        super().__init__(robot, grid)
        self._path = []

    def step(self):
        if not self._path:
            self._path = self._next_target_path()
            if not self._path:
                self._done = True
                print("BFS: exploración completa.")
                return
        if not self._move_to(*self._path[0]):
            self._path = []   # recalcular si bloqueado
        else:
            self._path.pop(0)

    def _next_target_path(self) -> list:
        start = self.robot.get_position()

        # 1. Intentar ir a una celda frontera (FREE adyacente a UNKNOWN)
        frontiers = set(map(tuple, self.grid.frontier_cells()))

        # 2. Si no hay fronteras, ir a cualquier celda FREE no visitada
        if not frontiers:
            targets = set()
            for r in range(self.grid.height):
                for c in range(self.grid.width):
                    if self.grid.is_free(r, c) and not self.grid.visited[r, c]:
                        targets.add((r, c))
            if not targets:
                return []
            frontiers = targets

        # BFS desde posición actual hasta el target más cercano
        q = deque([[start]])
        seen = {start}
        while q:
            path = q.popleft()
            if path[-1] in frontiers:
                return path[1:]
            for nb in self.grid.get_neighbors(*path[-1]):
                if nb not in seen:
                    seen.add(nb)
                    q.append(path + [nb])
        return []


# ── DFS ───────────────────────────────────────────────────────────────────────

class DFSExplorer(BaseExplorer):
    """Profundiza antes de retroceder (depth-first). Visita todas las FREE."""
    def __init__(self, robot, grid):
        super().__init__(robot, grid)
        self._stack   = [robot.get_position()]
        self._visited = set()
        self._path    = []

    def step(self):
        if self._path:
            if not self._move_to(*self._path[0]):
                self._path = []
            else:
                self._path.pop(0)
            return

        # Añadir celdas FREE no visitadas al stack si el stack está vacío
        if not self._stack:
            unvisited = [(r, c)
                         for r in range(self.grid.height)
                         for c in range(self.grid.width)
                         if self.grid.is_free(r, c) and (r, c) not in self._visited]
            if not unvisited:
                self._done = True
                print("DFS: exploración completa.")
                return
            self._stack.append(unvisited[0])

        target = self._stack[-1]
        r, c   = self.robot.get_position()
        if target == (r, c):
            self._visited.add(target)
            self._stack.pop()
            nbs = self.grid.get_neighbors(r, c)
            random.shuffle(nbs)
            for nb in nbs:
                if nb not in self._visited:
                    self._stack.append(nb)
        else:
            self._path = self._bfs_path((r, c), target)
            if not self._path:
                self._stack.pop()


# ── A* ────────────────────────────────────────────────────────────────────────

class AStarExplorer(BaseExplorer):
    """Navega a la frontera con menor f = g + h (Manhattan)."""
    def __init__(self, robot, grid):
        super().__init__(robot, grid)
        self._path = []
        self.goal  = (grid.height - 1, grid.width - 1)

    def _h(self, pos) -> int:
        return abs(pos[0] - self.goal[0]) + abs(pos[1] - self.goal[1])

    def _astar(self, start: tuple, goal: tuple) -> list:
        heap = [(self._h(start), 0, start, [])]
        closed = set()
        while heap:
            _, g, node, path = heapq.heappop(heap)
            if node in closed:
                continue
            closed.add(node)
            path = path + [node]
            if node == goal:
                return path[1:]
            for nb in self.grid.get_neighbors(*node):
                if nb not in closed:
                    ng = g + 1
                    heapq.heappush(heap, (ng + self._h(nb), ng, nb, path))
        return []

    def step(self):
        if not self._path:
            fronts = self.grid.frontier_cells()
            if fronts:
                target = min(fronts, key=self._h)
            else:
                # Sin fronteras: ir a celda FREE no visitada más cercana a la meta
                unvisited = [(r, c)
                             for r in range(self.grid.height)
                             for c in range(self.grid.width)
                             if self.grid.is_free(r, c) and not self.grid.visited[r, c]]
                if not unvisited:
                    self._done = True
                    print("A*: exploración completa.")
                    return
                target = min(unvisited, key=self._h)

            self._path = self._astar(self.robot.get_position(), target)
            if not self._path:
                self._done = True
                return

        if not self._move_to(*self._path[0]):
            self._path = []
        else:
            self._path.pop(0)


# ── RANDOM ────────────────────────────────────────────────────────────────────

class RandomExplorer(BaseExplorer):
    """Movimiento aleatorio (baseline de comparación)."""
    def step(self):
        if not self.grid.frontier_cells():
            self._done = True
            return
        action = random.choice(["forward", "left", "right"])
        if action == "left":
            self.robot.turn_left()
        elif action == "right":
            self.robot.turn_right()
        self.robot.move_forward()
