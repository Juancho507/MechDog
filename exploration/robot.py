"""
SimulatedMechDog — robot de movimiento discreto por celdas.
Sensor ultrasónico simulado con PyBullet raycast.
"""

import math
import pybullet as p


class SimulatedMechDog:

    # heading: 0=Norte, 1=Este, 2=Sur, 3=Oeste
    DIR_DELTA = {0: (-1, 0), 1: (0, 1), 2: (1, 0), 3: (0, -1)}
    DIR_YAW   = {0: 90, 1: 0, 2: 270, 3: 180}   # grados para PyBullet

    def __init__(self, env, sensor_range: float = 1.5):
        self.env          = env
        self.sensor_range = sensor_range
        self.row, self.col = env.start
        self.heading      = 1   # inicia mirando al Este
        self.body_id      = self._spawn()
        self._sync()

    def _spawn(self) -> int:
        col_sh = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.10, 0.15, 0.07])
        vis_sh = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.10, 0.15, 0.07],
            rgbaColor=[0.2, 0.6, 1.0, 1.0])
        wx, wy = self.env.cell_to_world(self.row, self.col)
        body = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=col_sh,
            baseVisualShapeIndex=vis_sh,
            basePosition=[wx, wy, 0.09],
        )
        p.changeDynamics(body, -1, linearDamping=10, angularDamping=10)
        return body

    def _sync(self):
        """Actualiza posición visual del robot en PyBullet."""
        wx, wy = self.env.cell_to_world(self.row, self.col)
        yaw = math.radians(self.DIR_YAW[self.heading])
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        p.resetBasePositionAndOrientation(self.body_id, [wx, wy, 0.09], orn)
        p.stepSimulation()

    # ── ACCIONES ─────────────────────────────────────────────────────────────

    def turn_left(self):
        self.heading = (self.heading - 1) % 4
        self._sync()

    def turn_right(self):
        self.heading = (self.heading + 1) % 4
        self._sync()

    def move_forward(self) -> bool:
        dr, dc = self.DIR_DELTA[self.heading]
        # Índice expandido de la pared entre celda actual y siguiente
        er_wall = 2*self.row + 1 + dr
        ec_wall = 2*self.col + 1 + dc
        if not self.env.is_wall(er_wall, ec_wall):
            self.row += dr
            self.col += dc
            self._sync()
            return True
        return False

    # ── SENSOR ───────────────────────────────────────────────────────────────

    def sense(self) -> dict:
        """Raycast en 4 direcciones → distancias en metros."""
        wx, wy = self.env.cell_to_world(self.row, self.col)
        origin = [wx, wy, 0.09]
        readings = {}

        for offset, name in [(0, "front"), (-1, "left"),
                              (1, "right"), (2, "back")]:
            h   = (self.heading + offset) % 4
            ang = math.radians(self.DIR_YAW[h])
            dx  = math.cos(ang) * self.sensor_range
            dy  = math.sin(ang) * self.sensor_range
            hit = p.rayTest(origin, [wx+dx, wy+dy, 0.09])
            if hit[0][0] >= 0:
                readings[name] = hit[0][2] * self.sensor_range
            else:
                readings[name] = self.sensor_range
        return readings

    # ── GETTERS ──────────────────────────────────────────────────────────────

    def get_position(self) -> tuple:
        return (self.row, self.col)

    def get_heading(self) -> int:
        return self.heading
