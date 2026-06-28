"""
obstacle_manager.py — Static & Dynamic Obstacle Management
============================================================
Handles creation, movement, and querying of all obstacles in the
simulation grid.  Dynamic obstacles bounce off grid boundaries and
can trigger adaptive replanning in the collision-avoidance layer.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Set, Tuple, Optional

from config import (
    GRID_WIDTH, GRID_HEIGHT,
    NUM_STATIC_OBSTACLES, NUM_DYNAMIC_OBSTACLES,
    OBSTACLE_SPEED_RANGE, OBSTACLE_SIZE_RANGE,
    DEFAULT_START, DEFAULT_GOAL,
)

Cell = Tuple[int, int]


# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class StaticObstacle:
    """An immovable obstacle occupying one or more grid cells."""
    cx    : int          # centre column
    cy    : int          # centre row
    radius: int = 1      # cells — cells within radius are blocked

    def cells(self) -> Set[Cell]:
        result: Set[Cell] = set()
        for dx in range(-self.radius, self.radius + 1):
            for dy in range(-self.radius, self.radius + 1):
                if dx*dx + dy*dy <= self.radius * self.radius:
                    nx, ny = self.cx + dx, self.cy + dy
                    if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
                        result.add((nx, ny))
        return result


# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class DynamicObstacle:
    """
    A moving obstacle that follows a linear trajectory, bouncing off walls.

    Position is stored as floating-point for smooth sub-cell movement;
    the occupied cell set is derived from the rounded integer position.
    """
    x     : float        # fractional column
    y     : float        # fractional row
    vx    : float        # velocity in columns / frame
    vy    : float        # velocity in rows / frame
    radius: int   = 1
    id    : int   = 0

    # ── Movement ─────────────────────────────────────────────────────────────

    def step(self, speed_multiplier: float = 1.0) -> None:
        """Advance position by one frame; reflect off grid boundaries."""
        self.x += self.vx * speed_multiplier
        self.y += self.vy * speed_multiplier

        # Bounce horizontally
        if self.x - self.radius < 0:
            self.x  = float(self.radius)
            self.vx = abs(self.vx)
        elif self.x + self.radius >= GRID_WIDTH:
            self.x  = float(GRID_WIDTH - self.radius - 1)
            self.vx = -abs(self.vx)

        # Bounce vertically
        if self.y - self.radius < 0:
            self.y  = float(self.radius)
            self.vy = abs(self.vy)
        elif self.y + self.radius >= GRID_HEIGHT:
            self.y  = float(GRID_HEIGHT - self.radius - 1)
            self.vy = -abs(self.vy)

    # ── Geometry ─────────────────────────────────────────────────────────────

    @property
    def ix(self) -> int:
        return int(round(self.x))

    @property
    def iy(self) -> int:
        return int(round(self.y))

    def cells(self) -> Set[Cell]:
        result: Set[Cell] = set()
        for dx in range(-self.radius, self.radius + 1):
            for dy in range(-self.radius, self.radius + 1):
                if dx*dx + dy*dy <= self.radius * self.radius:
                    nx, ny = self.ix + dx, self.iy + dy
                    if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
                        result.add((nx, ny))
        return result

    def distance_to(self, pos: Cell) -> float:
        """Euclidean distance from obstacle centre to a grid cell."""
        return math.hypot(self.x - pos[0], self.y - pos[1])

    def predicted_position(self, frames_ahead: int) -> Tuple[float, float]:
        """Simple linear extrapolation (does not account for bounces)."""
        return self.x + self.vx * frames_ahead, self.y + self.vy * frames_ahead


# ──────────────────────────────────────────────────────────────────────────────
class ObstacleManager:
    """
    Central registry for all obstacles in the simulation.

    Provides:
    - Blocked-cell set for A* path planning
    - Dynamic obstacle update loop
    - Proximity queries for collision avoidance
    """

    def __init__(self,
                 start: Cell = DEFAULT_START,
                 goal : Cell = DEFAULT_GOAL,
                 seed : Optional[int] = None):
        self._start = start
        self._goal  = goal
        self._rng   = random.Random(seed)

        self.static_obstacles : List[StaticObstacle]  = []
        self.dynamic_obstacles: List[DynamicObstacle] = []

        # Keep exclusion zones around start and goal
        self._exclusion: Set[Cell] = self._build_exclusion_zone()

        self._generate_static()
        self._generate_dynamic()

    # ── Initialisation ────────────────────────────────────────────────────────

    def _build_exclusion_zone(self, margin: int = 4) -> Set[Cell]:
        zone: Set[Cell] = set()
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                sx, sy = self._start[0] + dx, self._start[1] + dy
                gx, gy = self._goal[0]  + dx, self._goal[1]  + dy
                if 0 <= sx < GRID_WIDTH  and 0 <= sy < GRID_HEIGHT:
                    zone.add((sx, sy))
                if 0 <= gx < GRID_WIDTH  and 0 <= gy < GRID_HEIGHT:
                    zone.add((gx, gy))
        return zone

    def _random_free_cell(self, radius: int = 1) -> Optional[Tuple[int, int]]:
        """Pick a random cell not in the exclusion zone and not already occupied."""
        existing = self.static_cell_set()
        for _ in range(200):
            cx = self._rng.randint(radius, GRID_WIDTH  - radius - 1)
            cy = self._rng.randint(radius, GRID_HEIGHT - radius - 1)
            candidate = StaticObstacle(cx, cy, radius)
            if not candidate.cells() & (self._exclusion | existing):
                return cx, cy
        return None

    def _generate_static(self) -> None:
        for _ in range(NUM_STATIC_OBSTACLES):
            r = self._rng.randint(*OBSTACLE_SIZE_RANGE)
            pos = self._random_free_cell(radius=r)
            if pos:
                self.static_obstacles.append(StaticObstacle(*pos, radius=r))

    def _generate_dynamic(self) -> None:
        margin = max(OBSTACLE_SIZE_RANGE)
        for i in range(NUM_DYNAMIC_OBSTACLES):
            r   = self._rng.randint(1, 2)
            spd = self._rng.uniform(*OBSTACLE_SPEED_RANGE)
            ang = self._rng.uniform(0, 2 * math.pi)
            x   = float(self._rng.randint(margin, GRID_WIDTH  - margin))
            y   = float(self._rng.randint(margin, GRID_HEIGHT - margin))
            obs = DynamicObstacle(
                x=x, y=y,
                vx=spd * math.cos(ang),
                vy=spd * math.sin(ang),
                radius=r,
                id=i,
            )
            self.dynamic_obstacles.append(obs)

    # ── Update ────────────────────────────────────────────────────────────────

    def step(self, speed_multiplier: float = 1.0) -> None:
        """Advance all dynamic obstacles by one simulation frame."""
        for obs in self.dynamic_obstacles:
            obs.step(speed_multiplier)

    # ── Cell sets (used by A*) ─────────────────────────────────────────────────

    def static_cell_set(self) -> Set[Cell]:
        result: Set[Cell] = set()
        for obs in self.static_obstacles:
            result |= obs.cells()
        return result

    def dynamic_cell_set(self) -> Set[Cell]:
        result: Set[Cell] = set()
        for obs in self.dynamic_obstacles:
            result |= obs.cells()
        return result

    def all_blocked_cells(self) -> Set[Cell]:
        return self.static_cell_set() | self.dynamic_cell_set()

    # ── Proximity queries ─────────────────────────────────────────────────────

    def nearest_dynamic_distance(self, pos: Cell) -> float:
        """Return distance to the closest dynamic obstacle centre."""
        if not self.dynamic_obstacles:
            return float("inf")
        return min(obs.distance_to(pos) for obs in self.dynamic_obstacles)

    def obstacles_within_radius(self, pos: Cell, radius: float) -> List[DynamicObstacle]:
        """Return all dynamic obstacles whose centre lies within *radius* cells of *pos*."""
        return [obs for obs in self.dynamic_obstacles if obs.distance_to(pos) <= radius]

    def path_is_clear(self, path: List[Cell]) -> bool:
        """Return True if no cell in *path* is currently blocked."""
        blocked = self.all_blocked_cells()
        return not any(cell in blocked for cell in path)

    # ── Debug ─────────────────────────────────────────────────────────────────

    def summary(self) -> dict:
        return {
            "static_obstacles" : len(self.static_obstacles),
            "dynamic_obstacles": len(self.dynamic_obstacles),
            "total_blocked"    : len(self.all_blocked_cells()),
        }
