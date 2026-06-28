"""
environment.py — Simulation Environment Orchestrator
=====================================================
Ties together the obstacle manager, A* planner, drone controller,
and collision-avoidance system into a single simulation environment.

The environment exposes a clean step() API so that any frontend
(Pygame, Streamlit, Web dashboard) can drive the simulation at
whatever frame rate it likes.
"""

import logging
import time
from typing import Optional, List, Tuple, Callable

from config import (
    GRID_WIDTH, GRID_HEIGHT,
    DEFAULT_START, DEFAULT_GOAL,
    SIM_SPEED_DEF, HEURISTIC,
    WEATHER_MODES, WIND_STRENGTH,
    NUM_DYNAMIC_OBSTACLES,
)
from astar             import AStarPlanner
from obstacle_manager  import ObstacleManager
from drone_controller  import DroneController, FlightState
from collision_avoidance import CollisionAvoidance, ThreatLevel

log = logging.getLogger(__name__)

Cell = Tuple[int, int]
Path = List[Cell]


# ──────────────────────────────────────────────────────────────────────────────
class SimEnvironment:
    """
    Central simulation environment.

    Usage
    -----
    env = SimEnvironment()
    env.start_simulation()
    while not env.finished:
        env.step()
        # render env.drone, env.path, env.obstacles …
    """

    def __init__(self,
                 start      : Cell  = DEFAULT_START,
                 goal       : Cell  = DEFAULT_GOAL,
                 heuristic  : str   = HEURISTIC,
                 use_airsim : bool  = False,
                 seed       : Optional[int] = 42):

        self.start     = start
        self.goal      = goal
        self.heuristic = heuristic
        self.use_airsim= use_airsim

        # Sub-systems
        self.obstacles = ObstacleManager(start=start, goal=goal, seed=seed)
        self.planner   = AStarPlanner(GRID_WIDTH, GRID_HEIGHT, heuristic=heuristic)
        self.drone     = DroneController(start, goal, use_airsim=use_airsim)
        self.collision = CollisionAvoidance(
            obs_manager    = self.obstacles,
            replan_callback= self._replan,
        )

        # Simulation state
        self.running        : bool  = False
        self.finished       : bool  = False
        self.paused         : bool  = False
        self.frame          : int   = 0
        self.speed_mult     : float = SIM_SPEED_DEF
        self.weather_mode   : str   = "Clear"

        # Path state
        self.path           : Path  = []
        self.old_paths      : List[Path] = []   # previously used paths (for visualisation)
        self.initial_path   : Path  = []

        # Metrics log (appended every frame)
        self._metric_log    : List[dict] = []

        # Optional external callbacks
        self.on_replan      : Optional[Callable] = None
        self.on_goal_reached: Optional[Callable] = None
        self.on_collision   : Optional[Callable] = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start_simulation(self) -> bool:
        """
        Compute the initial path and arm the drone.
        Returns False if no path can be found.
        """
        self.finished = False
        self.frame    = 0
        blocked = self.obstacles.all_blocked_cells()
        path    = self.planner.find_path(self.start, self.goal, blocked)

        if not path:
            log.error("No initial path found from %s to %s", self.start, self.goal)
            return False

        # Optionally smooth the path
        path = self.planner.smooth_path(path, self.obstacles.static_cell_set())

        self.path         = path
        self.initial_path = list(path)
        self.drone.takeoff()
        self.drone.set_path(path)
        self.running = True
        log.info("Simulation started. Initial path: %d waypoints", len(path))
        log.info("A* stats: %s", self.planner.complexity_report())
        return True

    def reset(self) -> None:
        """Full reset — regenerates obstacles and replans from scratch."""
        self.obstacles = ObstacleManager(start=self.start, goal=self.goal)
        self.drone.reset(self.start)
        self.collision.reset()
        self.path       = []
        self.old_paths  = []
        self.running    = False
        self.finished   = False
        self.paused     = False
        self.frame      = 0
        self._metric_log.clear()
        log.info("Simulation reset.")

    def pause(self) -> None:
        self.paused = not self.paused

    # ── Main step ─────────────────────────────────────────────────────────────

    def step(self) -> None:
        """Advance the simulation by one logical frame."""
        if not self.running or self.paused or self.finished:
            return

        self.frame += 1

        # 1. Move dynamic obstacles
        self.obstacles.step(self.speed_mult)

        # 2. Assess collision risk
        threat, new_path = self.collision.assess(
            drone_pos    = self.drone.cell,
            goal         = self.goal,
            current_path = self.path,
            frame        = self.frame,
        )

        if threat == ThreatLevel.CRITICAL:
            self.drone.emergency_stop()
            if self.on_collision:
                self.on_collision(self.drone.cell, threat)

        if new_path:
            self.old_paths.append(list(self.path))
            self.path = new_path
            self.drone.notify_replan()
            self.drone.set_path(new_path)
            if self.on_replan:
                self.on_replan(self.frame, len(new_path))

        # 3. Advance drone
        goal_reached = self.drone.step(self.speed_mult)
        if goal_reached:
            self.finished = True
            self.running  = False
            log.info("Goal reached in %d frames!", self.frame)
            if self.on_goal_reached:
                self.on_goal_reached(self.frame, self.drone.total_distance)

        # 4. Log metrics
        self._metric_log.append(self._snapshot())

    # ── Replanning (injected into CollisionAvoidance) ──────────────────────────

    def _replan(self, current_pos: Cell, goal: Cell) -> Optional[Path]:
        """A* replan from current_pos to goal avoiding all current obstacles."""
        blocked = self.obstacles.all_blocked_cells()
        # Ensure drone's current cell is NOT in blocked (edge case)
        blocked.discard(current_pos)
        blocked.discard(goal)

        new_path = self.planner.find_path(current_pos, goal, blocked)
        if new_path:
            new_path = self.planner.smooth_path(new_path, self.obstacles.static_cell_set())
        return new_path

    # ── Setters ───────────────────────────────────────────────────────────────

    def set_speed(self, multiplier: float) -> None:
        self.speed_mult = float(multiplier)

    def set_heuristic(self, name: str) -> None:
        self.heuristic = name
        self.planner   = AStarPlanner(GRID_WIDTH, GRID_HEIGHT, heuristic=name)

    def set_weather(self, mode: str) -> None:
        import config as cfg
        self.weather_mode = mode
        if mode == "Windy"  : cfg.WIND_STRENGTH = 0.4
        elif mode == "Stormy": cfg.WIND_STRENGTH = 0.9
        else                 : cfg.WIND_STRENGTH = 0.0

    # ── Metrics snapshot ──────────────────────────────────────────────────────

    def _snapshot(self) -> dict:
        av = self.collision.metrics()
        dr = self.drone.status()
        return {
            "frame"          : self.frame,
            "battery"        : dr["battery"],
            "state"          : dr["state"],
            "pos"            : dr["position"],
            "progress"       : dr["progress_pct"],
            "collision_count": av["collision_count"],
            "replan_count"   : av["replan_count"],
            "path_length"    : dr["path_length"],
            "total_distance" : dr["total_distance"],
            "obstacles"      : self.obstacles.summary()["total_blocked"],
        }

    def full_metrics(self) -> dict:
        return {
            "total_frames"   : self.frame,
            "drone"          : self.drone.status(),
            "collision"      : self.collision.metrics(),
            "obstacle_summary": self.obstacles.summary(),
            "planner"        : self.planner.complexity_report(),
            "weather"        : self.weather_mode,
            "speed_mult"     : self.speed_mult,
            "heuristic"      : self.heuristic,
            "finished"       : self.finished,
        }
