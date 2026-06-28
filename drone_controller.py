"""
drone_controller.py — Drone State Machine & Movement Controller
================================================================
Manages the drone's position, velocity, battery, and flight state.

When AirSim is available the controller issues real API calls.
When running in standalone simulation mode it advances the drone
along the planned path at the configured speed.

AirSim API calls mirrored here:
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToPositionAsync(x, y, z, speed)
    client.landAsync().join()
"""

import math
import logging
import time
from enum import Enum, auto
from typing import Optional, List, Tuple

from config import (
    DRONE_SPEED, DRONE_ALTITUDE, AIRSIM_SCALE,
    BATTERY_CAPACITY, BATTERY_MOVE_DRAIN,
    BATTERY_HOVER_DRAIN, BATTERY_REPLAN_PENALTY,
    BATTERY_LOW_THRESHOLD, DEFAULT_START, DEFAULT_GOAL,
    WIND_STRENGTH,
)

log = logging.getLogger(__name__)

Cell = Tuple[int, int]
Path = List[Cell]


# ──────────────────────────────────────────────────────────────────────────────
class FlightState(Enum):
    IDLE       = auto()
    TAKING_OFF = auto()
    FLYING     = auto()
    HOVERING   = auto()   # stopped; obstacle nearby
    REPLANNING = auto()
    LANDING    = auto()
    LANDED     = auto()
    EMERGENCY  = auto()   # battery critical or no path


# ──────────────────────────────────────────────────────────────────────────────
class DroneController:
    """
    Controls drone movement along a pre-planned path.

    Parameters
    ----------
    start         : starting grid cell
    goal          : destination grid cell
    use_airsim    : if True, attempt to connect to AirSim
    speed         : movement speed (cells / frame in sim mode)
    """

    def __init__(self,
                 start      : Cell = DEFAULT_START,
                 goal       : Cell = DEFAULT_GOAL,
                 use_airsim : bool = False,
                 speed      : float = 0.12):
        self.start = start
        self.goal  = goal
        self.speed = speed

        # Floating-point position (smooth sub-cell movement)
        self.x  : float = float(start[0])
        self.y  : float = float(start[1])

        self.state   : FlightState  = FlightState.IDLE
        self.path    : Path         = []
        self.path_index : int       = 0   # next waypoint index

        # Battery
        self.battery       : float = BATTERY_CAPACITY
        self.battery_log   : List[float] = [BATTERY_CAPACITY]

        # Distance tracking
        self.total_distance: float = 0.0
        self.path_history  : List[Cell] = [start]

        # AirSim client (None in simulation mode)
        self._airsim_client = None
        if use_airsim:
            self._connect_airsim()

        # Trail of visited cells for visualisation
        self.trail : List[Tuple[float, float]] = [(self.x, self.y)]

    # ── AirSim integration ────────────────────────────────────────────────────

    def _connect_airsim(self) -> None:
        """Attempt to connect to a running AirSim instance."""
        try:
            import airsim                                   # type: ignore
            self._airsim_client = airsim.MultirotorClient()
            self._airsim_client.confirmConnection()
            self._airsim_client.enableApiControl(True)
            self._airsim_client.armDisarm(True)
            log.info("AirSim connected successfully.")
        except Exception as exc:
            log.warning("AirSim not available (%s). Running in simulation mode.", exc)
            self._airsim_client = None

    def _airsim_takeoff(self) -> None:
        if self._airsim_client:
            self._airsim_client.takeoffAsync().join()

    def _airsim_move_to(self, cell: Cell) -> None:
        if self._airsim_client:
            x = cell[0] * AIRSIM_SCALE
            y = cell[1] * AIRSIM_SCALE
            z = DRONE_ALTITUDE
            self._airsim_client.moveToPositionAsync(
                x, y, z, DRONE_SPEED
            ).join()

    def _airsim_land(self) -> None:
        if self._airsim_client:
            self._airsim_client.landAsync().join()
            self._airsim_client.armDisarm(False)
            self._airsim_client.enableApiControl(False)

    # ── Flight lifecycle ──────────────────────────────────────────────────────

    def takeoff(self) -> None:
        self.state = FlightState.TAKING_OFF
        log.info("Drone taking off from %s", self.start)
        self._airsim_takeoff()
        self.state = FlightState.HOVERING

    def set_path(self, path: Path) -> None:
        """Load a new path and reset the waypoint index to 1 (skip current pos)."""
        if not path:
            return
        self.path       = path
        self.path_index = 1   # index 0 == current position
        self.state      = FlightState.FLYING

    def land(self) -> None:
        self.state = FlightState.LANDING
        self._airsim_land()
        self.state = FlightState.LANDED
        log.info("Drone landed. Total distance: %.2f cells", self.total_distance)

    def emergency_stop(self) -> None:
        self.state = FlightState.HOVERING
        log.warning("Emergency stop at (%.1f, %.1f)", self.x, self.y)

    # ── Per-frame update ──────────────────────────────────────────────────────

    def step(self, speed_multiplier: float = 1.0) -> bool:
        """
        Advance the drone by one simulation frame.
        Returns True when the goal has been reached.
        """
        if self.state not in (FlightState.FLYING, FlightState.REPLANNING):
            # Hover drain
            self._drain_battery(BATTERY_HOVER_DRAIN)
            return False

        if not self.path or self.path_index >= len(self.path):
            # Path exhausted
            if self._at_goal():
                self.land()
                return True
            self.state = FlightState.HOVERING
            return False

        # ── Move toward next waypoint ─────────────────────────────────────────
        target_x, target_y = float(self.path[self.path_index][0]), \
                              float(self.path[self.path_index][1])
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        step_size = self.speed * speed_multiplier

        # Apply wind drift
        if WIND_STRENGTH > 0:
            import random
            self.x += random.uniform(-WIND_STRENGTH, WIND_STRENGTH) * 0.1
            self.y += random.uniform(-WIND_STRENGTH, WIND_STRENGTH) * 0.1

        if dist <= step_size:
            # Arrived at waypoint
            self.x, self.y = target_x, target_y
            prev_cell = self.path[self.path_index - 1]
            step_cost = math.hypot(target_x - prev_cell[0], target_y - prev_cell[1])
            self.total_distance += step_cost
            self._drain_battery(BATTERY_MOVE_DRAIN * step_cost)
            self.trail.append((self.x, self.y))
            self.path_history.append(self.path[self.path_index])
            self.path_index += 1

            # AirSim: issue move command to next waypoint
            if self.path_index < len(self.path):
                self._airsim_move_to(self.path[self.path_index])
        else:
            # Interpolate toward waypoint
            nx = self.x + (dx / dist) * step_size
            ny = self.y + (dy / dist) * step_size
            step_moved = math.hypot(nx - self.x, ny - self.y)
            self.total_distance += step_moved
            self._drain_battery(BATTERY_MOVE_DRAIN * step_moved)
            self.x, self.y = nx, ny
            self.trail.append((self.x, self.y))

        # Check battery emergency
        if self.battery <= 0:
            self.state   = FlightState.EMERGENCY
            self.battery = 0.0
            log.error("Battery depleted — emergency landing!")
            return False

        return False

    def notify_replan(self) -> None:
        """Called whenever a replanning event occurs; drains extra battery."""
        self._drain_battery(BATTERY_REPLAN_PENALTY)
        self.state = FlightState.REPLANNING

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _drain_battery(self, amount: float) -> None:
        self.battery = max(0.0, self.battery - amount)
        self.battery_log.append(round(self.battery, 2))

    def _at_goal(self) -> bool:
        return math.hypot(self.x - self.goal[0], self.y - self.goal[1]) < 1.0

    @property
    def cell(self) -> Cell:
        """Current grid cell (rounded integer position)."""
        return (int(round(self.x)), int(round(self.y)))

    @property
    def battery_pct(self) -> float:
        return self.battery

    @property
    def battery_low(self) -> bool:
        return self.battery < BATTERY_LOW_THRESHOLD

    @property
    def progress_pct(self) -> float:
        if not self.path:
            return 0.0
        return min(100.0, (self.path_index / max(len(self.path) - 1, 1)) * 100)

    def reset(self, start: Optional[Cell] = None) -> None:
        s = start or self.start
        self.x             = float(s[0])
        self.y             = float(s[1])
        self.state         = FlightState.IDLE
        self.path          = []
        self.path_index    = 0
        self.battery       = BATTERY_CAPACITY
        self.battery_log   = [BATTERY_CAPACITY]
        self.total_distance= 0.0
        self.path_history  = [s]
        self.trail         = [(self.x, self.y)]

    def status(self) -> dict:
        return {
            "state"          : self.state.name,
            "position"       : (round(self.x, 2), round(self.y, 2)),
            "cell"           : self.cell,
            "battery"        : round(self.battery, 1),
            "progress_pct"   : round(self.progress_pct, 1),
            "total_distance" : round(self.total_distance, 2),
            "path_length"    : len(self.path),
            "waypoint_index" : self.path_index,
        }
