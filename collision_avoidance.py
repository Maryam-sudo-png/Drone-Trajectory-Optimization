"""
collision_avoidance.py — Collision Detection & Adaptive Replanning
===================================================================
Implements a two-tier threat system:

  Tier 1 — WARNING  (obstacle within WARNING_RADIUS)
      Pre-emptively replan the path before the drone is in danger.

  Tier 2 — CRITICAL (obstacle within COLLISION_RADIUS)
      Emergency stop + immediate replan.

Euclidean distance formula used throughout:
    d = sqrt((x2-x1)^2 + (y2-y1)^2)
"""

import math
import time
import logging
from enum import Enum, auto
from typing import List, Optional, Tuple, Callable

from config import COLLISION_RADIUS, WARNING_RADIUS
from obstacle_manager import ObstacleManager, DynamicObstacle

log = logging.getLogger(__name__)

Cell = Tuple[int, int]
Path = List[Cell]


# ──────────────────────────────────────────────────────────────────────────────
class ThreatLevel(Enum):
    NONE     = auto()
    WARNING  = auto()
    CRITICAL = auto()


# ──────────────────────────────────────────────────────────────────────────────
class CollisionEvent:
    """Record of a single collision-avoidance action."""
    def __init__(self, frame: int, position: Cell, threat: ThreatLevel,
                 obstacle_id: int, distance: float):
        self.frame       = frame
        self.position    = position
        self.threat      = threat
        self.obstacle_id = obstacle_id
        self.distance    = distance
        self.timestamp   = time.time()

    def __repr__(self) -> str:
        return (f"CollisionEvent(frame={self.frame}, pos={self.position}, "
                f"threat={self.threat.name}, obs={self.obstacle_id}, "
                f"dist={self.distance:.2f})")


# ──────────────────────────────────────────────────────────────────────────────
class CollisionAvoidance:
    """
    Monitors drone position against dynamic obstacles and decides when
    to trigger path replanning via an injected callback.

    Parameters
    ----------
    obs_manager      : the shared ObstacleManager instance
    replan_callback  : called with (current_pos, goal) → new Path | None
    collision_radius : hard-stop distance (cells)
    warning_radius   : pre-emptive replan distance (cells)
    """

    def __init__(self,
                 obs_manager     : ObstacleManager,
                 replan_callback : Callable[[Cell, Cell], Optional[Path]],
                 collision_radius: float = COLLISION_RADIUS,
                 warning_radius  : float = WARNING_RADIUS):
        self._obs            = obs_manager
        self._replan         = replan_callback
        self.coll_radius     = collision_radius
        self.warn_radius     = warning_radius

        # State
        self._last_threat    = ThreatLevel.NONE
        self._replan_cooldown= 0        # frames to wait before next replan
        self.COOLDOWN_FRAMES = 15       # prevent replan storm

        # Metrics
        self.collision_count : int  = 0
        self.replan_count    : int  = 0
        self.events          : List[CollisionEvent] = []

    # ── Main update ───────────────────────────────────────────────────────────

    def assess(self,
               drone_pos: Cell,
               goal     : Cell,
               current_path: Path,
               frame    : int = 0) -> Tuple[ThreatLevel, Optional[Path]]:
        """
        Assess the current situation and return:
            (threat_level, new_path_or_None)

        Call once per simulation frame.
        """
        if self._replan_cooldown > 0:
            self._replan_cooldown -= 1

        # ── Find closest threatening obstacle ─────────────────────────────────
        threats = self._obs.obstacles_within_radius(drone_pos, self.warn_radius)
        if not threats:
            self._last_threat = ThreatLevel.NONE
            return ThreatLevel.NONE, None

        # Sort by distance
        threats.sort(key=lambda o: o.distance_to(drone_pos))
        closest  = threats[0]
        distance = closest.distance_to(drone_pos)

        # ── Classify threat ───────────────────────────────────────────────────
        if distance <= self.coll_radius:
            threat = ThreatLevel.CRITICAL
        elif distance <= self.warn_radius:
            threat = ThreatLevel.WARNING
        else:
            threat = ThreatLevel.NONE

        if threat == ThreatLevel.NONE:
            self._last_threat = ThreatLevel.NONE
            return ThreatLevel.NONE, None

        # ── Record event ──────────────────────────────────────────────────────
        event = CollisionEvent(frame, drone_pos, threat, closest.id, distance)
        self.events.append(event)
        if threat == ThreatLevel.CRITICAL:
            self.collision_count += 1
            log.warning("CRITICAL: drone at %s, obstacle %d at %.2f cells",
                        drone_pos, closest.id, distance)

        # ── Decide whether to replan ──────────────────────────────────────────
        should_replan = (
            threat == ThreatLevel.CRITICAL
            or (threat == ThreatLevel.WARNING and self._last_threat == ThreatLevel.NONE)
            or not self._obs.path_is_clear(current_path)
        )

        new_path = None
        if should_replan and self._replan_cooldown == 0:
            new_path = self._replan(drone_pos, goal)
            if new_path:
                self.replan_count   += 1
                self._replan_cooldown = self.COOLDOWN_FRAMES
                log.info("Replanned path: %d nodes", len(new_path))
            else:
                log.error("No alternative path found — drone may be trapped!")

        self._last_threat = threat
        return threat, new_path

    # ── Prediction helper ─────────────────────────────────────────────────────

    def predict_collision_on_path(self,
                                  path          : Path,
                                  lookahead_frames: int = 20) -> bool:
        """
        Scan the next *lookahead_frames* steps along *path* and check whether
        any dynamic obstacle's predicted position comes within the collision
        radius of each waypoint.

        Returns True if a future collision is predicted.
        """
        for step, cell in enumerate(path[:lookahead_frames]):
            for obs in self._obs.dynamic_obstacles:
                px, py = obs.predicted_position(step)
                dist = math.hypot(px - cell[0], py - cell[1])
                if dist <= self.coll_radius:
                    return True
        return False

    # ── Static path safety check ──────────────────────────────────────────────

    @staticmethod
    def euclidean_distance(a: Cell, b: Tuple[float, float]) -> float:
        """d = sqrt((x2-x1)^2 + (y2-y1)^2)"""
        return math.hypot(b[0] - a[0], b[1] - a[1])

    # ── Metrics ───────────────────────────────────────────────────────────────

    def metrics(self) -> dict:
        return {
            "collision_count"   : self.collision_count,
            "replan_count"      : self.replan_count,
            "total_events"      : len(self.events),
            "last_threat"       : self._last_threat.name,
            "collision_radius"  : self.coll_radius,
            "warning_radius"    : self.warn_radius,
        }

    def reset(self) -> None:
        self.collision_count  = 0
        self.replan_count     = 0
        self.events.clear()
        self._last_threat     = ThreatLevel.NONE
        self._replan_cooldown = 0
