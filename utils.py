"""
utils.py — Shared Utility Functions
====================================
Formatting, logging setup, and small mathematical helpers shared
across all modules.
"""

import math
import logging
import sys
from typing import Tuple, List

Cell = Tuple[int, int]


# ──────────────────────────────────────────────────────────────────────────────
def setup_logging(level: int = logging.INFO) -> None:
    """Configure root logger with coloured console output."""
    fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    logging.basicConfig(
        stream=sys.stdout,
        level=level,
        format=fmt,
        datefmt="%H:%M:%S",
    )


def format_time(seconds: float) -> str:
    """Format seconds as MM:SS."""
    m = int(seconds // 60)
    s = int(seconds % 60)
    return f"{m:02d}:{s:02d}"


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation."""
    return a + (b - a) * clamp(t, 0.0, 1.0)


def euclidean(a: Cell, b: Cell) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def manhattan(a: Cell, b: Cell) -> float:
    return abs(b[0] - a[0]) + abs(b[1] - a[1])


def path_total_cost(path: List[Cell]) -> float:
    """Sum of Euclidean distances between consecutive waypoints."""
    total = 0.0
    for i in range(1, len(path)):
        total += euclidean(path[i - 1], path[i])
    return round(total, 3)


def battery_colour(pct: float) -> str:
    """Return a hex colour string based on battery percentage."""
    if pct > 60:
        return "#00ff88"    # green
    elif pct > 30:
        return "#ffe066"    # yellow
    elif pct > 15:
        return "#ff8c00"    # orange
    else:
        return "#ff3a3a"    # red


def threat_colour(threat_name: str) -> str:
    mapping = {
        "NONE"    : "#00ff88",
        "WARNING" : "#ffe066",
        "CRITICAL": "#ff3a3a",
    }
    return mapping.get(threat_name, "#ffffff")
