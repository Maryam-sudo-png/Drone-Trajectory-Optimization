"""
astar.py — A* Pathfinding Algorithm Implementation
===================================================
Design and Analysis of Algorithms — Core Module

A* searches the graph of grid cells using:
    f(n) = g(n) + h(n)

where
    g(n) = exact cost from start → n   (path cost so far)
    h(n) = admissible heuristic n → goal (never overestimates)
    f(n) = estimated total cost through n

Time  Complexity : O(b^d)  worst case (b = branching factor, d = depth)
                   O(E log V) with a binary-heap open set (grid: E≈8V)
Space Complexity : O(V) for the closed/open sets — V = GRID_WIDTH × GRID_HEIGHT

Reference: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968).
"""

import heapq
import math
from typing import List, Tuple, Optional, Dict, Set
from config import ALLOW_DIAGONAL, HEURISTIC

# ─── Type aliases ─────────────────────────────────────────────────────────────
Cell  = Tuple[int, int]
Path  = List[Cell]


# ──────────────────────────────────────────────────────────────────────────────
class Node:
    """
    Represents a single cell in the A* search tree.

    Attributes
    ----------
    pos   : (col, row) grid coordinate
    g     : cost from start to this node
    h     : heuristic estimate to goal
    f     : g + h
    parent: previous node on the best path found so far
    """

    __slots__ = ("pos", "g", "h", "f", "parent")

    def __init__(self, pos: Cell, g: float = 0.0, h: float = 0.0,
                 parent: Optional["Node"] = None):
        self.pos    = pos
        self.g      = g
        self.h      = h
        self.f      = g + h
        self.parent = parent

    # Heap ordering is by f; break ties with h (prefer nodes closer to goal)
    def __lt__(self, other: "Node") -> bool:
        return (self.f, self.h) < (other.f, other.h)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Node) and self.pos == other.pos

    def __hash__(self) -> int:
        return hash(self.pos)

    def __repr__(self) -> str:
        return f"Node({self.pos}, g={self.g:.2f}, h={self.h:.2f})"


# ──────────────────────────────────────────────────────────────────────────────
class AStarPlanner:
    """
    Grid-based A* path planner with pluggable heuristics.

    Parameters
    ----------
    width, height   : grid dimensions in cells
    heuristic       : "euclidean" | "manhattan" | "chebyshev"
    allow_diagonal  : whether 8-connected movement is allowed
    """

    # Straight and diagonal movement costs
    STRAIGHT_COST  = 1.0
    DIAGONAL_COST  = math.sqrt(2)          # ≈ 1.414

    def __init__(self,
                 width: int,
                 height: int,
                 heuristic: str      = HEURISTIC,
                 allow_diagonal: bool = ALLOW_DIAGONAL):
        self.width          = width
        self.height         = height
        self.heuristic_name = heuristic
        self.allow_diagonal = allow_diagonal

        # Choose heuristic function
        self._heuristic_fn = {
            "euclidean" : self._euclidean,
            "manhattan" : self._manhattan,
            "chebyshev" : self._chebyshev,
        }.get(heuristic, self._euclidean)

        # Stats exposed after each search
        self.nodes_expanded : int = 0
        self.nodes_generated: int = 0
        self.path_length    : float = 0.0

    # ── Heuristic functions ───────────────────────────────────────────────────

    @staticmethod
    def _euclidean(a: Cell, b: Cell) -> float:
        """Straight-line distance — optimal for 8-connected grids."""
        return math.hypot(b[0] - a[0], b[1] - a[1])

    @staticmethod
    def _manhattan(a: Cell, b: Cell) -> float:
        """L1 distance — optimal for 4-connected grids."""
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    @staticmethod
    def _chebyshev(a: Cell, b: Cell) -> float:
        """Chebyshev (L∞) distance — uniform-cost 8-connected."""
        return max(abs(b[0] - a[0]), abs(b[1] - a[1]))

    # ── Neighbour generation ──────────────────────────────────────────────────

    def _neighbours(self, pos: Cell, obstacles: Set[Cell]) -> List[Tuple[Cell, float]]:
        """
        Return walkable neighbours of *pos* with their step costs.

        Diagonal moves are skipped when either adjacent cardinal neighbour
        is blocked (corner-cutting prevention).
        """
        cx, cy = pos
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),           # cardinal
        ]
        if self.allow_diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]   # diagonal

        result: List[Tuple[Cell, float]] = []
        blocked = obstacles  # set for O(1) lookup

        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy

            # Boundary check
            if not (0 <= nx < self.width and 0 <= ny < self.height):
                continue

            neighbour = (nx, ny)
            if neighbour in blocked:
                continue

            # Prevent diagonal corner-cutting
            if dx != 0 and dy != 0:
                if (cx + dx, cy) in blocked or (cx, cy + dy) in blocked:
                    continue
                cost = self.DIAGONAL_COST
            else:
                cost = self.STRAIGHT_COST

            result.append((neighbour, cost))

        return result

    # ── Main search ───────────────────────────────────────────────────────────

    def find_path(self,
                  start    : Cell,
                  goal     : Cell,
                  obstacles: Set[Cell]) -> Optional[Path]:
        """
        Run A* from *start* to *goal* avoiding *obstacles*.

        Returns the path as a list of (col, row) cells from start (inclusive)
        to goal (inclusive), or None if no path exists.

        Complexity
        ----------
        Time : O(E log V)  where E ≈ 8V for 8-connected grids
        Space: O(V)        open set + closed set
        """
        # Reset statistics
        self.nodes_expanded  = 0
        self.nodes_generated = 1
        self.path_length     = 0.0

        # Guard: start/goal must not be inside an obstacle
        if start in obstacles or goal in obstacles:
            return None

        # ── Data structures ──────────────────────────────────────────────────
        # open_heap : min-heap of Node objects ordered by f
        # open_map  : pos → best g seen (for O(1) duplicate detection)
        # closed    : set of expanded positions
        open_heap: List[Node] = []
        open_map : Dict[Cell, float] = {}
        closed   : Set[Cell] = set()

        start_node = Node(
            pos = start,
            g   = 0.0,
            h   = self._heuristic_fn(start, goal),
        )
        heapq.heappush(open_heap, start_node)
        open_map[start] = 0.0

        # ── Search loop ──────────────────────────────────────────────────────
        while open_heap:
            current = heapq.heappop(open_heap)

            # Skip stale heap entries (lazy deletion)
            if current.pos in closed:
                continue
            if open_map.get(current.pos, float("inf")) < current.g:
                continue

            closed.add(current.pos)
            self.nodes_expanded += 1

            # ── Goal test ────────────────────────────────────────────────────
            if current.pos == goal:
                path = self._reconstruct(current)
                self.path_length = current.g
                return path

            # ── Expand neighbours ────────────────────────────────────────────
            for neighbour_pos, step_cost in self._neighbours(current.pos, obstacles):
                if neighbour_pos in closed:
                    continue

                tentative_g = current.g + step_cost

                if tentative_g >= open_map.get(neighbour_pos, float("inf")):
                    continue   # not an improvement

                h = self._heuristic_fn(neighbour_pos, goal)
                child = Node(
                    pos    = neighbour_pos,
                    g      = tentative_g,
                    h      = h,
                    parent = current,
                )
                heapq.heappush(open_heap, child)
                open_map[neighbour_pos] = tentative_g
                self.nodes_generated += 1

        # Open set exhausted — no path
        return None

    # ── Path reconstruction ───────────────────────────────────────────────────

    @staticmethod
    def _reconstruct(node: Node) -> Path:
        """Walk parent pointers from goal back to start, then reverse."""
        path: Path = []
        current: Optional[Node] = node
        while current is not None:
            path.append(current.pos)
            current = current.parent
        path.reverse()
        return path

    # ── Utility helpers ───────────────────────────────────────────────────────

    def path_cost(self, path: Path) -> float:
        """Return the sum of step costs along *path*."""
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total += self.DIAGONAL_COST if (dx != 0 and dy != 0) else self.STRAIGHT_COST
        return total

    def smooth_path(self, path: Path, obstacles: Set[Cell]) -> Path:
        """
        Greedy line-of-sight path smoother (string-pulling).

        Removes unnecessary waypoints by checking if the drone can travel
        directly between non-adjacent nodes without hitting an obstacle.
        Reduces the number of waypoints and produces more natural flight arcs.
        """
        if len(path) < 3:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Try to reach as far ahead as possible in a straight line
            j = len(path) - 1
            while j > i + 1:
                if self._line_of_sight(path[i], path[j], obstacles):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j

        return smoothed

    def _line_of_sight(self, a: Cell, b: Cell, obstacles: Set[Cell]) -> bool:
        """
        Bresenham line-of-sight check between two cells.
        Returns True if no obstacle cell falls on the line segment a→b.
        """
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if (x0, y0) in obstacles:
                return False
            if x0 == x1 and y0 == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0  += sx
            if e2 < dx:
                err += dx
                y0  += sy

    def complexity_report(self) -> Dict:
        """Return a dict summarising the last search's complexity metrics."""
        return {
            "nodes_expanded"  : self.nodes_expanded,
            "nodes_generated" : self.nodes_generated,
            "path_length_cells": round(self.path_length, 3),
            "heuristic"       : self.heuristic_name,
            "diagonal_moves"  : self.allow_diagonal,
            "time_complexity" : "O(E log V)",
            "space_complexity": "O(V)",
        }
