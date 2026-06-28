"""
main.py — Entry Point
======================
Adaptive Real-Time Drone Trajectory Optimization
and Collision Avoidance Framework

Run modes
---------
  python main.py               → launch the web dashboard (default)
  python main.py --headless    → run simulation headlessly and print metrics
  python main.py --test-astar  → run A* unit tests and print results
"""

import argparse
import sys
import time
import logging

from utils       import setup_logging
from environment import SimEnvironment
from config      import (
    GRID_WIDTH, GRID_HEIGHT,
    DEFAULT_START, DEFAULT_GOAL,
    FPS,
)

setup_logging(logging.INFO)
log = logging.getLogger("main")


# ──────────────────────────────────────────────────────────────────────────────
def run_headless(steps: int = 3000, verbose: bool = True) -> dict:
    """
    Run the full simulation without any GUI.
    Useful for automated testing and benchmarking.
    """
    log.info("=== Headless Simulation Start ===")
    env = SimEnvironment(
        start      = DEFAULT_START,
        goal       = DEFAULT_GOAL,
        heuristic  = "euclidean",
        use_airsim = False,
    )

    ok = env.start_simulation()
    if not ok:
        log.error("Could not find initial path. Aborting.")
        sys.exit(1)

    t0 = time.perf_counter()
    for frame in range(steps):
        env.step()
        if verbose and frame % 100 == 0:
            m = env.full_metrics()
            print(
                f"  Frame {frame:04d} | "
                f"Pos {m['drone']['position']} | "
                f"Battery {m['drone']['battery']:.1f}% | "
                f"Replans {m['collision']['replan_count']} | "
                f"Progress {m['drone']['progress_pct']:.1f}%"
            )
        if env.finished:
            break

    elapsed = time.perf_counter() - t0
    metrics = env.full_metrics()
    metrics["wall_time_s"] = round(elapsed, 3)

    print("\n=== Simulation Complete ===")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    return metrics


# ──────────────────────────────────────────────────────────────────────────────
def run_astar_tests() -> None:
    """Quick self-test for the A* planner."""
    from astar import AStarPlanner

    planner = AStarPlanner(20, 20)

    # Test 1: straight corridor
    path = planner.find_path((0, 0), (19, 19), set())
    assert path is not None, "Test 1 failed: expected path in empty grid"
    assert path[0]  == (0, 0),   "Test 1 failed: wrong start"
    assert path[-1] == (19, 19), "Test 1 failed: wrong goal"
    print(f"✓ Test 1 passed — empty grid, path length: {len(path)}")

    # Test 2: blocked path
    wall = {(i, 10) for i in range(20)}
    path2 = planner.find_path((0, 0), (0, 19), wall)
    assert path2 is None, "Test 2 failed: expected no path through solid wall"
    print("✓ Test 2 passed — solid wall blocks path correctly")

    # Test 3: path around obstacle
    obstacle = {(5, y) for y in range(0, 8)}
    path3 = planner.find_path((0, 5), (10, 5), obstacle)
    assert path3 is not None, "Test 3 failed: expected path around obstacle"
    print(f"✓ Test 3 passed — path around obstacle: {len(path3)} waypoints")

    # Test 4: heuristic variants
    for h in ("euclidean", "manhattan", "chebyshev"):
        p = AStarPlanner(20, 20, heuristic=h)
        path4 = p.find_path((0, 0), (19, 19), set())
        assert path4, f"Test 4 failed for heuristic {h}"
    print("✓ Test 4 passed — all three heuristics produce valid paths")

    print("\nAll A* tests passed ✓")
    print("Complexity report:", planner.complexity_report())


# ──────────────────────────────────────────────────────────────────────────────
def launch_dashboard() -> None:
    """Launch the Streamlit web dashboard."""
    import subprocess
    import os
    script = os.path.join(os.path.dirname(__file__), "dashboard.py")
    log.info("Launching Streamlit dashboard …")
    subprocess.run(
        [sys.executable, "-m", "streamlit", "run", script,
         "--server.headless", "true",
         "--server.port", "8501"],
        check=False,
    )


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Drone Trajectory Optimization & Collision Avoidance"
    )
    parser.add_argument("--headless",   action="store_true",
                        help="Run simulation without GUI")
    parser.add_argument("--test-astar", action="store_true",
                        help="Run A* unit tests")
    parser.add_argument("--steps", type=int, default=3000,
                        help="Max frames in headless mode")
    args = parser.parse_args()

    if args.test_astar:
        run_astar_tests()
    elif args.headless:
        run_headless(steps=args.steps)
    else:
        launch_dashboard()
