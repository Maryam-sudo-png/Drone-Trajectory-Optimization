# Adaptive Real-Time Drone Trajectory Optimization
## and Collision Avoidance Framework for Dynamic Environments

**Course:** Design and Analysis of Algorithms (DAA)  
**Stack:** Python · Microsoft AirSim · Unreal Engine 4.27 · OpenCV · A\*  

---

## Project Overview

This framework demonstrates **autonomous drone navigation** in a 3-D dynamic environment using the **A\* pathfinding algorithm** enhanced with real-time collision avoidance and adaptive replanning.

### Core DAA Concepts Demonstrated

| Concept | Implementation |
|---|---|
| Graph Traversal | Grid graph with 8-directional adjacency |
| Greedy Heuristic | Euclidean / Manhattan / Chebyshev h(n) |
| Optimal Search | f(n) = g(n) + h(n) — A\* completeness + optimality |
| Dynamic Replanning | Triggered by obstacle proximity or path blockage |
| Time Complexity | O(E log V) with binary-heap open set |
| Space Complexity | O(V) for open + closed sets |

---

## A\* Algorithm Explained

```
f(n) = g(n) + h(n)

g(n) — exact cost from START to node n (path taken so far)
h(n) — admissible heuristic from n to GOAL (never overestimates)
f(n) — estimated total cost through n
```

**Heuristic options:**
- **Euclidean** `h = √((Δx)² + (Δy)²)` — optimal for 8-connected grids
- **Manhattan** `h = |Δx| + |Δy|` — optimal for 4-connected grids  
- **Chebyshev** `h = max(|Δx|, |Δy|)` — uniform-cost 8-connected

**Admissibility:** All three heuristics never overestimate the true cost, guaranteeing that A\* finds the optimal path.

---

## Collision Avoidance

```
Euclidean distance:  d = √((x₂-x₁)² + (y₂-y₁)²)

Threat tiers:
  d ≤ COLLISION_RADIUS (3 cells) → CRITICAL: stop + immediate replan
  d ≤ WARNING_RADIUS   (5 cells) → WARNING:  pre-emptive replan
```

Replanning uses the same A\* planner on the current obstacle snapshot, so the new path is guaranteed to be obstacle-free at the moment of computation.

---

## Project Structure

```
project/
├── main.py               ← Entry point (dashboard / headless / tests)
├── astar.py              ← A* planner with 3 heuristics + path smoother
├── drone_controller.py   ← Flight state machine + AirSim API wrapper
├── collision_avoidance.py← Two-tier threat detection + replan trigger
├── obstacle_manager.py   ← Static & dynamic obstacle generation
├── environment.py        ← Simulation orchestrator
├── utils.py              ← Shared helpers
├── config.py             ← All constants & tunable parameters
├── requirements.txt
└── README.md
```

---

## Installation

### 1. Python environment

```bash
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. AirSim + Unreal Engine 4.27 (optional, for real 3-D simulation)

```bash
# Install AirSim Python client
pip install airsim

# Download a pre-built AirSim environment from:
# https://github.com/microsoft/AirSim/releases

# Launch UE4 environment, then set use_airsim=True in config.py
```

### 3. settings.json for AirSim

Place this in `~/Documents/AirSim/settings.json`:

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0
    }
  }
}
```

---

## Running the Project

```bash
# Interactive web dashboard (default)
python main.py

# Headless benchmark (no GUI)
python main.py --headless --steps 5000

# Unit test A* correctness
python main.py --test-astar
```

---

## Complexity Analysis

### Time Complexity

| Phase | Complexity | Notes |
|---|---|---|
| A\* search | **O(E log V)** | E ≈ 8V for 8-connected grid |
| Path smoothing | O(V²) worst | Greedy line-of-sight; typically O(V) |
| Collision check | O(K) | K = number of dynamic obstacles |
| Per-frame replanning | O(E log V) | Only triggered when needed |

Grid dimensions: V = 60 × 40 = **2,400 nodes**, E ≈ **19,200 edges**.

### Space Complexity

| Structure | Complexity |
|---|---|
| Open set (heap) | O(V) |
| Closed set | O(V) |
| Parent map | O(V) |
| **Total** | **O(V)** |

---

## Key Features

- **Adaptive replanning** — path recalculated whenever an obstacle enters the warning zone or blocks the current path
- **Path smoothing** — Bresenham line-of-sight string-pulling removes unnecessary waypoints
- **Battery system** — movement, hovering, and replanning all consume battery
- **Weather effects** — wind/storm add positional drift to drone movement
- **Predictive collision detection** — look-ahead along the planned path detects future intersections
- **Multi-heuristic support** — switch between Euclidean, Manhattan, Chebyshev at runtime

---

## Demo Flow

1. Drone spawns at (2, 2)
2. A\* computes shortest safe path to (57, 37)
3. Dynamic obstacles move and cross the planned path
4. Warning zone triggers pre-emptive replanning
5. Critical zone triggers emergency stop + immediate replan
6. Drone resumes on new path, reaches goal

---

## AirSim API Calls Used

```python
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
client.moveToPositionAsync(x, y, z, speed).join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
```

---

*COMSATS University Islamabad, Wah Campus — Department of Computer Science*
