"""
config.py — Central Configuration for Drone Trajectory Optimization Framework
All constants, thresholds, and tunable parameters live here.
"""

# ─── Grid & World ─────────────────────────────────────────────────────────────
GRID_WIDTH  = 60          # number of cells horizontally
GRID_HEIGHT = 40          # number of cells vertically
CELL_SIZE   = 18          # pixels per cell

# ─── AirSim / Simulation Mapping ──────────────────────────────────────────────
AIRSIM_SCALE      = 2.0   # metres per grid cell (used when real AirSim connected)
DRONE_SPEED       = 3.0   # m/s in AirSim (or cells/s in sim mode)
DRONE_ALTITUDE    = -5.0  # NED z-value (negative = up)

# ─── Start / Goal defaults ────────────────────────────────────────────────────
DEFAULT_START = (2, 2)
DEFAULT_GOAL  = (57, 37)

# ─── A* ───────────────────────────────────────────────────────────────────────
HEURISTIC = "euclidean"   # "euclidean" | "manhattan" | "chebyshev"
ALLOW_DIAGONAL = True

# ─── Collision Avoidance ──────────────────────────────────────────────────────
COLLISION_RADIUS   = 3.0  # grid cells — hard stop + replan
WARNING_RADIUS     = 5.0  # grid cells — pre-emptive replan

# ─── Obstacles ────────────────────────────────────────────────────────────────
NUM_STATIC_OBSTACLES  = 40
NUM_DYNAMIC_OBSTACLES = 8
OBSTACLE_SPEED_RANGE  = (0.04, 0.14)   # cells per frame
OBSTACLE_SIZE_RANGE   = (1, 3)         # radius in cells

# ─── Battery ──────────────────────────────────────────────────────────────────
BATTERY_CAPACITY       = 100.0
BATTERY_MOVE_DRAIN     = 0.03          # per cell traversed
BATTERY_HOVER_DRAIN    = 0.005         # per frame while stopped
BATTERY_REPLAN_PENALTY = 0.8           # extra drain on each replanning event
BATTERY_LOW_THRESHOLD  = 20.0

# ─── Weather ──────────────────────────────────────────────────────────────────
WEATHER_MODES   = ["Clear", "Windy", "Foggy", "Stormy"]
WIND_STRENGTH   = 0.0   # 0.0–1.0 drift factor applied to drone movement

# ─── UI / Colours (hex used only in web dashboard) ────────────────────────────
COLOR_BG          = "#0d0f1a"
COLOR_GRID        = "#1a1d2e"
COLOR_DRONE       = "#00c8ff"
COLOR_GOAL        = "#00ff88"
COLOR_PATH        = "#3a8eff"
COLOR_PATH_OLD    = "#1a3a6e"
COLOR_OBSTACLE_ST = "#ff3a3a"
COLOR_OBSTACLE_DY = "#ff8c00"
COLOR_WARNING     = "#ffe066"
COLOR_TEXT        = "#e0e8ff"
COLOR_PANEL       = "#141728"

# ─── Simulation ───────────────────────────────────────────────────────────────
FPS             = 30
SIM_SPEED_MIN   = 0.5    # speed multiplier
SIM_SPEED_MAX   = 3.0
SIM_SPEED_DEF   = 1.0
