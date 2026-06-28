"""
dashboard.py — Streamlit Web Dashboard
Adaptive Real-Time Drone Trajectory Optimization Framework
"""

import streamlit as st
import time
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from environment import SimEnvironment
from config import DEFAULT_START, DEFAULT_GOAL, GRID_WIDTH, GRID_HEIGHT

st.set_page_config(
    page_title="Drone Trajectory Optimizer",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.markdown("""
<style>
body { background-color: #0d0f1a; color: #c8d8f0; }
.metric-container { background: #141728; border: 1px solid #1e2d45; border-radius: 8px; padding: 12px; margin: 4px 0; }
.stButton>button { background: #0d1a2e; border: 1px solid #2563eb; color: #00c8ff; font-weight: 600; border-radius: 6px; width: 100%; }
.stButton>button:hover { background: #1a2d4e; }
</style>
""", unsafe_allow_html=True)

# ── Session state init ────────────────────────────────────────────────────────
if "env" not in st.session_state:
    st.session_state.env = None
if "running" not in st.session_state:
    st.session_state.running = False
if "log" not in st.session_state:
    st.session_state.log = []

# ── Sidebar ───────────────────────────────────────────────────────────────────
with st.sidebar:
    st.title("🚁 Mission Control")
    st.markdown("---")

    heuristic = st.selectbox("A* Heuristic", ["euclidean", "manhattan", "chebyshev"])
    speed = st.slider("Simulation Speed", 1, 8, 3)
    weather = st.selectbox("Weather Mode", ["Clear", "Windy", "Foggy", "Stormy"])
    steps_per_frame = st.slider("Steps per refresh", 5, 60, 20)

    st.markdown("---")
    col1, col2 = st.columns(2)
    with col1:
        start_btn = st.button("▶ Launch")
    with col2:
        reset_btn = st.button("↺ Reset")

    st.markdown("---")
    st.markdown("**A\\* Complexity**")
    st.markdown("`Time : O(E log V)`")
    st.markdown("`Space: O(V)`")
    st.markdown("`V = 60 × 40 = 2,400`")

# ── Main area ─────────────────────────────────────────────────────────────────
st.title("Adaptive Drone Trajectory Optimization")
st.caption("DAA Project · A* Pathfinding · Real-time Collision Avoidance")

# Metrics row
m1, m2, m3, m4, m5, m6 = st.columns(6)
bat_ph   = m1.empty()
pos_ph   = m2.empty()
coll_ph  = m3.empty()
rep_ph   = m4.empty()
prog_ph  = m5.empty()
state_ph = m6.empty()

# Canvas placeholder
canvas_ph = st.empty()

# Log
log_ph = st.empty()

# ── Button actions ────────────────────────────────────────────────────────────
if reset_btn:
    st.session_state.env = None
    st.session_state.running = False
    st.session_state.log = ["System reset."]

if start_btn:
    env = SimEnvironment(heuristic=heuristic, seed=None)
    env.set_speed(speed)
    env.set_weather(weather)
    ok = env.start_simulation()
    if ok:
        st.session_state.env = env
        st.session_state.running = True
        st.session_state.log = [f"Mission launched! Path: {len(env.path)} nodes", f"Heuristic: {heuristic}"]
    else:
        st.error("No path found — try resetting.")

# ── Simulation loop ───────────────────────────────────────────────────────────
def draw_grid(env):
    """Draw the simulation grid as an SVG string."""
    CS = 11
    W = GRID_WIDTH * CS
    H = GRID_HEIGHT * CS

    parts = [f'<svg width="{W}" height="{H}" xmlns="http://www.w3.org/2000/svg" style="background:#080c14;border-radius:8px">']

    # Grid lines
    for c in range(0, GRID_WIDTH+1, 5):
        parts.append(f'<line x1="{c*CS}" y1="0" x2="{c*CS}" y2="{H}" stroke="#111827" stroke-width="0.3"/>')
    for r in range(0, GRID_HEIGHT+1, 5):
        parts.append(f'<line x1="0" y1="{r*CS}" x2="{W}" y2="{r*CS}" stroke="#111827" stroke-width="0.3"/>')

    # Old paths
    for op in env.old_paths[-2:]:
        if len(op) > 1:
            pts = " ".join(f"{p[0]*CS+CS//2},{p[1]*CS+CS//2}" for p in op)
            parts.append(f'<polyline points="{pts}" fill="none" stroke="#1a3a6e" stroke-width="1.2"/>')

    # Current path
    if len(env.path) > 1:
        pts = " ".join(f"{p[0]*CS+CS//2},{p[1]*CS+CS//2}" for p in env.path)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="#2563eb" stroke-width="1.8" stroke-dasharray="5,3"/>')

    # Trail
    if len(env.drone.trail) > 1:
        pts = " ".join(f"{p[0]*CS+CS//2},{p[1]*CS+CS//2}" for p in env.drone.trail[-80:])
        parts.append(f'<polyline points="{pts}" fill="none" stroke="rgba(0,200,255,0.3)" stroke-width="1"/>')

    # Static obstacles
    for o in env.obstacles.static_obstacles:
        cx, cy, r = o.cx*CS+CS//2, o.cy*CS+CS//2, max(3, o.radius*CS-1)
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="rgba(255,58,58,0.7)" stroke="#ff3a3a" stroke-width="0.8"/>')

    # Dynamic obstacles
    for o in env.obstacles.dynamic_obstacles:
        cx = o.x*CS+CS//2; cy = o.y*CS+CS//2; r = max(3, o.radius*CS-1)
        parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="rgba(255,140,0,0.85)" stroke="#ff8c00" stroke-width="1"/>')
        # velocity vector
        vx2 = cx + o.vx*CS*7; vy2 = cy + o.vy*CS*7
        parts.append(f'<line x1="{cx:.1f}" y1="{cy:.1f}" x2="{vx2:.1f}" y2="{vy2:.1f}" stroke="rgba(255,140,0,0.5)" stroke-width="0.8"/>')

    # Goal
    gx, gy = GOAL[0]*CS+CS//2, GOAL[1]*CS+CS//2
    parts.append(f'<circle cx="{gx}" cy="{gy}" r="{CS}" fill="rgba(0,255,136,0.2)" stroke="#00ff88" stroke-width="1.5"/>')
    parts.append(f'<text x="{gx}" y="{gy+4}" text-anchor="middle" fill="#00ff88" font-size="9" font-weight="bold">G</text>')

    # Warning zone
    threat = env.collision._last_threat.name
    if threat != "NONE":
        dx = env.drone.x*CS+CS//2; dy = env.drone.y*CS+CS//2
        col = "rgba(255,58,58,0.08)" if threat=="CRITICAL" else "rgba(255,224,102,0.07)"
        bcol = "rgba(255,58,58,0.5)" if threat=="CRITICAL" else "rgba(255,224,102,0.4)"
        wr = 5*CS
        parts.append(f'<circle cx="{dx:.1f}" cy="{dy:.1f}" r="{wr}" fill="{col}" stroke="{bcol}" stroke-width="0.8" stroke-dasharray="3,3"/>')

    # Drone
    dx = env.drone.x*CS+CS//2; dy = env.drone.y*CS+CS//2; ds = CS*0.9
    parts.append(f'<circle cx="{dx:.1f}" cy="{dy:.1f}" r="{ds*2}" fill="rgba(0,200,255,0.15)"/>')
    parts.append(f'<circle cx="{dx:.1f}" cy="{dy:.1f}" r="{ds}" fill="#00c8ff" stroke="white" stroke-width="0.8"/>')
    for ax, ay in [(1,1),(-1,1),(1,-1),(-1,-1)]:
        ex = dx+ax*ds*1.4; ey = dy+ay*ds*1.4
        parts.append(f'<line x1="{dx:.1f}" y1="{dy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="rgba(0,200,255,0.7)" stroke-width="0.8"/>')
        parts.append(f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="{ds*0.4}" fill="rgba(0,200,255,0.6)"/>')

    parts.append('</svg>')
    return "".join(parts)

GOAL = DEFAULT_GOAL

if st.session_state.running and st.session_state.env:
    env = st.session_state.env
    env.set_speed(speed)
    env.set_weather(weather)

    for _ in range(steps_per_frame):
        env.step()
        if env.finished:
            st.session_state.running = False
            st.session_state.log.insert(0, f"✅ Goal reached! Frames: {env.frame}")
            break

    m = env.full_metrics()
    dr = m["drone"]
    ca = m["collision"]

    bat   = dr["battery"]
    bat_c = "🟢" if bat > 50 else "🟡" if bat > 25 else "🔴"
    bat_ph.metric("Battery", f"{bat_c} {bat:.1f}%")
    pos_ph.metric("Position", f"({dr['position'][0]:.1f}, {dr['position'][1]:.1f})")
    coll_ph.metric("Collisions", ca["collision_count"])
    rep_ph.metric("Replans", ca["replan_count"])
    prog_ph.metric("Progress", f"{dr['progress_pct']:.0f}%")
    state_ph.metric("State", dr["state"])

    canvas_ph.markdown(draw_grid(env), unsafe_allow_html=True)

    # log
    log_ph.text_area("Event Log", "\n".join(st.session_state.log[:20]), height=120)

    if st.session_state.running:
        time.sleep(0.05)
        st.rerun()
else:
    # Show static grid
    if st.session_state.env:
        canvas_ph.markdown(draw_grid(st.session_state.env), unsafe_allow_html=True)
    else:
        canvas_ph.info("Press **▶ Launch** in the sidebar to start the simulation.")
    log_ph.text_area("Event Log", "\n".join(st.session_state.log[:20]), height=120)
