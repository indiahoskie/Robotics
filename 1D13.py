# 1D_Follower.py
# Behavior:
# - Move straight (no turning) to maintain a target distance from the nearest obstacle/wall.
# - Bang-Bang: forward if too far, reverse if too close, stop inside margin.
# - P-Control: smooth velocity proportional to distance error (optional).

import time
import math
import inspect

# If your course uses this exact import, keep it:
from mbot_bridge.api import MBot

# ------------------------ TUNABLES ------------------------
MODE = "BANG"           # "BANG" for Bang-Bang (as in slides), or "P" for P-control
SETPOINT = 0.50         # meters; desired distance to the nearest wall/object
MARGIN   = 0.05         # meters; acceptable window around setpoint to STOP (Bang-Bang)
VEL_FWD  = 0.25         # m/s forward speed (Bang-Bang)
VEL_REV  = -0.22        # m/s reverse speed (Bang-Bang)
LOOP_DT  = 0.10         # seconds; control loop period

# P-control settings (used if MODE == "P")
KP       = 0.9          # proportional gain (m/s per meter of error)
V_MAX    = 0.35         # clamp max forward speed (m/s)
V_MIN    = -0.30        # clamp max reverse speed (m/s)
DEAD_BAND = 0.02        # meters around setpoint to treat as "close enough" (P-mode)
# ---------------------------------------------------------

bot = MBot()

# --- Choose a 1D drive adaptor: drive_x(vx) that only commands linear x ---
def _make_drive_x():
    """
    Returns a function drive_x(vx) that only moves along the robot's x-axis
    (no turning). It adapts to available MBot APIs:
      - bot.drive(vx, vy, wz)
      - bot.drive(vx, wz)
      - bot.motors(v_l, v_r)  (approximate linear mapping)
    """
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            print("[INFO] Using drive(vx, vy, wz) -> 1D along x")
            return lambda vx: bot.drive(vx, 0.0, 0.0)
        elif n == 2:
            print("[INFO] Using drive(vx, wz) -> 1D along x, zero yaw")
            return lambda vx: bot.drive(vx, 0.0)
    if hasattr(bot, "motors"):
        print("[INFO] Using motors(v_l, v_r) -> equal wheels for straight 1D")
        # NOTE: units may differ (ticks vs m/s). Keep small values if needed.
        return lambda vx: bot.motors(vx, vx)
    raise AttributeError("No suitable drive function found on MBot.")
drive_x = _make_drive_x()

# --- Robust nearest-distance getter (handles different scan formats) ---
def nearest_distance_meters():
    """
    Returns the nearest obstacle distance in meters (float).
    Tries multiple scan formats + findMinDist if available.
    """
    scan = bot.get_lidar_scan()

    # If your SDK provided a helper like findMinDist(scan), use it:
    if hasattr(bot, "findMinDist"):
        try:
            d = bot.findMinDist(scan)
            if isinstance(d, (int, float)) and math.isfinite(d) and d > 0:
                return d
        except Exception:
            pass

    # Otherwise, parse common formats:
    # 1) dict with 'ranges'
    if isinstance(scan, dict) and "ranges" in scan:
        rng = [r for r in scan["ranges"] if isinstance(r, (int, float)) and math.isfinite(r) and r > 0]
        return min(rng) if rng else float("inf")

    # 2) list/tuple of floats
    if isinstance(scan, (list, tuple)) and scan and isinstance(scan[0], (int, float)):
        rng = [r for r in scan if isinstance(r, (int, float)) and math.isfinite(r) and r > 0]
        return min(rng) if rng else float("inf")

    # 3) list/tuple of (angle, distance) pairs
    if isinstance(scan, (list, tuple)) and scan and isinstance(scan[0], (list, tuple)) and len(scan[0]) >= 2:
        rng = [p[1] for p in scan if isinstance(p[1], (int, float)) and math.isfinite(p[1]) and p[1] > 0]
        return min(rng) if rng else float("inf")

    # Unknown format
    return float("inf")

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# ------------------------ CONTROL LOOP ------------------------
print(f"[INFO] Mode={MODE}  Setpoint={SETPOINT:.2f} m  (Bang-Bang margin={MARGIN:.2f} m | P: Kp={KP})")
try:
    while True:
        dist = nearest_distance_meters()

        # If LIDAR gave nothing useful, don't move.
        if not math.isfinite(dist) or dist <= 0 or dist == float("inf"):
            # Safety: stop if we can't see
            drive_x(0.0)
            time.sleep(LOOP_DT)
            continue

        # ERROR defined as (desired - current)
        error = SETPOINT - dist

        if MODE.upper() == "BANG":
            # Bang-Bang per
