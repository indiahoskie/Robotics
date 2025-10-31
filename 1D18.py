import time
import math
import numpy as np
import inspect
from mbot_bridge.api import MBot

# -------------------- TUNABLES (adjust for your robot) --------------------
SETPOINT      = 1.00    # meters: desired distance to wall/obstacle
MARGIN        = 0.05    # meters: hysteresis band around setpoint (±MARGIN)
FWD_SPEED     = 0.25    # m/s when too far (drive forward)
REV_SPEED     = -0.22   # m/s when too close (drive backward)
SEARCH_SPEED  = 0.15    # m/s when no object detected yet (drive forward to find one)
LOOP_HZ       = 10      # control loop rate
WINDOW        = 5       # beams from each edge used to estimate "forward" distance
# -------------------------------------------------------------------------

# ---- Helper: robust 1D motion (no turning) regardless of API shape ----
def make_move_and_stop(bot):
    """
    Returns (move_1d, stop) where:
      - move_1d(vx) sends linear velocity vx (m/s), with no yaw
      - stop() halts the robot safely
    Works with drive(vx,vy,wz), drive(vx,wz), or motors(l,r).
    """
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            def move_1d(vx): bot.drive(float(vx), 0.0, 0.0)
            def stop():      bot.drive(0.0, 0.0, 0.0)
            return move_1d, stop
        elif n == 2:
            def move_1d(vx): bot.drive(float(vx), 0.0)
            def stop():      bot.drive(0.0, 0.0)
            return move_1d, stop
    if hasattr(bot, "motors"):
        def move_1d(vx): bot.motors(float(vx), float(vx))
        def stop():      bot.motors(0.0, 0.0)
        return move_1d, stop
    # Safe no-ops if nothing matches
    def move_1d(_vx): pass
    def stop():        pass
    return move_1d, stop

# ---- Your forward-distance estimator (kept in your style) ----
def find_fwd_dist(ranges, thetas, window=WINDOW):
    """
    Estimate forward distance by averaging beams closest to "straight ahead".
    Assumes scan arrays wrap around; uses first/last 'window' beams.
    Returns float('inf') if no valid readings.
    """
    if not ranges or not thetas:
        return float('inf')

    # Guard for short scans
    w = max(1, min(int(window), len(ranges)//2 if len(ranges) >= 2 else 1))

    fwd_ranges = np.array(list(ranges[:w]) + list(ranges[-w:]), dtype=float)
    fwd_thetas = np.array(list(thetas[:w]) + list(thetas[-w:]), dtype=float)

    # Keep only positive distances
    valid_idx = (fwd_ranges > 0.0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')

    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]

    # Project onto robot-forward axis
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists)) if fwd_dists.size else float('inf')

# ---- Main program ----
robot = MBot()
move_1d, stop_robot = make_move_and_stop(robot)

try:
    dt = 1.0 / float(LOOP_HZ)
    print(f"[INFO] 1D Bang-Bang: setpoint={SETPOINT:.2f} m  margin=±{MARGIN:.2f} m")

    while True:
        # Read the latest LiDAR scan.
        scan = robot.read_lidar()

        # Support (ranges,thetas) or dict {'range':...,'angle':...}
        if isinstance(scan, dict):
            ranges = scan.get('range') or scan.get('ranges') or []
            thetas = scan.get('angle') or scan.get('angles') or []
        else:
            ranges, thetas = scan  # your original structure

        # Compute forward distance estimate
        dist_to_wall = find_fwd_dist(ranges, thetas, window=WINDOW)

        # If no valid target yet, drive forward to "find" something
        if not np.isfinite(dist_to_wall) or math.isinf(dist_to_wall):
            move_1d(SEARCH_SPEED)
            print(f"dist=∞  state=SEARCH  vx={SEARCH_SPEED:+.2f} m/s")
            time.sleep(dt)
            continue

        # Bang-Bang Logic (thermostat-style)
        error = dist_to_wall - SETPOINT  # + => too far; - => too close

        if error > MARGIN:
            # Too far from wall → move forward
            vx = FWD_SPEED
            state = "TOO_FAR -> FORWARD"
        elif error < -MARGIN:
            # Too close to wall → move backward
            vx = REV_SPEED
            state = "TOO_CLOSE -> BACKWARD"
        else:
            # Inside acceptable band → stop
            vx = 0.0
            state = "IN_BAND -> HOLD"

        move_1d(vx)
        print(f"dist={dist_to_wall:.2f} m  err={error:+.2f} m  vx={vx:+.2f} m/s  {state}")

        time.sleep(dt)

except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"[ERROR] {e}")
finally:
    stop_robot()
    print("\n[INFO] Stopped 1D Bang-Bang controller.")
