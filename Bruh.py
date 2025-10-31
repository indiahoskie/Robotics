#!/usr/bin/env python3
import time
import math
import numpy as np
from mbot_bridge.api import MBot

# =================== TUNABLES ===================
SETPOINT = 0.60          # meters: target distance from wall/object ahead
TOLERANCE = 0.05         # meters: deadband around setpoint
KP = 1.2                 # proportional gain (lower if it oscillates)
MAX_SPEED = 0.35         # m/s safety cap
MIN_SPEED = 0.02         # m/s; treat smaller as zero
FRONT_WINDOW = 6         # # of LiDAR rays near forward to average
LOSS_TIMEOUT = 1.0       # s; if no valid LiDAR this long, stop
DT = 0.1                 # s loop period (~10 Hz)
# =================================================

# Get distance to wall (forward-only projection)
def find_fwd_dist(ranges, thetas, window=FRONT_WINDOW):
    """Find forward distance using rays near 0 rad; returns np.nan if invalid."""
    if not ranges or not thetas:
        return np.nan

    fwd_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    fwd_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)

    valid = np.isfinite(fwd_ranges) & (fwd_ranges > 0.0)
    if not np.any(valid): 
        return np.nan

    fwd_ranges = fwd_ranges[valid]
    fwd_thetas = fwd_thetas[valid]

    # Project onto robot's forward axis
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    fwd_dists = fwd_dists[fwd_dists > 0.0]
    if fwd_dists.size == 0:
        return np.nan

    return float(np.mean(fwd_dists))

# ---------- STRICT 1D MOTION HELPERS (no rotation) ----------
_last_mode = None  # remember which API worked

def _send_vx(robot, vx):
    """Send pure forward/backward command. No rotation is ever commanded."""
    global _last_mode

    # Deadband
    if abs(vx) < MIN_SPEED:
        vx = 0.0

    # Try previously working mode first
    try:
        if _last_mode == "drive3":
            robot.drive(vx, 0.0, 0.0)  # vx only; vy=0, wz=0
            return "drive3"
        elif _last_mode == "drive2":
            robot.drive(vx, 0.0)       # vx only; wz=0
            return "drive2"
        elif _last_mode == "motors":
            robot.motors(vx, vx)       # equal wheels => no rotation
            return "motors"
    except Exception:
        pass

    # Probe signatures (still 1D only)
    try:
        robot.drive(vx, 0.0, 0.0); _last_mode = "drive3"; return "drive3"
    except Exception:
        pass
    try:
        robot.drive(vx, 0.0); _last_mode = "drive2"; return "drive2"
    except Exception:
        pass
    if hasattr(robot, "motors"):
        try:
            robot.motors(vx, vx); _last_mode = "motors"; return "motors"
        except Exception:
            pass

    return None

def _stop(robot):
    """Stop with zero rotation regardless of API."""
    try:
        robot.stop(); return
    except Exception:
        pass
    try:
        if _last_mode == "drive3": robot.drive(0.0, 0.0, 0.0)
        elif _last_mode == "drive2": robot.drive(0.0, 0.0)
        elif _last_mode == "motors": robot.motors(0.0, 0.0)
    except Exception:
        pass
# -------------------------------------------------------------

# Initialize robot and setpoint
robot = MBot()
setpoint = SETPOINT

try:
    last_ok = time.time()
    loop = 0

    while True:
        # Read LiDAR
        try:
            ranges, thetas = robot.read_lidar()
        except AttributeError:
            print("[ERROR] MBot.read_lidar() not found in this SDK.")
            break

        # Forward distance
        dist = find_fwd_dist(ranges, thetas)

        # Safety if LiDAR lost
        now = time.time()
        if not (isinstance(dist, (float, int)) and math.isfinite(dist)):
            if (now - last_ok) > LOSS_TIMEOUT:
                _stop(robot)
                if loop % 5 == 0:
                    print("[WARN] No valid forward LiDAR; holding still.")
                time.sleep(DT); loop += 1; continue
        else:
            last_ok = now

        # 1D P-control: positive error => move forward; negative => move backward
        error = dist - setpoint

        if abs(error) <= TOLERANCE:
            vx = 0.0
        else:
            vx = KP * error
            vx = max(-MAX_SPEED, min(MAX_SPEED, vx))

        # Send pure 1D command (no rotation)
        mode = _send_vx(robot, vx)
        if mode is None:
            print("[ERROR] No known motion API (drive or motors) available.")
            break

        if loop % 5 == 0:
            d = dist if math.isfinite(dist) else float('nan')
            print(f"[DBG] mode={mode} | dist={d:.3f} m | set={setpoint:.3f} m | err={error:.3f} | vx={vx:.3f} m/s")

        time.sleep(DT)
        loop += 1

except KeyboardInterrupt:
    print("\n[INFO] CTRL-C received, stopping.")
except Exception as e:
    print(f"[ERROR] {e}")
finally:
    _stop(robot)
    print("[INFO] Robot stopped.")

