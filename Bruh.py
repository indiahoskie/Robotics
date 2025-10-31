#!/usr/bin/env python3
import time
import math
import numpy as np
from mbot_bridge.api import MBot

# =================== TUNABLES ===================
SETPOINT = 0.60          # meters: target distance from the wall/object in front
TOLERANCE = 0.05         # meters: deadband to reduce oscillations
KP = 1.2                 # proportional gain (lower if oscillates; try 0.8–1.5)
MAX_SPEED = 0.35         # m/s cap for safety
MIN_SPEED = 0.02         # m/s; under this we send 0 to avoid jitter
FRONT_WINDOW = 6         # LiDAR rays near 0 rad to average
LOSS_TIMEOUT = 1.0       # seconds: if no valid LiDAR for this long, stop
PRINT_EVERY = 5          # print debug every N loops
DT = 0.1                 # loop time (s) ~10 Hz
# =================================================


# Get distance to wall
def find_fwd_dist(ranges, thetas, window=FRONT_WINDOW):
    """Find the distance to the nearest object in front of the robot.

    Args:
        ranges (list): The ranges from the Lidar scan.
        thetas (list): The angles from the Lidar scan.
        window (int, optional): The window to average ranges over. Defaults to 5.

    Returns:
        float: The distance to the nearest obstacle in front of the robot.
               Returns np.nan if no valid rays.
    """
    # Guard against empty scans
    if not ranges or not thetas:
        return np.nan

    # Grab the rays near the front of the scan.
    fwd_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    fwd_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)

    # Grab just the positive values.
    valid_idx = (np.isfinite(fwd_ranges) & (fwd_ranges > 0)).nonzero()
    if len(valid_idx[0]) == 0:
        return np.nan

    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]

    # Compute forward distances (project onto robot's forward axis).
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    fwd_dists = fwd_dists[fwd_dists > 0]  # keep forward-only projections
    if fwd_dists.size == 0:
        return np.nan

    return float(np.mean(fwd_dists))  # Return the mean.


# --- Small motion helpers so this works with multiple MBot APIs ---
_last_mode = None  # remembers which command signature succeeded last time

def _send_vx(robot, vx):
    """Send a forward/backward velocity command in the simplest possible way.
       Tries drive(vx,0,0), then drive(vx,0), then motors(vx,vx)."""
    global _last_mode

    # deadband
    if abs(vx) < MIN_SPEED:
        vx = 0.0

    # If a mode already worked, try it first for speed
    try:
        if _last_mode == "drive3":
            robot.drive(vx, 0.0, 0.0); return "drive3"
        elif _last_mode == "drive2":
            robot.drive(vx, 0.0); return "drive2"
        elif _last_mode == "motors":
            robot.motors(vx, vx); return "motors"
    except Exception:
        pass  # fall through to probing

    # Probe available signatures
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

    return None  # no known motion API

def _stop_robot(robot):
    """Stop safely regardless of which API is present."""
    try:
        robot.stop()
        return
    except Exception:
        pass
    # If no stop(), try zeroing whatever motion API we found
    if _last_mode == "drive3":
        try: robot.drive(0.0, 0.0, 0.0)
        except Exception: pass
    elif _last_mode == "drive2":
        try: robot.drive(0.0, 0.0)
        except Exception: pass
    elif _last_mode == "motors":
        try: robot.motors(0.0, 0.0)
        except Exception: pass


# Initialize a robot object.
robot = MBot()
setpoint = SETPOINT  # ✅ Filled: choose setpoint above.

try:
    # Loop forever.
    last_ok = time.time()
    loop = 0

    while True:
        # Read the latest Lidar scan.
        try:
            ranges, thetas = robot.read_lidar()
        except AttributeError:
            print("[ERROR] MBot.read_lidar() not found. Check your SDK name.")
            break

        # Get the distance to the wall in front of the robot.
        dist_to_wall = find_fwd_dist(ranges, thetas)

        # --- ✅ TODO IMPLEMENTED: Follow-me 1D P-Controller ---
        # If LiDAR invalid for too long, stop for safety.
        now = time.time()
        if not (isinstance(dist_to_wall, (float, int)) and math.isfinite(dist_to_wall)):
            # invalid reading this cycle
            if (now - last_ok) > LOSS_TIMEOUT:
                _stop_robot(robot)
                if loop % PRINT_EVERY == 0:
                    print("[WARN] No valid forward LiDAR distance; holding position.")
                time.sleep(DT)
                loop += 1
                continue
        else:
            last_ok = now

        # Compute error so that positive error -> move forward, negative -> move back
        # (i.e., if we're farther than desired, go forward to close the gap)
        error = dist_to_wall - setpoint

        # Deadband for stability
        if abs(error) <= TOLERANCE:
            vx_cmd = 0.0
        else:
            vx_cmd = KP * error
            # Saturate speed
            vx_cmd = max(-MAX_SPEED, min(MAX_SPEED, vx_cmd))

        # Send velocity command (adapts to available MBot API)
        mode_used = _send_vx(robot, vx_cmd)
        if mode_used is None:
            print("[ERROR] No known motion API (drive/motors) available on MBot.")
            break

        # Debug prints every N loops
        if loop % PRINT_EVERY == 0:
            # Handle NaN pretty-print
            dshow = float(dist_to_wall) if math.isfinite(dist_to_wall) else float('nan')
            print(f"[DBG] mode={mode_used} | dist={dshow:.3f} m | set={setpoint:.3f} m | "
                  f"err={error if math.isfinite(error) else float('nan'):.3f} | "
                  f"vx={vx_cmd:.3f} m/s")

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(DT)
        loop += 1

except KeyboardInterrupt:
    print("\n[INFO] CTRL-C received, stopping.")
except Exception as e:
    print(f"[ERROR] Exception: {e}")
finally:
    _stop_robot(robot)
    print("[INFO] Robot stopped.")
