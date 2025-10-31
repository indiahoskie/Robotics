#!/usr/bin/env python3
"""
Follow Me 1D (forward/back only)
- Keeps a set distance to the object in front using LiDAR.
- P-control with deadband and output clamping.
- No rotation commanded, strictly 1D motion along x.
"""

import time
import math
import sys
import numpy as np
from mbot_bridge.api import MBot

# ======================= Tunables =======================
SETPOINT_M     = 0.60   # desired front distance (meters) — change if you want
DEADBAND_M     = 0.03   # no motion if |error| <= deadband
KP             = 1.2    # proportional gain
V_MAX_FWD      = 0.40   # max forward speed (m/s)
V_MAX_BACK     = -0.35  # max reverse speed (m/s) — negative
LOOP_HZ        = 20     # control loop frequency
FRONT_WINDOW   = 6      # rays taken from each end of scan (near 0 rad)
# =======================================================

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# ---- Front distance helper (robust to noise/outliers) ----
def front_distance(ranges, thetas, window=FRONT_WINDOW):
    """
    Estimate forward distance using rays near 0 rad.
    Uses projection onto x-axis and median for robustness.
    """
    if not ranges or not thetas or len(ranges) != len(thetas):
        return math.inf

    n = len(ranges)
    win = min(window, n // 2)

    # typical scans are ordered [-pi, +pi); near 0 rad are at the ends
    fwd_r = np.array(list(ranges[:win] + ranges[-win:]), dtype=float)
    fwd_t = np.array(list(thetas[:win] + thetas[-win:]), dtype=float)

    # keep valid, positive ranges
    mask = np.isfinite(fwd_r) & (fwd_r > 0.0) & np.isfinite(fwd_t)
    if not np.any(mask):
        return math.inf

    proj = fwd_r[mask] * np.cos(fwd_t[mask])  # project to robot x-axis
    if proj.size == 0:
        return math.inf

    # median resists leg/edge outliers
    return float(np.median(proj))

# ---- Read LiDAR with several common API names ----
def read_lidar(robot):
    """
    Try several method names to get (ranges, thetas).
    Return (ranges, thetas) or (None, None) if no new scan.
    """
    candidates = [
        "read_lidar",        # (ranges, thetas)
        "readLidar",         # (ranges, thetas)
        "get_lidar_scan",    # object with .ranges and .thetas
        "read_lidar_scan",   # object with .ranges and .thetas
        "get_scan",          # (ranges, thetas) or object
        "readScan",          # ...
    ]
    for name in candidates:
        if hasattr(robot, name):
            fn = getattr(robot, name)
            try:
                scan = fn()
            except TypeError:
                # Some APIs are non-callable properties
                scan = getattr(robot, name)

            # Normalize to two lists
            if scan is None:
                return (None, None)

            if isinstance(scan, tuple) and len(scan) == 2:
                ranges, thetas = scan
                return (list(ranges), list(thetas))
            # If it's an object with attributes
            if hasattr(scan, "ranges") and hasattr(scan, "thetas"):
                return (list(scan.ranges), list(scan.thetas))
    # Couldn’t find a compatible method
    return (None, None)

# ---- Drive adapters: strictly x only, no rotation ----
class Driver:
    def __init__(self, robot):
        self.robot = robot
        # Discover a usable drive call
        if hasattr(robot, "drive"):
            # try arity 3 (vx, vy, wz) or 2 (vx, wz)
            try:
                robot.drive(0.0, 0.0, 0.0)
                self.mode = "drive_vx_vy_wz"
            except Exception:
                try:
                    robot.drive(0.0, 0.0)
                    self.mode = "drive_vx_wz"
                except Exception:
                    self.mode = None
        else:
            self.mode = None

        # Alternatives
        if self.mode is None:
            if hasattr(robot, "setVelocity"):
                self.mode = "setVelocity"
            elif hasattr(robot, "set_cmd_vel"):
                self.mode = "set_cmd_vel"
            elif hasattr(robot, "stop"):
                self.mode = "stop_only"
            else:
                self.mode = "none"

    def send(self, vx):
        """Send forward/back speed only. No rotation."""
        try:
            if self.mode == "drive_vx_vy_wz":
                self.robot.drive(float(vx), 0.0, 0.0)
            elif self.mode == "drive_vx_wz":
                self.robot.drive(float(vx), 0.0)
            elif self.mode == "setVelocity":
                self.robot.setVelocity(float(vx), 0.0, 0.0)
            elif self.mode == "set_cmd_vel":
                self.robot.set_cmd_vel(float(vx), 0.0)
            elif self.mode == "stop_only":
                if abs(vx) < 1e-6:
                    self.robot.stop()
                else:
                    # can’t move without a drive API — fail closed
                    self.robot.stop()
            else:
                # No known API — fail closed
                pass
        except Exception:
            # Never spin out if a call fails
            try:
                self.stop()
            except Exception:
                pass

    def stop(self):
        """Best-effort full stop."""
        try:
            if hasattr(self.robot, "stop"):
                self.robot.stop()
            else:
                # Try common drive variants with zeros
                if hasattr(self.robot, "drive"):
                    try: self.robot.drive(0.0, 0.0, 0.0)
                    except Exception:
                        try: self.robot.drive(0.0, 0.0)
                        except Exception: pass
                if hasattr(self.robot, "setVelocity"):
                    try: self.robot.setVelocity(0.0, 0.0, 0.0)
                    except Exception: pass
                if hasattr(self.robot, "set_cmd_vel"):
                    try: self.robot.set_cmd_vel(0.0, 0.0)
                    except Exception: pass
        except Exception:
            pass

def main():
    robot = MBot()
    driver = Driver(robot)
    dt = 1.0 / LOOP_HZ

    try:
        while True:
            ranges, thetas = read_lidar(robot)
            if ranges is None or thetas is None:
                # no new scan — hold position
                driver.send(0.0)
                time.sleep(dt)
                continue

            d_front = front_distance(ranges, thetas)

            # Compute P-control with deadband
            vx_cmd = 0.0
            if math.isfinite(d_front):
                error = SETPOINT_M - d_front
                if abs(error) > DEADBAND_M:
                    vx_cmd = KP * error
                    vx_cmd = clamp(vx_cmd, V_MAX_BACK, V_MAX_FWD)
                else:
                    vx_cmd = 0.0
            else:
                # No good reading — stop
                vx_cmd = 0.0

            # Strictly forward/back
            driver.send(vx_cmd)

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Log the error to stderr so you can show your TA exactly what failed
        print(f"[ERROR] {type(e).__name__}: {e}", file=sys.stderr)
    finally:
        driver.stop()
        time.sleep(0.2)

if __name__ == "__main__":
    main()
