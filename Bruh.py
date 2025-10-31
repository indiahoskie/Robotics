#!/usr/bin/env python3
"""
Follow Me 1D Script
This script makes the robot maintain a setpoint distance from a wall/object in front using lidar.
"""

import time
import numpy as np
import sys
import inspect
from mbot_bridge.api import MBot

# Constants
SETPOINT   = 0.25   # meters - desired distance from wall
TOLERANCE  = 0.05   # meters - acceptable range around setpoint
FWD_SPEED  = 0.20   # m/s when moving forward
REV_SPEED  = -0.20  # m/s when backing up
LOOP_DT    = 0.10   # seconds

def get_front_range_m(ranges, thetas, window=5):
    """
    Get front range reading from lidar sensor.
    Returns distance in meters or None if reading unavailable.
    """
    if not ranges or not thetas or len(ranges) == 0:
        return None

    # Use rays near 0 rad (front): first/last 'window' samples
    front_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    front_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)

    # Valid positive distances
    valid = front_ranges > 0
    if not np.any(valid):
        return None

    # Forward projection on robot x-axis
    fwd = front_ranges[valid] * np.cos(front_thetas[valid])
    fwd = fwd[fwd > 0]  # only forward
    if fwd.size == 0:
        return None

    return float(np.mean(fwd))

def make_set_linear(bot):
    """
    Return a function set_vx(vx) to command linear x only.
    Tries drive(vx,vy,wz), then drive(vx,wz), then set_vel(v,w), then motors(l,r).
    """
    if hasattr(bot, "drive") and callable(getattr(bot, "drive")):
        try:
            n = len(inspect.signature(bot.drive).parameters)
        except Exception:
            n = 3
        if n >= 3:
            print("[INFO] Drive via bot.drive(vx, vy, wz)")
            return lambda vx: bot.drive(float(vx), 0.0, 0.0)
        elif n == 2:
            print("[INFO] Drive via bot.drive(vx, wz)")
            return lambda vx: bot.drive(float(vx), 0.0)

    if hasattr(bot, "set_vel") and callable(getattr(bot, "set_vel")):
        print("[INFO] Drive via bot.set_vel(v, w)")
        return lambda vx: bot.set_vel(float(vx), 0.0)

    if hasattr(bot, "motors") and callable(getattr(bot, "motors")):
        print("[INFO] Drive via bot.motors(l, r)")
        return lambda vx: bot.motors(float(vx), float(vx))

    print("[WARN] No known drive API; velocity commands will be ignored.")
    return lambda vx: None

def safe_stop(bot, set_vx):
    try:
        if hasattr(bot, "stop"):
            bot.stop()
            return
    except Exception:
        pass
    set_vx(0.0)

def read_lidar_pairs(bot):
    """
    Read lidar as (ranges, thetas).
    - Supports read_lidar() returning tuple or dict
    - Falls back to bot.read('lidar'/'LIDAR'/'lidar_scan'/'scan')
    Returns (ranges, thetas) or (None, None).
    """
    # 1) Preferred API
    if hasattr(bot, "read_lidar") and callable(getattr(bot, "read_lidar")):
        try:
            scan = bot.read_lidar()
            if scan is None:
                return None, None
            if isinstance(scan, dict):
                ranges = scan.get('ranges') or scan.get('range')
                thetas = scan.get('angles') or scan.get('angle')
                return ranges, thetas
            else:
                try:
                    ranges, thetas = scan
                    return ranges, thetas
                except Exception:
                    return None, None
        except Exception as e:
            if "no data on channel" in str(e).lower():
                return None, None
            # fall through

    # 2) Generic bus read
    if hasattr(bot, "read") and callable(getattr(bot, "read")):
        for ch in ("lidar", "LIDAR", "lidar_scan", "scan"):
            try:
                scan = bot.read(ch)
                if scan is None:
                    continue
                if isinstance(scan, dict):
                    ranges = scan.get('ranges') or scan.get('range') or scan.get('distances')
                    thetas = scan.get('angles') or scan.get('angle')
                    return ranges, thetas
                else:
                    try:
                        ranges, thetas = scan
                        return ranges, thetas
                    except Exception:
                        return None, None
            except Exception as e:
                if "no data on channel" in str(e).lower():
                    return None, None
                continue

    return None, None

def follow_wall_loop(robot):
    """
    Drives forward by default. If the front distance is closer than SETPOINT,
    back up. Stop within the tolerance band.
    """
    set_vx = make_set_linear(robot)

    while True:
        ranges, thetas = read_lidar_pairs(robot)

        if not ranges or not thetas:
            # No lidar yet: hold still and retry (safer than blind driving)
            set_vx(0.0)
            time.sleep(LOOP_DT)
            continue

        dist = get_front_range_m(ranges, thetas)

        if dist is None or not np.isfinite(dist):
            set_vx(0.0)
            time.sleep(LOOP_DT)
            continue

        # --- Bang-Bang behavior (1D) ---
        if dist < SETPOINT - TOLERANCE:
            # Too close -> back up
            set_vx(REV_SPEED)
            state = "BACKING"
        elif dist > SETPOINT + TOLERANCE:
            # Too far -> go forward
            set_vx(FWD_SPEED)
            state = "FORWARD"
        else:
            # Within band -> stop
            set_vx(0.0)
            state = "STOP"

        print(f"[1D] dist={dist:.3f} m | target={SETPOINT:.3f}±{TOLERANCE:.3f} | state={state}")
        time.sleep(LOOP_DT)

def main():
    """Main function to run the follow 1D controller."""
    robot = None
    try:
        print("Connecting to MBot...")
        robot = MBot()
        time.sleep(1.0)
        print("Starting follow 1D controller...")
        follow_wall_loop(robot)

    except KeyboardInterrupt:
        print("\nStopping robot...")
        if robot:
            safe_stop(robot, make_set_linear(robot))
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        if robot:
            safe_stop(robot, make_set_linear(robot))
        sys.exit(1)

if __name__ == "__main__":
    main()
