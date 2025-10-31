import time
import numpy as np
import inspect
from mbot_bridge.api import MBot

# ------------------------ CONFIG ------------------------
SETPOINT = 1.0     # meters: target distance to obstacle
KP = 2.0
MAX_SPEED = 0.5
MIN_SPEED = 0.1
LOOP_HZ = 10
NO_DATA_SLEEP = 0.1   # seconds to wait when no lidar data yet
# --------------------------------------------------------

# ---------- Helpers ----------
def find_fwd_dist(ranges, thetas, window=5):
    """Mean forward distance using first/last 'window' rays."""
    if not ranges or not thetas:
        return float('inf')
    fwd_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    fwd_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)
    valid_idx = (fwd_ranges > 0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists))

def make_drive_adapter(bot):
    """Match whatever drive/motors signature your MBot has."""
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:  # drive(vx, vy, wz)
            def move(vx):
                bot.drive(vx, 0.0, 0.0)
            return move
        elif n == 2:  # drive(vx, wz)
            def move(vx):
                bot.drive(vx, 0.0)
            return move
    if hasattr(bot, "motors"):
        def move(vx):
            bot.motors(vx, vx)
        return move
    # Safe no-op if nothing found
    def move(_vx): 
        pass
    return move

def make_lidar_reader(bot):
    """
    Return a function that yields (ranges, thetas) or None if no data.
    Tries multiple common MBot APIs and channel styles.
    """
    candidates = [
        "read_lidar",
        "read_scan",
        "read_laser",
        "read_range_scan",
        "get_lidar",
        "get_scan",
    ]

    # Direct method candidates
    for name in candidates:
        if hasattr(bot, name) and callable(getattr(bot, name)):
            fn = getattr(bot, name)
            def reader_direct():
                try:
                    scan = fn()
                    if scan is None:
                        return None
                    if isinstance(scan, dict):
                        ranges = scan.get("range") or scan.get("ranges")
                        thetas = scan.get("angle") or scan.get("angles")
                    else:
                        ranges, thetas = scan
                    if ranges is None or thetas is None:
                        return None
                    return list(ranges), list(thetas)
                except Exception as e:
                    # If the driver isn't up yet, we just say "no data" and retry
                    if "no data on channel" in str(e).lower():
                        return None
                    # Other errors should still surface so you see what's wrong
                    raise
            print(f"[INFO] Using lidar reader via bot.{name}()")
            return reader_direct

    # Generic bus-style read("lidar") fallback (some APIs expose a generic reader)
    for ch in ("lidar", "LIDAR", "lidar_scan", "scan"):
        if hasattr(bot, "read") and callable(getattr(bot, "read")):
            def reader_generic(channel=ch):
                try:
                    scan = bot.read(channel)  # may raise if no data yet
                    if scan is None:
                        return None
                    if isinstance(scan, dict):
                        ranges = scan.get("range") or scan.get("ranges")
                        thetas = scan.get("angle") or scan.get("angles")
                    else:
                        # Support tuple or (ranges, thetas) object
                        ranges, thetas = scan
                    if ranges is None or thetas is None:
                        return None
                    return list(ranges), list(thetas)
                except Exception as e:
                    if "no data on channel" in str(e).lower():
                        return None
                    raise
            print(f"[INFO] Using lidar reader via bot.read('{ch}')")
            return reader_generic

    # If nothing matches, provide a reader that returns None
    print("[WARN] No known LiDAR read method found on MBot; will keep waiting for data.")
    return lambda: None

# ---------- Main ----------
robot = MBot()
move = make_drive_adapter(robot)
read_lidar = make_lidar_reader(robot)

dt = 1.0 / LOOP_HZ
print("[INFO] 1D distance-hold starting... Ctrl-C to stop.")

try:
    while True:
        data = read_lidar()
        if not data:
            # No LiDAR yet: stop and retry
            move(0.0)
            time.sleep(NO_DATA_SLEEP)
            continue

        ranges, thetas = data
        dist_to_wall = find_fwd_dist(ranges, thetas)

        if not np.isfinite(dist_to_wall):
            move(0.0)
            time.sleep(dt)
            continue

        # P-control (positive error = too far -> forward; negative = too close -> reverse)
        error = dist_to_wall - SETPOINT
        cmd = float(np.clip(KP * error, -MAX_SPEED, MAX_SPEED))
        if abs(cmd) < MIN_SPEED and abs(error) > 0.1:
            cmd = MIN_SPEED if cmd > 0 else -MIN_SPEED

        move(cmd)
        print(f"Distance: {dist_to_wall:.2f} m | Error: {error:+.2f} m | Speed: {cmd:+.2f} m/s")
        time.sleep(dt)

except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"[ERROR] {e}")
finally:
    move(0.0)
    print("\n[INFO] Stopped 1D hold.")
