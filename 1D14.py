import time
import math
import inspect
from mbot_bridge.api import MBot

# -------------------- TUNABLES (1D) --------------------
DESIRED_DIST = 0.60   # meters: target distance straight ahead
HYSTERESIS   = 0.04   # +/- band around target to avoid chatter
FWD_SPEED    = 0.25   # m/s when too far
REV_SPEED    = -0.22  # m/s when too close
LOOP_HZ      = 10     # control loop rate (Hz)
# ------------------------------------------------------

bot = MBot()

# --- Small motion adapter so we NEVER turn (1D only) ---
def _make_drive_adapter(bot):
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            # drive(vx, vy, wz)
            def drive_1d(vx):
                bot.drive(vx, 0.0, 0.0)  # 1D: no lateral, no yaw
            return drive_1d
        elif n == 2:
            # drive(vx, wz)
            def drive_1d(vx):
                bot.drive(vx, 0.0)      # 1D: zero yaw
            return drive_1d
    # Fallback: try motors if available
    if hasattr(bot, "motors"):
        def drive_1d(vx):
            # naive diff-drive mapping (no turn): both wheels same
            bot.motors(vx, vx)
        return drive_1d
    # Last resort: no-op to be safe
    def drive_1d(_vx):
        pass
    return drive_1d

drive_1d = _make_drive_adapter(bot)

def _parse_scan(scan):
    """
    Normalize output of bot.read_lidar() into (angles, ranges) lists.
    Supports:
      - dict like {'angle': [...], 'range': [...]}
      - list/tuple of (angle, range) pairs
    """
    if isinstance(scan, dict) and 'angle' in scan and 'range' in scan:
        return list(scan['angle']), list(scan['range'])
    if isinstance(scan, (list, tuple)) and scan and isinstance(scan[0], (list, tuple)) and len(scan[0]) == 2:
        a, r = zip(*scan)
        return list(a), list(r)
    return [], []

def _valid(r):
    return r is not None and r > 0.0 and not math.isinf(r) and not math.isnan(r)

def read_front_distance():
    """
    Return the LiDAR distance for the beam whose angle is closest to 0 rad.
    If no valid reading, return None.
    """
    scan = bot.read_lidar()          # <<— per your notes / API
    angles, ranges = _parse_scan(scan)
    if not angles or not ranges or len(angles) != len(ranges):
        return None

    # Find index of angle closest to 0 (straight ahead)
    best_i = None
    best_abs_ang = float('inf')
    for i, a in enumerate(angles):
        aa = abs(a)
        if aa < best_abs_ang:
            best_abs_ang = aa
            best_i = i

    if best_i is None:
        return None
    d = ranges[best_i]
    return d if _valid(d) else None

def hold_distance_1d():
    """
    Bang-bang 1D controller:
      - If too far: move forward at FWD_SPEED
      - If too close: move backward at REV_SPEED
      - If within hysteresis band: stop
      - NO turning at any time
    """
    dt = 1.0 / LOOP_HZ
    print(f"[INFO] 1D hold using read_lidar(); target={DESIRED_DIST:.2f} m, hysteresis=±{HYSTERESIS:.2f} m")
    try:
        while True:
            d = read_front_distance()
            if d is None:
                # No safe reading: stop
                drive_1d(0.0)
                # print("[WARN] No valid front LiDAR; stopping.")
                time.sleep(dt)
                continue

            # Bang-bang with hysteresis
            if d > DESIRED_DIST + HYSTERESIS:
                vx = FWD_SPEED          # too far → go forward
            elif d < DESIRED_DIST - HYSTERESIS:
                vx = REV_SPEED          # too close → back up
            else:
                vx = 0.0                # in band → hold

            drive_1d(vx)
            # Optional debug:
            # print(f"d={d:.3f}  -> vx={vx:+.2f}")

            time.sleep(dt)
    except KeyboardInterrupt:
        drive_1d(0.0)
        print("\n[INFO] Stopped 1D hold.")

if __name__ == "__main__":
    hold_distance_1d()
