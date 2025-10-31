import time
import numpy as np
import inspect

# Constants
SETPOINT = 0.25   # meters - desired distance from obstacle
TOLERANCE = 0.05  # meters - acceptable range around setpoint
SPEED = 0.20      # base linear speed (m/s)
LOOP_DT = 0.10    # control period (s)
K_TURN = 1.5      # mapping from "desired vy" to yaw rate when only 2-DOF drive exists
MAX_WZ = 1.2      # max yaw rate (rad/s)

# ---------- Your helpers (kept) ----------
def find_min_dist(lidar_scan):
    """Find the minimum value in a lidar scan vector."""
    if not lidar_scan or len(lidar_scan) == 0:
        return None
    return min(lidar_scan)

def find_min_nonzero_dist(lidar_scan):
    """Find the minimum value in a lidar scan, ignoring zeros (bad readings)."""
    if not lidar_scan or len(lidar_scan) == 0:
        return None
    nonzero_distances = [d for d in lidar_scan if d > 0]
    if len(nonzero_distances) == 0:
        return None
    return min(nonzero_distances)

# ---------- Minimal compatibility additions ----------
def make_lidar_reader(bot):
    """
    Return a function that yields (distances, angles) or (None, None) if no data yet.
    Tries your professor's patterns first, then common MBot fallbacks.
    """
    # 1) Your professor's names (as in your code)
    if hasattr(bot, 'get_lidar_scan') and callable(getattr(bot, 'get_lidar_scan')):
        def _read():
            try:
                d, a = bot.get_lidar_scan()
                return d, a
            except Exception as e:
                if "no data on channel" in str(e).lower():
                    return None, None
                return None, None
        return _read

    if hasattr(bot, 'get_scan') and callable(getattr(bot, 'get_scan')):
        def _read():
            try:
                scan = bot.get_scan()
                if scan is None:
                    return None, None
                if isinstance(scan, dict):
                    d = scan.get('distances') or scan.get('ranges') or scan.get('range')
                    a = scan.get('angles')    or scan.get('angle')
                    return d, a
                # Some APIs return (distances, angles)
                try:
                    d, a = scan
                    return d, a
                except:
                    return scan, None
            except Exception as e:
                if "no data on channel" in str(e).lower():
                    return None, None
                return None, None
        return _read

    # 2) Other common variants
    for name in ('read_lidar', 'get_scan', 'get_laser', 'get_range_scan'):
        if hasattr(bot, name) and callable(getattr(bot, name)):
            fn = getattr(bot, name)
            def _read(fn=fn):
                try:
                    scan = fn()
                    if scan is None:
                        return None, None
                    if isinstance(scan, dict):
                        d = scan.get('distances') or scan.get('ranges') or scan.get('range')
                        a = scan.get('angles')    or scan.get('angle')
                        return d, a
                    try:
                        d, a = scan
                        return d, a
                    except:
                        return scan, None
                except Exception as e:
                    if "no data on channel" in str(e).lower():
                        return None, None
                    return None, None
            return _read

    # 3) Generic bus-style read("lidar")
    if hasattr(bot, 'read') and callable(getattr(bot, 'read')):
        def _read_generic():
            for ch in ('lidar', 'LIDAR', 'lidar_scan', 'scan'):
                try:
                    scan = bot.read(ch)
                    if scan is None:
                        continue
                    if isinstance(scan, dict):
                        d = scan.get('distances') or scan.get('ranges') or scan.get('range')
                        a = scan.get('angles')    or scan.get('angle')
                        return d, a
                    try:
                        d, a = scan
                        return d, a
                    except:
                        return scan, None
                except Exception as e:
                    if "no data on channel" in str(e).lower():
                        return None, None
                    continue
            return None, None
        return _read_generic

    # If nothing matched, just return no data
    return lambda: (None, None)

def make_drive_adapter(bot):
    """
    Return a function drive3(vx, vy, wz) that adapts to:
      - bot.drive(vx, vy, wz)
      - bot.drive(vx, wz)
      - bot.set_vel(v, w)
      - bot.motors(l, r)
      - or no-op if none exist
    """
    # Full holonomic
    if hasattr(bot, 'drive'):
        try:
            n = len(inspect.signature(bot.drive).parameters)
        except Exception:
            n = 3
        if n >= 3:
            def drive3(vx, vy, wz):
                bot.drive(float(vx), float(vy), float(wz))
            return drive3
        elif n == 2:
            # Map vy into yaw rate; vx stays linear x
            def drive3(vx, vy, wz):
                w_cmd = np.clip(K_TURN * vy, -MAX_WZ, MAX_WZ) if wz == 0 else float(wz)
                bot.drive(float(vx), float(w_cmd))
            return drive3

    if hasattr(bot, 'set_vel'):
        def drive3(vx, vy, wz):
            # Approx: use forward speed from vector norm, yaw from desired lateral
            v = float(np.hypot(vx, vy))
            w = float(np.clip(K_TURN * np.arctan2(vy, max(1e-6, vx)), -MAX_WZ, MAX_WZ)) if (vx or vy) else 0.0
            if wz != 0:
                w = float(wz)
            bot.set_vel(v, w)
        return drive3

    if hasattr(bot, 'motors'):
        def drive3(vx, vy, wz):
            # Very simple diff-drive mapping:
            v = float(np.hypot(vx, vy))
            w = float(np.clip(K_TURN * np.arctan2(vy, max(1e-6, vx)), -MAX_WZ, MAX_WZ))
            if wz != 0:
                w = float(wz)
            # Convert (v,w) ~ (v +/- k*w)
            k = 0.25  # wheelbase factor (rough)
            left  = v - k * w
            right = v + k * w
            bot.motors(left, right)
        return drive3

    # Fallback no-op
    def drive3(vx, vy, wz):
        pass
    return drive3

def make_stop(bot, drive3):
    def _stop():
        try:
            if hasattr(bot, 'stop'):
                bot.stop()
                return
        except:
            pass
        drive3(0.0, 0.0, 0.0)
    return _stop

# ---------- Your main logic with tiny glue ----------
def follow_2d_loop(bot):
    """Follow 2D: maintains setpoint distance from nearest obstacle using lidar scan."""
    read_lidar = make_lidar_reader(bot)
    drive3 = make_drive_adapter(bot)
    stop = make_stop(bot, drive3)

    while True:
        # Get lidar scan
        distances, angles = read_lidar()

        if distances is None or len(distances) == 0:
            stop()
            time.sleep(LOOP_DT)
            continue

        # Ensure numpy arrays
        dists = np.array(distances, dtype=float)
        angs  = np.array(angles, dtype=float) if angles is not None else None

        # Find minimum nonzero distance and its angle
        min_dist = None
        min_angle = 0.0
        for i, dist in enumerate(dists):
            if dist > 0:
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                    if angs is not None and i < len(angs):
                        min_angle = float(angs[i])
                    else:
                        # If no angles, assume "front" direction
                        min_angle = 0.0

        if min_dist is None:
            stop()
            time.sleep(LOOP_DT)
            continue

        # Maintain setpoint distance (bang-bang around tolerance band)
        if min_dist < SETPOINT - TOLERANCE:
            # Too close: move away from obstacle along its bearing
            vx = -SPEED * np.cos(min_angle)
            vy = -SPEED * np.sin(min_angle)
            drive3(vx, vy, 0.0)
        elif abs(min_dist - SETPOINT) <= TOLERANCE:
            # At setpoint: stop
            stop()
        else:
            # Too far: move toward obstacle along its bearing
            vx =  SPEED * np.cos(min_angle)
            vy =  SPEED * np.sin(min_angle)
            drive3(vx, vy, 0.0)

        # Debug print
        print(f"min_dist={min_dist:.3f} m  target={SETPOINT:.3f} m  angle={min_angle:.2f} rad")
        time.sleep(LOOP_DT)

# ---------- Example usage ----------
# from mbot_bridge.api import MBot
# bot = MBot()
# try:
#     follow_2d_loop(bot)
# except KeyboardInterrupt:
#     pass
