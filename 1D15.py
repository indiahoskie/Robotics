import time
import math
import inspect
import numpy as np
from mbot_bridge.api import MBot

# -------------------- TUNABLES (1D) --------------------
setpoint   = 1.0   # Target distance to maintain from wall (meters)
Kp         = 2.0   # Proportional gain
max_speed  = 0.5   # Max magnitude of linear speed (m/s)
min_speed  = 0.1   # Minimum speed when correcting (to avoid stall)
loop_hz    = 10    # Control loop rate (Hz)
window     = 5     # Number of beams on each edge used for "front" estimate
# ------------------------------------------------------

robot = MBot()

# ---------- Drive adapter: always 1D (no turning) ----------
def _make_move_and_stop(robot):
    # Preferred: set_vel(linear, angular)
    if hasattr(robot, "set_vel"):
        def move_1d(vx):
            robot.set_vel(float(vx), 0.0)
        def stop():
            try:
                robot.stop()
            except Exception:
                robot.set_vel(0.0, 0.0)
        return move_1d, stop

    # Common: drive(vx, vy, wz) or drive(vx, wz)
    if hasattr(robot, "drive"):
        sig = inspect.signature(robot.drive)
        n = len(sig.parameters)
        if n >= 3:
            def move_1d(vx):
                robot.drive(float(vx), 0.0, 0.0)
            def stop():
                robot.drive(0.0, 0.0, 0.0)
            return move_1d, stop
        elif n == 2:
            def move_1d(vx):
                robot.drive(float(vx), 0.0)
            def stop():
                robot.drive(0.0, 0.0)
            return move_1d, stop

    # Fallback: differential motors(v_l, v_r)
    if hasattr(robot, "motors"):
        def move_1d(vx):
            robot.motors(float(vx), float(vx))
        def stop():
            robot.motors(0.0, 0.0)
        return move_1d, stop

    # Last resort: no-ops (safe)
    def move_1d(_vx): pass
    def stop(): pass
    return move_1d, stop

move_1d, emergency_stop = _make_move_and_stop(robot)

# ---------- LiDAR parser to ensure (ranges, thetas) ----------
def _parse_lidar_to_ranges_thetas(scan):
    """
    Normalize various read_lidar() formats to (ranges, thetas), both Python lists.
    Supported shapes:
      - dict with keys like {'range': [...], 'angle': [...]} or {'ranges': [...], 'thetas': [...]}
      - tuple/list of two arrays: (angles, ranges) or (ranges, angles)
      - list of (angle, range) pairs
    Returns (ranges, thetas) or ([], []) if unknown.
    """
    # Dict form
    if isinstance(scan, dict):
        angles = scan.get('angle') or scan.get('angles') or scan.get('theta') or scan.get('thetas')
        ranges = scan.get('range') or scan.get('ranges') or scan.get('r')
        if ranges is not None and angles is not None:
            return list(ranges), list(angles)

    # Pair of lists form
    if isinstance(scan, (list, tuple)) and len(scan) == 2 and all(isinstance(x, (list, tuple, np.ndarray)) for x in scan):
        a, b = scan

        def looks_like_angles(x):
            try:
                arr = np.asarray(x, dtype=float)
                return np.all(np.isfinite(arr)) and np.max(np.abs(arr)) <= (math.pi * 2.1)
            except Exception:
                return False

        def looks_like_ranges(x):
            try:
                arr = np.asarray(x, dtype=float)
                return np.all(np.isfinite(arr)) and np.min(arr) >= 0.0
            except Exception:
                return False

        # Try to detect ordering
        if looks_like_angles(a) and looks_like_ranges(b):
            return list(b), list(a)  # (ranges, thetas)
        if looks_like_angles(b) and looks_like_ranges(a):
            return list(a), list(b)
        # Fall back to user's original assumption: (ranges, thetas)
        return list(a), list(b)

    # List of (angle, range) pairs
    if isinstance(scan, (list, tuple)) and scan and isinstance(scan[0], (list, tuple)) and len(scan[0]) == 2:
        angles, ranges = zip(*scan)
        return list(ranges), list(angles)

    return [], []

# ---------- Forward distance utility (your original logic) ----------
def find_fwd_dist(ranges, thetas, window=window):
    """
    Estimate forward distance by averaging beams nearest to "front".
    Assumes the scan arrays start near 0 rad and wrap around (common on some APIs):
      - Use first 'window' beams and last 'window' beams.
    """
    if not ranges or not thetas:
        return float('inf')

    w = max(1, int(window))
    # Guard against lists shorter than window
    w = min(w, len(ranges) // 2 if len(ranges) >= 2 else 1)

    fwd_ranges = np.array(list(ranges[:w]) + list(ranges[-w:]), dtype=float)
    fwd_thetas = np.array(list(thetas[:w]) + list(thetas[-w:]), dtype=float)

    # Keep valid positive ranges
    valid = np.where(fwd_ranges > 0.0)[0]
    if valid.size == 0:
        return float('inf')

    fwd_ranges = fwd_ranges[valid]
    fwd_thetas = fwd_thetas[valid]

    # Project onto robot's forward axis
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists))

# ----------------------- Main loop -----------------------
def run_1d_controller():
    dt = 1.0 / float(loop_hz)
    print(f"[INFO] 1D distance-hold using read_lidar(); setpoint={setpoint:.2f} m")

    try:
        while True:
            # Read LiDAR and normalize to (ranges, thetas)
            scan = robot.read_lidar()
            ranges, thetas = _parse_lidar_to_ranges_thetas(scan)

            # Compute forward distance estimate
            dist_to_wall = find_fwd_dist(ranges, thetas, window=window)

            # If no valid reading, stop for safety and try again
            if not np.isfinite(dist_to_wall):
                move_1d(0.0)
                # print("[WARN] No valid LiDAR forward reading; stopping.")
                time.sleep(dt)
                continue

            # P-control on forward distance (1D only)
            error = setpoint - dist_to_wall
            control_signal = Kp * error

            # Clamp and enforce minimum speed when correcting
            speed = float(np.clip(control_signal, -max_speed, max_speed))
            if abs(speed) < min_speed and abs(error) > 0.1:
                speed = math.copysign(min_speed, speed)

            # Command linear speed only (no angular component)
            move_1d(speed)

            # Debug print
            print(f"Distance: {dist_to_wall:.2f} m | Error: {error:+.2f} m | Speed: {speed:+.2f} m/s")

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        emergency_stop()
        print("\n[INFO] Stopped 1D controller.")

if __name__ == "__main__":
    run_1d_controller()
