import time
import math
import numpy as np
import inspect
from mbot_bridge.api import MBot


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    min_dist, min_angle = None, None
    if ranges is None or thetas is None:
        return None, None

    for r, th in zip(ranges, thetas):
        if r is None:
            continue
        # valid = positive and finite
        if r > 0.0 and np.isfinite(r):
            if (min_dist is None) or (r < min_dist):
                min_dist, min_angle = float(r), float(th)

    return min_dist, min_angle


robot = MBot()
setpoint = 0.80  # desired distance to maintain (meters)

# --- Controller params (tune as needed) ---
Kp        = 1.2   # proportional gain for speed magnitude
deadband  = 0.03  # m: don't move if |error| < deadband
max_speed = 0.50  # m/s cap on |speed|
min_speed = 0.06  # m/s floor when correcting and |error| is meaningful
LOOP_HZ   = 10

# --- Tiny sender that uses whatever drive API exists, but NEVER turns ---
def _send_planar_velocity(bot, vx, vy):
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        try:
            if n >= 3:            # drive(vx, vy, wz)
                bot.drive(float(vx), float(vy), 0.0)
                return
            elif n == 2:          # drive(vx, wz) -> best-effort: project vy into vx
                bot.drive(float(vx), 0.0)
                return
        except TypeError:
            pass
    if hasattr(bot, "motors"):     # fallback: diff-drive best-effort on vx only
        v = float(vx)
        bot.motors(v, v)
        return
    # if no known API, do nothing safely


try:
    while True:
        # Read the latest Lidar scan.
        ranges, thetas = robot.read_lidar()

        # Get the distance and angle to the closest object.
        dist_to_wall, angle_to_wall = find_min_dist(ranges, thetas)

        # If nothing valid, stop and try again.
        if dist_to_wall is None or not np.isfinite(dist_to_wall):
            _send_planar_velocity(robot, 0.0, 0.0)
            time.sleep(1.0 / LOOP_HZ)
            continue

        # --- 2D Follow-Me controller (no turning) ---
        # Positive error => we're too far (move toward object)
        # Negative error => we're too close (move away)
        error = dist_to_wall - setpoint

        if abs(error) < deadband:
            vx_cmd, vy_cmd = 0.0, 0.0
        else:
            # speed magnitude from P-control, clamped
            speed = float(np.clip(Kp * error, -max_speed, max_speed))
            # ensure minimum motion when correcting
            if abs(speed) < min_speed and abs(error) > 0.10:
                speed = math.copysign(min_speed, speed)

            # Move along the ray to the object (angle_to_wall).
            # No yaw commanded; robot slides in plane.
            vx_cmd = speed * math.cos(angle_to_wall)
            vy_cmd = speed * math.sin(angle_to_wall)

        _send_planar_velocity(robot, vx_cmd, vy_cmd)

        # Debug
        print(f"dist={dist_to_wall:.2f}m  err={error:+.2f}m  v=({vx_cmd:+.2f},{vy_cmd:+.2f}) m/s  theta={angle_to_wall:+.2f}rad")

        time.sleep(1.0 / LOOP_HZ)
except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"[ERROR] {e}")
finally:
    # Best-effort stop
    try:
        if hasattr(robot, "drive"):
            try:
                robot.drive(0.0, 0.0, 0.0)
            except TypeError:
                robot.drive(0.0, 0.0)
        elif hasattr(robot, "motors"):
            robot.motors(0.0, 0.0)
        else:
            robot.stop()
    except Exception:
        pass
    print("\n[INFO] 2D follow-me stopped.")
