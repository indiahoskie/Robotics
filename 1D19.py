import time
import numpy as np
from mbot_bridge.api import MBot
import math
import inspect

# Get distance to wall
def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot."""
    if not ranges or not thetas:
        return float('inf')

    # Guard for short scans
    w = max(1, min(window, len(ranges)//2 if len(ranges) >= 2 else 1))

    # Grab the rays near the front of the scan.
    fwd_ranges = np.array(list(ranges[:w]) + list(ranges[-w:]), dtype=float)
    fwd_thetas = np.array(list(thetas[:w]) + list(thetas[-w:]), dtype=float)

    # Grab just the positive values.
    valid_idx = (fwd_ranges > 0.0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')

    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]

    # Compute forward distances.
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists)) if fwd_dists.size else float('inf')


# ----- small adapter so this works whether you have set_vel, drive, or motors -----
def send_speed(bot, vx):
    # Try set_vel(linear, angular)
    if hasattr(bot, "set_vel"):
        try:
            bot.set_vel(float(vx), 0.0)
            return
        except Exception:
            pass
    # Try drive(vx, vy, wz) or drive(vx, wz)
    if hasattr(bot, "drive"):
        try:
            bot.drive(float(vx), 0.0, 0.0)  # 3-arg
            return
        except TypeError:
            try:
                bot.drive(float(vx), 0.0)   # 2-arg
                return
            except Exception:
                pass
    # Fallback: differential motors
    if hasattr(bot, "motors"):
        try:
            bot.motors(float(vx), float(vx))
            return
        except Exception:
            pass
    # If nothing works, do nothing safely


def stop_robot(bot):
    try:
        if hasattr(bot, "set_vel"):
            bot.set_vel(0.0, 0.0)
        elif hasattr(bot, "drive"):
            try:
                bot.drive(0.0, 0.0, 0.0)
            except TypeError:
                bot.drive(0.0, 0.0)
        elif hasattr(bot, "motors"):
            bot.motors(0.0, 0.0)
        else:
            bot.stop()
    except Exception:
        pass


# Initialize a robot object.
robot = MBot()
setpoint = 1.0  # Target distance to maintain from wall (in meters)

# Controller parameters
Kp = 2.0        # Proportional gain - how aggressively to respond to error
max_speed = 0.5 # Maximum robot speed
min_speed = 0.1 # Minimum robot speed to keep moving

try:
    # Loop forever.
    while True:
        # Read the latest Lidar scan.
        scan = robot.read_lidar()
        # Support dict or tuple returns
        if isinstance(scan, dict):
            ranges = scan.get('range') or scan.get('ranges') or []
            thetas = scan.get('angle') or scan.get('angles') or []
        else:
            ranges, thetas = scan

        # Get the distance to the wall in front of the robot.
        dist_to_wall = find_fwd_dist(ranges, thetas)

        # If invalid reading, stop and retry
        if not np.isfinite(dist_to_wall) or math.isinf(dist_to_wall):
            send_speed(robot, 0.0)
            time.sleep(0.1)
            continue

        # --- SIGN FIX: positive error => too far (go forward). Negative => too close (back up).
        error = dist_to_wall - setpoint

        # Simple proportional controller
        control_signal = Kp * error

        # Clamp the speed to reasonable limits
        speed = float(np.clip(control_signal, -max_speed, max_speed))

        # Apply minimum speed threshold to avoid getting stuck (unless we're basically at setpoint)
        if abs(speed) < min_speed and abs(error) > 0.1:
            speed = min_speed if speed > 0 else -min_speed

        # Set robot velocity (1D only)
        send_speed(robot, speed)

        # Print status for debugging
        print(f"Distance: {dist_to_wall:.2f}m, Error: {error:+.2f}m, Speed: {speed:+.2f}m/s")

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
except Exception as e:
    print(f"[ERROR] {e}")
finally:
    # Catch any exception, including the user quitting, and stop the robot.
    stop_robot(robot)
    print("\n[INFO] Controller stopped.")
