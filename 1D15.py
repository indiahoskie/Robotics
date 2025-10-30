import time
import numpy as np
from mbot_bridge.api import MBot

# --- Small compatibility shim: always get (ranges, thetas) from read_lidar() ---
def get_ranges_thetas(robot):
    scan = robot.read_lidar()
    # Dict forms: {'range': [...], 'angle': [...] } or similar
    if isinstance(scan, dict):
        ranges = scan.get('range') or scan.get('ranges') or scan.get('r')
        thetas = scan.get('angle') or scan.get('angles') or scan.get('theta') or scan.get('thetas')
        return list(ranges), list(thetas)
    # Pair of arrays/lists: (ranges, thetas) or (thetas, ranges)
    if isinstance(scan, (list, tuple)) and len(scan) == 2:
        a, b = scan
        # Heuristic: angles are typically in [-pi, pi], ranges are >= 0
        try:
            a_min, b_min = float(np.min(a)), float(np.min(b))
            a_max, b_max = float(np.max(a)), float(np.max(b))
        except Exception:
            # Fall back to user's original assumption
            return list(a), list(b)
        # If 'a' looks like angles (small magnitude, possibly negative) and 'b' like ranges (non-negative)
        if (-3.5 <= a_min <= 0.0 or 0.0 <= a_min <= 3.5) and a_max <= 7.0 and b_min >= 0.0:
            return list(b), list(a)  # (ranges, thetas)
        # Otherwise assume original order was (ranges, thetas)
        return list(a), list(b)
    # List of (angle, range) pairs
    if isinstance(scan, (list, tuple)) and scan and isinstance(scan[0], (list, tuple)) and len(scan[0]) == 2:
        angles, ranges = zip(*scan)
        return list(ranges), list(angles)
    # Unknown format -> empty, caller handles it
    return [], []

# Get distance to wall
def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot.

    Args:
        ranges (list): The ranges from the Lidar scan.
        thetas (list): The angles from the Lidar scan.
        window (int, optional): The window to average ranges over. Defaults to 5.

    Returns:
        float: The distance to the nearest obstacle in front of the robot.
    """
    if not ranges or not thetas:
        return float('inf')
    w = max(1, min(window, len(ranges)//2 if len(ranges) >= 2 else 1))

    # Grab the rays near the front of the scan.
    fwd_ranges = np.array(list(ranges[:w]) + list(ranges[-w:]), dtype=float)
    fwd_thetas = np.array(list(thetas[:w]) + list(thetas[-w:]), dtype=float)

    # Grab just the positive values.
    valid_idx = (fwd_ranges > 0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')

    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]

    # Compute forward distances.
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists))  # Return the mean.


# Initialize a robot object.
robot = MBot()
setpoint = 1.0  # Target distance to maintain from wall (in meters)

# Controller parameters
Kp = 2.0       # Proportional gain - how aggressively to respond to error
max_speed = 0.5  # Maximum robot speed
min_speed = 0.1  # Minimum robot speed to keep moving

try:
    # Loop forever.
    while True:
        # Read the latest Lidar scan and normalize to (ranges, thetas).
        ranges, thetas = get_ranges_thetas(robot)

        # Get the distance to the wall in front of the robot.
        dist_to_wall = find_fwd_dist(ranges, thetas)

        # If we didn't get a valid reading, stop and try again.
        if not np.isfinite(dist_to_wall):
            # Stop safely if available
            try:
                robot.set_vel(0.0, 0.0)
            except Exception:
                try:
                    robot.stop()
                except Exception:
                    pass
            time.sleep(0.1)
            continue

        # Calculate the error (difference between desired and actual distance)
        error = setpoint - dist_to_wall

        # Simple proportional controller
        # If error is positive, we're too close, so move backward
        # If error is negative, we're too far, so move forward
        control_signal = Kp * error

        # Convert control signal to robot speeds
        # Clamp the speed to reasonable limits
        speed = float(np.clip(control_signal, -max_speed, max_speed))

        # Apply minimum speed threshold to avoid getting stuck
        if abs(speed) < min_speed and abs(error) > 0.1:
            speed = min_speed if speed > 0 else -min_speed

        # Set robot velocity
        # Positive speed = forward, negative speed = backward
        try:
            robot.set_vel(speed, 0.0)  # (linear_velocity, angular_velocity)
        except AttributeError:
            # Fallback if your API uses 'drive(vx, vy, wz)' or 'drive(vx, wz)'
            if hasattr(robot, "drive"):
                try:
                    robot.drive(speed, 0.0, 0.0)
                except TypeError:
                    robot.drive(speed, 0.0)
            elif hasattr(robot, "motors"):
                robot.motors(speed, speed)

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
    try:
        robot.stop()
    except Exception:
        try:
            robot.set_vel(0.0, 0.0)
        except Exception:
            try:
                robot.drive(0.0, 0.0, 0.0)
            except Exception:
                pass
    print("\n[INFO] Controller stopped.")

