import time
import numpy as np
import inspect
from mbot_bridge.api import MBot

# Get distance to wall
def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot."""
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

# Initialize robot
robot = MBot()
setpoint = 1.0   # Target distance (meters)

# Controller parameters
Kp = 2.0
max_speed = 0.5
min_speed = 0.1

# Detect correct drive function
def make_drive_adapter(bot):
    """Automatically match whatever drive method your MBot has."""
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
    elif hasattr(bot, "motors"):
        def move(vx):
            bot.motors(vx, vx)
        return move
    else:
        def move(_vx):
            pass
        return move

move = make_drive_adapter(robot)

try:
    while True:
        # Read lidar scan
        scan = robot.read_lidar()
        # Handle different formats (tuple or dict)
        if isinstance(scan, dict):
            ranges = scan.get('range') or scan.get('ranges')
            thetas = scan.get('angle') or scan.get('angles')
        else:
            ranges, thetas = scan

        # Compute forward distance
        dist_to_wall = find_fwd_dist(ranges, thetas)

        if not np.isfinite(dist_to_wall):
            move(0.0)
            time.sleep(0.1)
            continue

        # --- FIXED SIGN ---
        # Positive error → robot too far → move forward
        # Negative error → robot too close → move backward
        error = dist_to_wall - setpoint
        control_signal = Kp * error
        speed = float(np.clip(control_signal, -max_speed, max_speed))

        # Apply minimum threshold
        if abs(speed) < min_speed and abs(error) > 0.1:
            speed = min_speed if speed > 0 else -min_speed

        # Drive the robot
        move(speed)

        print(f"Distance: {dist_to_wall:.2f}m, Error: {error:+.2f}m, Speed: {speed:+.2f}m/s")
        time.sleep(0.1)

except KeyboardInterrupt:
    move(0.0)
    print("\n[INFO] Stopped 1D hold.")
except Exception as e:
    print(f"[ERROR] {e}")
    move(0.0)
