import time
import numpy as np
from mbot_bridge.api import MBot

# Get distance to wall
def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot."""
    fwd_ranges = np.array(ranges[:window] + ranges[-window:])
    fwd_thetas = np.array(thetas[:window] + thetas[-window:])
    valid_idx = (fwd_ranges > 0).nonzero()
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return np.mean(fwd_dists) if len(fwd_dists) > 0 else None

# Initialize robot
robot = MBot()

# Bang-Bang Controller Parameters
SETPOINT = 1.0         # Desired distance to wall (meters)
TOLERANCE = 0.1        # Acceptable margin (meters)
FORWARD_SPEED = 0.3    # Forward velocity (m/s)
BACKWARD_SPEED = -0.3  # Backward velocity (m/s)

try:
    while True:
        ranges, thetas = robot.read_lidar()
        dist_to_wall = find_fwd_dist(ranges, thetas)

        if dist_to_wall is None:
            print("No valid lidar data. Stopping.")
            robot.stop()
            time.sleep(0.2)
            continue

        print(f"Distance to wall: {dist_to_wall:.2f} m")

        # Bang-Bang Control Logic
        if dist_to_wall < SETPOINT - TOLERANCE:
            robot.drive(BACKWARD_SPEED, 0.0, 0.0)
        elif dist_to_wall > SETPOINT + TOLERANCE:
            robot.drive(FORWARD_SPEED, 0.0, 0.0)
        else:
            robot.stop()

        time.sleep(0.1)

except Exception as e:
    print(f"Stopping due to error: {e}")
    robot.stop()
