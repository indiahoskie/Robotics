#!/usr/bin/env python3
"""
Improved Follow Me 1D Script
Maintains a setpoint distance from an object using lidar.
"""

import time
import numpy as np
import sys
from mbot_bridge.api import MBot

# Constants
SETPOINT = 0.25      # Desired distance from object (meters)
TOLERANCE = 0.05     # Acceptable range around setpoint (meters)
FORWARD_SPEED = 0.3  # Forward speed (m/s)
BACKWARD_SPEED = -0.3  # Backward speed (m/s)

def get_front_range_m(ranges, thetas):
    """
    Get average front-facing lidar distance using ±15° window.
    """
    if not ranges or not thetas or len(ranges) != len(thetas):
        return None

    # Convert to numpy arrays
    ranges = np.array(ranges)
    thetas = np.array(thetas)

    # Filter for angles within ±15° (≈0.26 rad)
    angle_window = 0.26
    mask = np.abs(thetas) < angle_window
    front_ranges = ranges[mask]
    front_thetas = thetas[mask]

    # Filter out invalid readings
    valid = front_ranges > 0.05  # ignore zero or very small readings
    front_ranges = front_ranges[valid]
    front_thetas = front_thetas[valid]

    if len(front_ranges) == 0:
        return None

    # Project distances forward
    forward_distances = front_ranges * np.cos(front_thetas)
    return np.mean(forward_distances)

def follow_wall_loop(robot):
    """
    Main control loop to maintain setpoint distance using lidar.
    """
    while True:
        ranges, thetas = robot.read_lidar()
        dist = get_front_range_m(ranges, thetas)

        if dist is None:
            print("No valid lidar data. Stopping.")
            robot.stop()
            time.sleep(0.2)
            continue

        print(f"Front distance: {dist:.2f} m")

        if dist < SETPOINT - TOLERANCE:
            robot.drive(BACKWARD_SPEED, 0.0, 0.0)
        elif dist > SETPOINT + TOLERANCE:
            robot.drive(FORWARD_SPEED, 0.0, 0.0)
        else:
            robot.stop()

        time.sleep(0.1)

def main():
    """Initialize and run the controller."""
    robot = None
    try:
        print("Connecting to mbot...")
        robot = MBot()
        time.sleep(1)
        print("Starting follow 1D controller...")
        follow_wall_loop(robot)
    except KeyboardInterrupt:
        print("\nStopping robot...")
        if robot:
            robot.stop()
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        if robot:
            robot.stop()
        sys.exit(1)

if __name__ == "__main__":
    main()


