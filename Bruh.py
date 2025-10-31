#!/usr/bin/env python3
"""
Follow Me 1D Script
This script makes the robot maintain a setpoint distance from a wall/object in front using lidar.
"""

import time
import numpy as np
import sys
from mbot_bridge.api import MBot

# Constants
SETPOINT = 0.25       # meters - desired distance from wall
TOLERANCE = 0.05      # meters - acceptable range around setpoint
FORWARD_SPEED = 0.2   # m/s
BACKWARD_SPEED = -0.2 # m/s

def get_front_range_m(ranges, thetas, window=10):
    """
    Get front range reading from lidar sensor.
    Returns distance in meters or None if reading unavailable.
    """
    if not ranges or not thetas or len(ranges) != len(thetas):
        return None

    # Use rays near 0 degrees (front of robot)
    front_ranges = np.array(ranges[:window] + ranges[-window:])
    front_thetas = np.array(thetas[:window] + thetas[-window:])

    # Filter out invalid readings
    valid = front_ranges > 0.05
    if not np.any(valid):
        return None

    front_ranges = front_ranges[valid]
    front_thetas = front_thetas[valid]

    # Project distances forward
    forward_distances = front_ranges * np.cos(front_thetas)
    return np.mean(forward_distances)

def follow_wall_loop(robot):
    """
    Follow controller using lidar sensor: maintains setpoint distance from wall/object.
    """
    while True:
        ranges, thetas = robot.read_lidar()
        dist = get_front_range_m(ranges, thetas)

        if dist is None:
            print("No valid lidar data. Stopping.")
            robot.stop()
            time.sleep(0.2)
            continue

        print(f"Distance to wall: {dist:.2f} m")

        if dist < SETPOINT - TOLERANCE:
            robot.drive(BACKWARD_SPEED, 0.0, 0.0)
        elif dist > SETPOINT + TOLERANCE:
            robot.drive(FORWARD_SPEED, 0.0, 0.0)
        else:
            robot.stop()

        time.sleep(0.1)

def main():
    """Main function to run the follow 1D controller."""
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
