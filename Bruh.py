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
SETPOINT = 0.25  # Desired distance from object (meters)
TOLERANCE = 0.05  # Acceptable range around setpoint (meters)

def get_front_range_m(ranges, thetas):
    """
    Get front range reading from lidar sensor.
    Returns distance in meters or None if reading unavailable.
    """
    if not ranges or not thetas or len(ranges) == 0:
        return None

    # Use a small window around 0 degrees (front of robot)
    window = 5
    front_ranges = ranges[:window] + ranges[-window:]
    front_thetas = thetas[:window] + thetas[-window:]

    valid_distances = []
    for i, dist in enumerate(front_ranges):
        if dist > 0:
            fwd_dist = dist * np.cos(front_thetas[i])
            valid_distances.append(fwd_dist)

    if not valid_distances:
        return None

    return np.mean(valid_distances)

def follow_wall_loop(robot):
    """
    Main control loop to maintain setpoint distance using lidar.
    """
    while True:
        ranges, thetas = robot.read_lidar()
        dist = get_front_range_m(ranges, thetas)

        if dist is None:
            robot.stop()
            time.sleep(0.2)
            continue

        print(f"Front distance: {dist:.2f} m")  # Debug output

        if dist < SETPOINT - TOLERANCE:
            robot.drive(-0.2, 0.0, 0.0)  # Backward
        elif dist > SETPOINT + TOLERANCE:
            robot.drive(0.2, 0.0, 0.0)  # Forward
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


