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
SETPOINT = 0.25  # meters - desired distance from wall
TOLERANCE = 0.05  # meters - acceptable range around setpoint

def get_front_range_m(ranges, thetas):
    """
    Get front range reading from lidar sensor.
    Returns distance in meters or None if reading unavailable.
    """
    if not ranges or not thetas or len(ranges) == 0:
        return None
    
    # Find forward distance (angles near 0)
    # Front of robot is typically at theta=0, so look at rays near 0
    window = 5
    front_ranges = ranges[:window] + ranges[-window:]
    front_thetas = thetas[:window] + thetas[-window:]
    
    # Filter out zeros (bad readings) and calculate forward distance
    valid_distances = []
    for i, dist in enumerate(front_ranges):
        if dist > 0:
            # Calculate forward component: distance * cos(angle)
            fwd_dist = dist * np.cos(front_thetas[i])
            valid_distances.append(fwd_dist)
    
    if len(valid_distances) == 0:
        return None
    
    # Return average of forward distances
    return np.mean(valid_distances)

def follow_wall_loop(robot):
    """
    Follow controller using lidar sensor: maintains setpoint distance from wall/object.
    If object moves closer, bot backs up. If object moves away, bot moves forward.
    """
    while True:
        ranges, thetas = robot.read_lidar()
        dist = get_front_range_m(ranges, thetas)
        
        if dist is None:
            robot.stop()
            time.sleep(0.2)
            continue

        # If too close to wall/object (or object moved closer), back up to maintain setpoint
        if dist < SETPOINT - TOLERANCE:
            robot.drive(-0.2, 0.0, 0.0)  # back up
            time.sleep(0.1)
        # If at setpoint (within tolerance), maintain position
        elif abs(dist - SETPOINT) <= TOLERANCE:
            robot.stop()
            time.sleep(0.1)
        # If too far from wall/object (or object moved away), drive forward to reach setpoint
        else:
            robot.drive(0.2, 0.0, 0.0)  # forward
            time.sleep(0.1)

def main():
    """Main function to run the follow 1D controller."""
    robot = None
    try:
        # Initialize the robot
        print("Connecting to mbot...")
        robot = MBot()
        
        # Wait a moment for connection to establish
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
