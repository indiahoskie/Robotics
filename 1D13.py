#!/usr/bin/env python3
import time
from mbot_bridge.api import MBot

# Create MBot object
robot = MBot()

try:
    # Drive forward at 0.3 m/s for 3 seconds
    print("Moving forward...")
    robot.drive(0.3, 0.0, 0.0)   # (vx, vy, wz) — forward, no sideways, no rotation
    time.sleep(3)

    # Stop the robot
    print("Stopping...")
    robot.drive(0.0, 0.0, 0.0)

except KeyboardInterrupt:
    # If you press Ctrl+C, stop safely
    print("Interrupted. Stopping robot.")
    robot.drive(0.0, 0.0, 0.0)

