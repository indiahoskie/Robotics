#!/usr/bin/env python3
import math
import time
from mbot_bridge.api import MBot

# Initialize robot
robot = MBot()

# Square parameters
forward_speed = 0.2        # m/s
forward_time = 2.0         # seconds per side
turn_speed = math.pi / 2   # rad/s (30 deg/sec)
turn_time = math.pi / 2 / turn_speed  # time for 90 degree turn

try:
    # Drive 1 square
    for square in range(1):
        print(f"Driving square {square + 1}/1")
        
        # 4 sides per square
        for side in range(4):
            # Move forward
            robot.drive(forward_speed, 0, 0)
            time.sleep(forward_time)
            robot.stop()
            time.sleep(0.1)
            
            # Turn (except after last side)
            if side < 3:
                robot.drive(0, 0, turn_speed)
                time.sleep(turn_time)
                robot.stop()
                time.sleep(0.1)
    
    print("Done!")

except:
    robot.stop()
