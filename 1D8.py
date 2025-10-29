#!/usr/bin/env python3
import math
import time
from mbot_bridge.api import MBot

# ---------------------- PARAMETERS ----------------------
forward_speed = 0.20          # m/s forward
forward_time  = 2.00          # seconds per side (target duration if path is clear)
turn_speed    = math.pi / 2   # rad/s (≈90°/s)
turn_time     = (math.pi / 2) / turn_speed  # time for a 90° turn (seconds)

# Safety / sensing
STOP_DISTANCE = 0.25          # meters: stop if object closer than this
CHECK_HZ      = 10            # range polling rate during forward motion
DO_AVOID      = True          # if True: quick back-up + nudge turn when blocked
REVERSE_SPEED = -0.18         # m/s for avoidance
REVERSE_TIME  = 0.5           # seconds to reverse when avoiding
NUDGE_TURN    = 0.5           # seconds of small turn to "slide" off the obstacle
# -------------------------------------------------------

# Initialize robot
robot = MBot()

def get_front_range_m(robot):
    """
    Returns front distance in meters or None if unavailable.
    Replace the internals with your known-good API if you have it.
    """
    try:
        if hasattr(robot, "get_range"):
            return robot.get_range("front")
        if hasattr(robot, "get_front_distance"):
            return robot.get_front_distance()
        if hasattr(robot, "range_front"):
            return robot.range_front()
        if hasattr(robot, "read_ultrasonic"):
            d = robot.read_ultrasonic("front")  # may be cm
            return d / 100.0 if d is not None else None
        if hasattr(robot, "get_sensors"):
            s = robot.get_sensors()
            for k in ("front_range_m", "front_distance_m", "range_front_m", "range"):
                if k in s:
                    return s[k]
    except Exception:
        pass
    return None

def safe_stop(bot):
    try:
        bot.stop()
    except Exception:
        pass

try:
    # Drive 1 square (change this if you want more)
    for square in range(1):
        print(f"Driving square {square + 1}/1")

        # 4 sides per square
        for side in range(4):
            print(f"  Side {side + 1}/4 → forward up to {forward_time:.2f}s")

            # Forward with obstacle check for up to forward_time
            side_start = time.time()
            period = 1.0 / CHECK_HZ

            while True:
                now = time.time()
                elapsed = now - side_start
                if elapsed >= forward_time:
                    break  # finished this side's forward run

                # Read distance
                dist = get_front_range_m(robot)
                if dist is not None:
                    print(f"    [dist] {dist:.2f} m", end="\r")

                if dist is not None and dist < STOP_DISTANCE:
                    print(f"\n    [STOP] Obstacle at {dist:.2f} m — stopping.")
                    safe_stop(robot)

                    if DO_AVOID:
                        # Quick avoidance: back up, slight turn, then continue timebox
                        print("    [AVOID] Reversing and nudging right.")
                        robot.drive(REVERSE_SPEED, 0.0, 0.0)
                        time.sleep(REVERSE_TIME)
                        safe_stop(robot)

                        robot.drive(0.0, 0.0, +turn_speed)  # nudge right
                        time.sleep(NUDGE_TURN)
                        safe_stop(robot)
                    else:
                        # If not avoiding, wait until clear before continuing
                        while True:
                            time.sleep(period)
                            dist2 = get_front_range_m(robot)
                            if dist2 is not None and dist2 >= STOP_DISTANCE:
                                print("    [CLEAR] Path clear; resuming.")
                                break
                        # continue loop to resume forward

                # Path is clear (or no reading); proceed forward a small step
                robot.drive(forward_speed, 0.0, 0.0)

                # Hold this small step, but keep loop responsive
                step_start = time.time()
                while time.time() - step_start < period:
                    time.sleep(0.001)

            # Stop at end of side
            safe_stop(robot)
            time.sleep(0.1)

            # Turn 90° (except after last side)
            if side < 3:
                print(f"  Turning 90° (≈{turn_time:.2f}s)")
                robot.drive(0.0, 0.0, turn_speed)
                time.sleep(turn_time)
                safe_stop(robot)
                time.sleep(0.1)

    print("Done!")

except KeyboardInterrupt:
    print("\nStopped by user.")
    safe_stop(robot)
except Exception as e:
    print(f"\n[ERROR] {e}")
    safe_stop(robot)
finally:
    safe_stop(robot)
