#!/usr/bin/env python3
import time
import math
from mbot_bridge.api import MBot

# how long to drive
DRIVE_TIME_S = 3.0
# base forward speed (m/s or wheel units depending on your API)
V = 0.30
# small bias to cancel drift: >0 speeds up LEFT wheel, <0 speeds up RIGHT
TRIM = 0.03     # start small (0.01–0.05) and adjust sign to cancel the turn

robot = MBot()

def stop():
    # best-effort stop across common APIs
    if hasattr(robot, "stop"):
        robot.stop()
    elif hasattr(robot, "drive"):
        try: robot.drive(0.0, 0.0, 0.0)
        except: 
            try: robot.drive(0.0, 0.0)
            except: pass
    elif hasattr(robot, "setVelocity"):
        try: robot.setVelocity(0.0, 0.0, 0.0)
        except: pass
    elif hasattr(robot, "set_cmd_vel"):
        try: robot.set_cmd_vel(0.0, 0.0)
        except: pass

try:
    print("Driving forward with trim...")
    if hasattr(robot, "motors"):
        # Differential drive: set individual wheel speeds
        # Assume units are m/s at wheel or normalized; tune V accordingly.
        v_l = V * (1.0 + TRIM)
        v_r = V * (1.0 - TRIM)
        t0 = time.time()
        while time.time() - t0 < DRIVE_TIME_S:
            robot.motors(v_l, v_r)
            time.sleep(0.02)
        robot.motors(0.0, 0.0)

    elif hasattr(robot, "drive"):
        # API might be drive(vx, vy, wz) or drive(vx, wz)
        # We keep wz=0 (no commanded rotation). If it still drifts, switch to Option B below.
        try:
            print("Using drive(vx, vy, wz)")
            t0 = time.time()
            while time.time() - t0 < DRIVE_TIME_S:
                robot.drive(V, 0.0, 0.0)
                time.sleep(0.02)
            robot.drive(0.0, 0.0, 0.0)
        except TypeError:
            print("Using drive(vx, wz)")
            t0 = time.time()
            while time.time() - t0 < DRIVE_TIME_S:
                robot.drive(V, 0.0)
                time.sleep(0.02)
            robot.drive(0.0, 0.0)

    elif hasattr(robot, "setVelocity"):
        print("Using setVelocity(vx, vy, wz)")
        t0 = time.time()
        while time.time() - t0 < DRIVE_TIME_S:
            robot.setVelocity(V, 0.0, 0.0)
            time.sleep(0.02)
        robot.setVelocity(0.0, 0.0, 0.0)

    elif hasattr(robot, "set_cmd_vel"):
        print("Using set_cmd_vel(vx, wz)")
        t0 = time.time()
        while time.time() - t0 < DRIVE_TIME_S:
            robot.set_cmd_vel(V, 0.0)
            time.sleep(0.02)
        robot.set_cmd_vel(0.0, 0.0)

    else:
        print("No known drive API found on MBot object.")

except KeyboardInterrupt:
    pass
finally:
    stop()
    print("Stopped.")
