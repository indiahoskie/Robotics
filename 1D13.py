#!/usr/bin/env python3
import time, math
from mbot_bridge.api import MBot

V = 0.30           # forward speed
KY = 1.5           # yaw P gain (start 0.8–2.0)
WZ_MAX = 0.4       # clamp yaw correction
DRIVE_TIME_S = 3.0

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x
def ang_wrap(a): 
    # wrap angle to [-pi, pi]
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a

robot = MBot()

def read_yaw():
    # Try common IMU interfaces: (roll, pitch, yaw) or dict/object
    for name in ["read_imu", "readImu", "get_imu", "getImu", "imu"]:
        if hasattr(robot, name):
            obj = getattr(robot, name)
            data = obj() if callable(obj) else obj
            # normalize possible formats
            if isinstance(data, tuple) and len(data) == 3:
                return float(data[2])
            if isinstance(data, dict) and "yaw" in data:
                return float(data["yaw"])
            if hasattr(data, "yaw"):
                return float(data.yaw)
    return None

def drive(vx, wz):
    # Send velocity through whichever API exists
    if hasattr(robot, "drive"):
        try: robot.drive(vx, 0.0, wz)    # vx, vy, wz
        except TypeError: robot.drive(vx, wz)  # vx, wz
    elif hasattr(robot, "setVelocity"):
        robot.setVelocity(vx, 0.0, wz)
    elif hasattr(robot, "set_cmd_vel"):
        robot.set_cmd_vel(vx, wz)

def stop():
    if hasattr(robot, "stop"): robot.stop()
    else:
        try: drive(0.0, 0.0)
        except: pass

try:
    yaw0 = read_yaw()
    if yaw0 is None:
        print("No IMU yaw; falling back to simple drive.")
        yaw_hold = False
    else:
        yaw_hold = True

    t0 = time.time()
    while time.time() - t0 < DRIVE_TIME_S:
        if yaw_hold:
            yaw = read_yaw()
            if yaw is None: wz = 0.0
            else:
                err = ang_wrap(yaw - yaw0)  # deviation from initial heading
                wz = clamp(-KY * err, -WZ_MAX, WZ_MAX)
        else:
            wz = 0.0

        drive(V, wz)
        time.sleep(0.02)

except KeyboardInterrupt:
    pass
finally:
    stop()
