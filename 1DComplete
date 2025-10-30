import time
import inspect
from mbot_bridge.api import MBot

bot = MBot()

# ------------------------ PARAMETERS ------------------------
SETPOINT = 0.5      # desired distance to wall (meters)
MARGIN = 0.05       # acceptable error range
VEL = 0.2           # forward/backward velocity
LOOP_DT = 0.1       # loop delay (seconds)
# ------------------------------------------------------------

# Choose drive method based on available API
def _detect_drive_fn():
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            print("[INFO] Using drive(vx, vy, wz)")
            return lambda vx=0, vy=0, wz=0: bot.drive(vx, vy, wz)
        elif n == 2:
            print("[INFO] Using drive(vx, wz)")
            return lambda vx=0, wz=0: bot.drive(vx, wz)
    elif hasattr(bot, "motors"):
        print("[INFO] Using motors(v_l, v_r)")
        return lambda vx=0, wz=0: bot.motors(vx, wz)
    raise AttributeError("No suitable drive function found.")

drive = _detect_drive_fn()

# ------------------------ CONTROL LOOP ------------------------
try:
    while True:
        # Get LIDAR scan and distance
        scan = bot.get_lidar_scan()
        dist_to_wall = bot.findMinDist(scan)

        if dist_to_wall < 0:
            continue  # skip invalid readings

        # Bang-Bang Control
        if dist_to_wall > SETPOINT + MARGIN:
            drive(VEL, 0, 0)  # move forward
        elif dist_to_wall < SETPOINT - MARGIN:
            drive(-VEL, 0, 0) # move backward
        else:
            drive(0, 0, 0)    # stop if within margin

        time.sleep(LOOP_DT)

except KeyboardInterrupt:
    drive(0, 0, 0)
    print("Stopped safely.")
