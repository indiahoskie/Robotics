import time
import inspect
from mbot_bridge.api import MBot

# ------------------------ TUNABLES ------------------------
SETPOINT = 0.45      # meters (desired distance to nearest wall)
MARGIN   = 0.03      # meters (deadband around setpoint)
V_FWD    = 0.22      # m/s when too far (bang-bang forward)
V_BACK   = -0.18     # m/s when too close (bang-bang backward)
KP       = 0.8       # proportional gain (P-control)
V_MAX    = 0.30      # speed clamp for P-control
LOOP_HZ  = 15        # control loop frequency
DT       = 1.0/LOOP_HZ
# ---------------------------------------------------------

bot = MBot()

# --- Motion adapter: always provide drive1d(vx) that works on your robot ---
def _make_drive1d(bot):
    if hasattr(bot, "drive"):
        # drive can be (vx, vy, wz) OR (vx, wz)
        n = len(inspect.signature(bot.drive).parameters)
        if n >= 3:
            def drive1d(vx):
                bot.drive(vx, 0.0, 0.0)
        else:
            def drive1d(vx):
                bot.drive(vx, 0.0)
    elif hasattr(bot, "motors"):
        # crude differential mapping for straight motion
        def drive1d(vx):
            bot.motors(vx, vx)
    else:
        raise RuntimeError("No supported drive method on MBot")
    return drive1d

drive1d = _make_drive1d(bot)

# ---------------------------------------------------------
# Helper: clamp a value
def clamp(x, lo, hi): 
    return max(lo, min(hi, x))

# ---------------------------------------------------------
# Choose ONE of these control loops to run.

def run_bang_bang():
    """
    Slides: 
      If the robot is too far from the wall -> drive forward
      If the robot is too close to the wall -> drive backward
      If within an allowable margin -> stop
      Repeat forever
    Need to pick: velocity, margin
    """
    ranges, thetas = [], []
    print("[Bang-Bang] setpoint=%.2fm, margin=%.2fm" % (SETPOINT, MARGIN))
    try:
        while True:
            # Read a scan
            getlidarscan(ranges, thetas)

            # Distance to nearest wall (provided function)
            dist_to_wall = findMinDist(ranges, thetas)
            if dist_to_wall < 0:        # skip invalid scans
                time.sleep(DT)
                continue

            # Error: positive if we are too far, negative if too close
            error = dist_to_wall - SETPOINT

            if abs(error) <= MARGIN:
                vx = 0.0
            elif error > 0:
                vx = V_FWD      # too far -> go forward
            else:
                vx = V_BACK     # too close -> go backward

            drive1d(vx)
            time.sleep(DT)
    except KeyboardInterrupt:
        drive1d(0.0)
        print("\n[Bang-Bang] Stopped.")

def run_p_control():
    """
    Slides:
      velocity = kp * error
      If within allowable margin -> stop
    Need to pick: kp, margin
    """
    ranges, thetas = [], []
    print("[P-Control] setpoint=%.2fm, kp=%.2f, margin=%.2fm" % (SETPOINT, KP, MARGIN))
    try:
        while True:
            # Read a scan
            getlidarscan(ranges, thetas)

            # Distance to nearest wall
            dist_to_wall = findMinDist(ranges, thetas)
            if dist_to_wall < 0:
                time.sleep(DT)
                continue

            error = dist_to_wall - SETPOINT          # + = too far, - = too close
            if abs(error) <= MARGIN:
                vx = 0.0
            else:
                vx = clamp(KP * error, -V_MAX, V_MAX)

            drive1d(vx)
            time.sleep(DT)
    except KeyboardInterrupt:
        drive1d(0.0)
        print("\n[P-Control] Stopped.")

# ---------------------------------------------------------
# >>> Pick which controller to run <<<
# run_bang_bang()
# run_p_control()
