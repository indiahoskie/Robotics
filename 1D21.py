import time
import numpy as np
import math
import inspect
from mbot_bridge.api import MBot

# ----------------- TUNABLES -----------------
SETPOINT   = 0.25   # desired distance from wall (m)
TOLERANCE  = 0.05   # acceptable margin (m)
FWD_SPEED  = 0.25   # forward speed (m/s)
REV_SPEED  = -0.20  # backward speed (m/s)
LOOP_HZ    = 10     # control rate
WINDOW     = 5      # beams near 0° considered "front"
# --------------------------------------------

# --- Helper: extract (ranges, thetas) from LiDAR ---
def get_ranges_thetas(robot):
    scan = robot.read_lidar()
    if isinstance(scan, dict):
        ranges = scan.get('range') or scan.get('ranges') or []
        thetas = scan.get('angle') or scan.get('angles') or []
    else:
        ranges, thetas = scan
    return ranges, thetas

# --- Find forward distance from LiDAR (averaging beams near 0°) ---
def find_fwd_dist(ranges, thetas, window=WINDOW):
    if not ranges or not thetas:
        return float('inf')
    fwd_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    fwd_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)
    valid_idx = (fwd_ranges > 0.0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists))

# --- Simple motion adapter (for different drive APIs) ---
def make_drive_adapter(bot):
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            def move(vx): bot.drive(float(vx), 0.0, 0.0)
            return move
        elif n == 2:
            def move(vx): bot.drive(float(vx), 0.0)
            return move
    elif hasattr(bot, "motors"):
        def move(vx): bot.motors(float(vx), float(vx))
        return move
    def move(_): pass
    return move

# Initialize robot
robot = MBot()
move = make_drive_adapter(robot)

# --- Bang-Bang Wall Follower ---
def follow_wall_loop():
    print(f"[INFO] Starting Bang-Bang Wall Follower | Setpoint={SETPOINT:.2f} m ± {TOLERANCE:.2f} m")
    dt = 1.0 / LOOP_HZ

    while True:
        ranges, thetas = get_ranges_thetas(robot)
        dist = find_fwd_dist(ranges, thetas)

        # If no valid LiDAR reading, stop and try again
        if not np.isfinite(dist) or math.isinf(dist):
            move(0.0)
            time.sleep(dt)
            continue

        # Bang-Bang control
        if dist > SETPOINT + TOLERANCE:
            # Too far → move forward
            move(FWD_SPEED)
            state = "FORWARD"
        elif dist < SETPOINT - TOLERANCE:
            # Too close → move backward
            move(REV_SPEED)
            state = "BACKWARD"
        else:
            # Within acceptable range → stop
            move(0.0)
            state = "HOLD"

        print(f"dist={dist:.3f} m | state={state} | target={SETPOINT:.2f}±{TOLERANCE:.2f}")
        time.sleep(dt)

try:
    follow_wall_loop()
except KeyboardInterrupt:
    move(0.0)
    print("\n[INFO] Stopped Bang-Bang controller.")
