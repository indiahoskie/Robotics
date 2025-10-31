Otherwise, keep cruising forward.

Here’s a drop-in version that does exactly that:

import time
import numpy as np
import inspect
from mbot_bridge.api import MBot

# ---------- TUNABLES ----------
setpoint = 0.35       # meters; do NOT go closer than this (your "hit" threshold)
Kp = 2.0              # proportional gain for backing away when too close
max_speed = 0.5       # absolute cap on linear speed (m/s)
min_speed = 0.1       # minimum commanded speed when correcting
CRUISE_SPEED = 0.25   # forward speed when path is clear (m/s)
DEADBAND = 0.03       # meters; tiny buffer around setpoint to avoid chatter
LOOP_DT = 0.1         # seconds; control loop period
# ------------------------------

def find_fwd_dist(ranges, thetas, window=5):
    """Estimate forward distance using rays near the front of the scan."""
    if not ranges or not thetas:
        return float('inf')

    fwd_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    fwd_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)

    valid_idx = (fwd_ranges > 0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')

    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)  # project onto forward axis
    return float(np.mean(fwd_dists))

# --- Motion adapter (auto-detect drive signature) ---
robot = MBot()

def make_drive_adapter(bot):
    """Return move(vx) that calls the correct method under the hood."""
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:  # drive(vx, vy, wz)
            def move(vx):
                bot.drive(vx, 0.0, 0.0)
            return move
        elif n == 2:  # drive(vx, wz)
            def move(vx):
                bot.drive(vx, 0.0)
            return move
    if hasattr(bot, "motors"):
        # NOTE: If motors() expects wheel velocities, vx here should be scaled.
        def move(vx):
            bot.motors(vx, vx)
        return move
    # Fallback no-op
    def move(_vx):
        pass
    return move

move = make_drive_adapter(robot)

try:
    while True:
        # --- Read lidar (support tuple or dict) ---
        scan = robot.read_lidar()
        if isinstance(scan, dict):
            ranges = scan.get('range') or scan.get('ranges')
            thetas = scan.get('angle') or scan.get('angles')
        else:
            ranges, thetas = scan

        dist_to_wall = find_fwd_dist(ranges, thetas)

        # --- Behavior: keep moving forward until an object is "hit"/too close ---
        if not np.isfinite(dist_to_wall):
            # No obstacle seen -> cruise forward
            speed = CRUISE_SPEED
            move(speed)
            print(f"Distance: inf, Cruising at {speed:+.2f} m/s")
            time.sleep(LOOP_DT)
            continue

        # If far away (> setpoint + deadband): cruise forward
        if dist_to_wall > (setpoint + DEADBAND):
            speed = CRUISE_SPEED
        else:
            # Too close or at boundary: back off/hold using P-control
            # error = dist - setpoint -> negative when too close
            error = dist_to_wall - setpoint
            control_signal = Kp * error
            # When too close (error <= 0), control_signal <= 0 (backward/hold)
            speed = float(np.clip(control_signal, -max_speed, max_speed))

            # Apply a minimum effort when we actually need to correct
            if abs(speed) < min_speed and error < -DEADBAND:
                speed = -min_speed

            # Small oscillations around boundary -> stop
            if abs(error) <= DEADBAND:
                speed = 0.0

        move(speed)
        print(f"Distance: {dist_to_wall:.2f} m | Setpoint: {setpoint:.2f} m | Cmd: {speed:+.2f} m/s")
        time.sleep(LOOP_DT)

except KeyboardInterrupt:
    move(0.0)
    print("\n[INFO] Stopped 1D forward-until-hit behavior.")
except Exception as e:
    print(f"[ERROR] {e}")
    move(0.0)
