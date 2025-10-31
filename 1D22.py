import time
import numpy as np
import inspect
from mbot_bridge.api import MBot

# ------------------------ CONFIG ------------------------
SETPOINT = 1.0     # meters: target distance to obstacle
KP = 2.0
MAX_SPEED = 0.5
MIN_SPEED = 0.1
LOOP_HZ = 10
NO_DATA_SLEEP = 0.1   # seconds to wait when no lidar data yet
# --------------------------------------------------------

# ---------- Helpers ----------
def find_fwd_dist(ranges, thetas, window=5):
    """Mean forward distance using first/last 'window' rays."""
    if not ranges or not thetas:
        return float('inf')
    fwd_ranges = np.array(ranges[:window] + ranges[-window:], dtype=float)
    fwd_thetas = np.array(thetas[:window] + thetas[-window:], dtype=float)
    valid_idx = (fwd_ranges > 0).nonzero()
    if len(valid_idx[0]) == 0:
        return float('inf')
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return float(np.mean(fwd_dists))

def make_drive_adapter(bot):
    """Match whatever drive/motors signature your MBot has."""
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
        def move(vx):
            bot.motors(vx, vx)
        return move
    # Safe no-op if nothing found
    def move(_vx): 
        pass
    return move

def make_lidar_reader(bot):
    """
    Return a function that yields (ranges, thetas) or None if no data.
    Tries multiple common MBot APIs and channel styles.
    """
    candidates = [
        "
