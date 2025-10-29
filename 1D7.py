import time, inspect
from mbot_bridge.api import MBot

# =============== CONFIG YOU SET ===============
RANGE_PATH  = "front_range_m"   # e.g., "front_range_m" or "sensors.ultrasonic.front_cm"
RANGE_UNITS = "m"               # "m" or "cm"
# =============================================

STOP_DISTANCE = 0.25
FORWARD_SPEED = 0.25
REVERSE_SPEED = -0.20
TURN_RATE     = 1.2
LOOP_HZ       = 10
REVERSE_TIME  = 0.7
TURN_TIME     = 0.8

bot = MBot()

# ---- Drive adapter ----
def _detect_drive_fn():
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            print("[INFO] drive(vx,vy,wz)")
            return lambda vx=0, vy=0, wz=0: bot.drive(vx, vy, wz)
        elif n == 2:
            print("[INFO] drive(vx,wz)")
            return lambda vx=0, vy_unused=0, wz=0: bot.drive(vx, wz)
    if hasattr(bot, "motors"):
        print("[INFO] motors(v_l,v_r)")
        WHEEL_BASE = 0.16
        def drive(vx=0, vy_unused=0, wz=0):
            v_l = vx - (wz * WHEEL_BASE / 2.0)
            v_r = vx + (wz * WHEEL_BASE / 2.0)
            bot.motors(v_l, v_r)
        return drive
    raise RuntimeError("No drive method on MBot.")

DRIVE = _detect_drive_fn()
STOP  = (lambda : bot.stop()) if hasattr(bot, "stop") else (lambda : DRIVE(0,0,0))

# ---- Generic dict key-path reader ----
def _get_by_path(root, path):
    cur = root
    for p in path.split("."):
        if isinstance(cur, dict):
            cur = cur.get(p, None)
        else:
            cur = getattr(cur, p, None)
        if cur is None:
            return None
    return cur

def _maybe_call(x):
    try:
        return x() if callable(x) and x.__code__.co_argcount == 0 else x
    except Exception:
        return x

def read_distance_m():
    """
    Tries dict/state calls first; then attribute path. Converts units.
    """
    # First: if RANGE_PATH can be resolved through sensor/state methods:
    for dm in ("get_sensors", "read_sensors", "get_state", "read_state"):
        fn = getattr(bot, dm, None)
        if callable(fn):
            d = fn()
            if isinstance(d, dict):
                val = _get_by_path(d, RANGE_PATH)
                if val is not None:
                    try:
                        f = float(_maybe_call(val))
                        return f/100.0 if RANGE_UNITS == "cm" else f
                    except Exception:
                        pass
    # Second: try direct attribute path on bot
    val = _get_by_path(bot, RANGE_PATH)
    if val is not None:
        try:
            f = float(_maybe_call(val))
            return f/100.0 if RANGE_UNITS == "cm" else f
        except Exception:
            return None
    return None

def avoid():
    print("[AVOID] stop → reverse → turn")
    STOP()
    time.sleep(0.05)
    DRIVE(REVERSE_SPEED, 0.0, 0.0); time.sleep(REVERSE_TIME)
    STOP(); time.sleep(0.05)
    DRIVE(0.0, 0.0, TURN_RATE); time.sleep(TURN_TIME)
    STOP(); time.sleep(0.05)

def main():
    period = 1.0/LOOP_HZ
    print(f"[INFO] Using RANGE_PATH='{RANGE_PATH}' units={RANGE_UNITS}")
    while True:
        t0 = time.time()
        dist = read_distance_m()
        if dist is None:
            print("[WARN] no distance value; stopping")
            STOP()
        else:
            print(f"[RANGE] {dist:.3f} m")
            if dist < STOP_DISTANCE:
                avoid()
            else:
                DRIVE(FORWARD_SPEED, 0.0, 0.0)
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

try:
    main()
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    STOP()
    print("Robot halted.")

