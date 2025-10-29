import time
import inspect
from mbot_bridge.api import MBot

# ------------------------ TUNABLES ------------------------
STOP_DISTANCE = 0.25     # meters; stop if obstacle closer than this
FORWARD_SPEED = 0.25     # m/s forward (linear x)
REVERSE_SPEED = -0.20    # m/s reverse
TURN_RATE = 1.2          # rad/s yaw (positive = left in most APIs)
LOOP_HZ = 10             # control loop frequency
REVERSE_TIME = 0.7       # seconds to back up
TURN_TIME = 0.8          # seconds to turn
# ---------------------------------------------------------

bot = MBot()

# --- Motion adapter (supports drive(vx, vy, wz), drive(vx, wz), or motors(v_l, v_r)) ---
def _detect_drive_fn():
    if hasattr(bot, "drive"):
        sig = inspect.signature(bot.drive)
        n = len(sig.parameters)
        if n >= 3:
            print("[INFO] Using drive(vx, vy, wz)")
            def drive(vx=0, vy=0, wz=0):
                try: bot.drive(vx, vy, wz)
                except Exception as e: print(f"[ERR] drive(3): {e}")
            return drive
        elif n == 2:
            print("[INFO] Using drive(vx, wz)")
            def drive(vx=0, vy_unused=0, wz=0):
                try: bot.drive(vx, wz)
                except Exception as e: print(f"[ERR] drive(2): {e}")
            return drive
    if hasattr(bot, "motors"):
        print("[INFO] Using motors(v_l, v_r) fallback")
        WHEEL_BASE = 0.16  # meters (adjust if needed)
        def drive(vx=0, vy_unused=0, wz=0):
            # Simple diff-drive conversion
            v_l = vx - (wz * WHEEL_BASE / 2.0)
            v_r = vx + (wz * WHEEL_BASE / 2.0)
            try: bot.motors(v_l, v_r)
            except Exception as e: print(f"[ERR] motors: {e}")
        return drive
    raise RuntimeError("No supported motion method found on MBot (drive or motors).")

def _detect_stop_fn():
    if hasattr(bot, "stop"):
        def stop(): 
            try: bot.stop()
            except Exception as e: print(f"[ERR] stop(): {e}")
        return stop
    # Fallback: send zero motion via drive adapter
    def stop():
        try: DRIVE(0.0, 0.0, 0.0)
        except Exception as e: print(f"[ERR] stop fallback: {e}")
    return stop

DRIVE = _detect_drive_fn()
STOP = _detect_stop_fn()

# --- Range adapter (tries common methods/keys, normalizes to meters) ---
def _detect_range_fn():
    # Method candidates in priority order: (callable, needs_arg, returns_cm)
    candidates = [
        ("get_range", True, False),
        ("get_front_distance", False, False),
        ("range_front", False, False),
        ("read_ultrasonic", True, True),   # often returns cm
    ]
    for name, needs_arg, is_cm in candidates:
        if hasattr(bot, name) and callable(getattr(bot, name)):
            fn = getattr(bot, name)
            try:
                if needs_arg:
                    val = fn("front")
                else:
                    val = fn()
                if val is not None:
                    meters = (val / 100.0) if is_cm else float(val)
                    print(f"[INFO] Using {name}{'(cm)' if is_cm else ''}; first read = {meters:.3f} m")
                    def reader():
                        try:
                            v = fn("front") if needs_arg else fn()
                            return None if v is None else ((v/100.0) if is_cm else float(v))
                        except Exception:
                            return None
                    return reader
            except Exception:
                pass

    # Try a sensor-dict method
    dict_methods = ["get_sensors", "read_sensors", "get_state", "read_state"]
    dict_keys = ["front_range_m", "front_distance_m", "range_front_m", "front_range", "range"]

    for dm in dict_methods:
        if hasattr(bot, dm) and callable(getattr(bot, dm)):
            fn = getattr(bot, dm)
            try:
                d = fn()
                if isinstance(d, dict):
                    for k in dict_keys:
                        if k in d:
                            try:
                                meters = float(d[k])
                                print(f"[INFO] Using {dm}()['{k}'] ; first read = {meters:.3f} m")
                                def reader():
                                    try:
                                        dd = fn()
                                        if not isinstance(dd, dict): return None
                                        v = dd.get(k, None)
                                        return None if v is None else float(v)
                                    except Exception:
                                        return None
                                return reader
                            except Exception:
                                continue
            except Exception:
                pass

    raise RuntimeError("No supported front distance method found on MBot.")

RANGE = _detect_range_fn()

# --- Control loop ---
def avoid_obstacle():
    print("[AVOID] Obstacle detected. Stopping.")
    STOP()
    time.sleep(0.05)
    print("[AVOID] Reversing...")
    DRIVE(REVERSE_SPEED, 0.0, 0.0)
    time.sleep(REVERSE_TIME)
    STOP()
    time.sleep(0.05)
    print("[AVOID] Turning...")
    DRIVE(0.0, 0.0, TURN_RATE)
    time.sleep(TURN_TIME)
    STOP()
    time.sleep(0.05)
    print("[AVOID] Done. Resuming forward.")

def main():
    print("Starting obstacle-aware drive. Press Ctrl+C to stop.")
    period = 1.0 / LOOP_HZ
    while True:
        t0 = time.time()
        dist = RANGE()
        if dist is None:
            print("[WARN] No distance reading; stopping for safety.")
            STOP()
        else:
            print(f"[INFO] Front distance: {dist:.3f} m")
            if dist < STOP_DISTANCE:
                avoid_obstacle()
            else:
                DRIVE(FORWARD_SPEED, 0.0, 0.0)

        # keep loop timing stable
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
