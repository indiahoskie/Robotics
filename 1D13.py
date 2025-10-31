
#!/usr/bin/env python3
import time
from mbot_bridge.api import MBot

# ====== Tunables ======
V_FWD        = 0.30     # base forward speed
Kp           = 0.8      # proportional gain on encoder delta
Ki           = 0.15     # integral gain (helps steady bias)
TRIM_CLAMP   = 0.30     # max |trim| (fraction of base speed)
DRIVE_TIME_S = 4.0      # drive duration
LOOP_HZ      = 40       # control loop rate
# ======================

robot = MBot()

def read_encoders():
    """
    Returns (left, right) cumulative encoder distance (or ticks).
    Tries several common API shapes.
    """
    candidates = ["read_encoders", "readEncoders", "get_encoders", "getEncoders", "encoders"]
    for name in candidates:
        if hasattr(robot, name):
            obj = getattr(robot, name)
            data = obj() if callable(obj) else obj
            # tuple
            if isinstance(data, tuple) and len(data) == 2:
                return float(data[0]), float(data[1])
            # dict
            if isinstance(data, dict):
                if "left" in data and "right" in data:
                    return float(data["left"]), float(data["right"])
                if "l" in data and "r" in data:
                    return float(data["l"]), float(data["r"])
            # object with attrs
            if hasattr(data, "left") and hasattr(data, "right"):
                return float(data.left), float(data.right)
            if hasattr(data, "l") and hasattr(data, "r"):
                return float(data.l), float(data.r)
    return None, None

def motors(vl, vr):
    """
    Send per-wheel speeds if available; otherwise degrade to drive(vx, wz=0).
    """
    if hasattr(robot, "motors"):
        robot.motors(float(vl), float(vr))
    elif hasattr(robot, "drive"):
        # emulate by average forward, zero yaw (may drift if wheels unbalanced)
        try:    robot.drive((vl+vr)/2.0, 0.0, 0.0)
        except: robot.drive((vl+vr)/2.0, 0.0)
    elif hasattr(robot, "setVelocity"):
        robot.setVelocity((vl+vr)/2.0, 0.0, 0.0)
    elif hasattr(robot, "set_cmd_vel"):
        robot.set_cmd_vel((vl+vr)/2.0, 0.0)
    else:
        raise RuntimeError("No known motor/drive API found.")

def stop():
    try:
        if hasattr(robot, "motors"):
            robot.motors(0.0, 0.0)
        elif hasattr(robot, "stop"):
            robot.stop()
        elif hasattr(robot, "drive"):
            try:    robot.drive(0.0, 0.0, 0.0)
            except: robot.drive(0.0, 0.0)
        elif hasattr(robot, "setVelocity"):
            robot.setVelocity(0.0, 0.0, 0.0)
        elif hasattr(robot, "set_cmd_vel"):
            robot.set_cmd_vel(0.0, 0.0)
    except Exception:
        pass

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def main():
    dt = 1.0/LOOP_HZ
    # try encoders
    L0, R0 = read_encoders()
    use_enc = (L0 is not None and R0 is not None)
    integ = 0.0

    t0 = time.time()
    try:
        if not use_enc:
            print("[WARN] No encoders detected; falling back to open-loop trim.")
            TRIM = 0.03  # tweak sign/size manually to cancel drift
            while time.time() - t0 < DRIVE_TIME_S:
                vl = V_FWD * (1.0 + TRIM)
                vr = V_FWD * (1.0 - TRIM)
                motors(vl, vr)
                time.sleep(dt)
            stop()
            return

        # Encoder-balanced straight drive
        print("[INFO] Using encoder-balanced straight drive.")
        # Re-sample baseline to reduce jitter
        time.sleep(0.05)
        L0, R0 = read_encoders()

        while time.time() - t0 < DRIVE_TIME_S:
            L, R = read_encoders()
            if L is None or R is None:
                # lost encoders → fail safe
                motors(0.0, 0.0)
                time.sleep(dt)
                continue

            # We want left and right to advance equally → error = (R-L)
            err = (R - L)
            integ += err * dt
            trim = Kp*err + Ki*integ
            trim = clamp(trim, -TRIM_CLAMP, TRIM_CLAMP)

            # Apply opposite adjustments to each wheel around base speed
            vl = V_FWD * (1.0 - trim)
            vr = V_FWD * (1.0 + trim)

            motors(vl, vr)
            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        stop()
        time.sleep(0.1)

if __name__ == "__main__":
    main()
