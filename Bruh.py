import time
import numpy as np
import inspect
from mbot_bridge.api import MBot

# ---------------- TUNABLES ----------------
SETPOINT   = 1.00   # meters: desired distance to wall
DEADBAND   = 0.10   # meters: OK band around setpoint
V_CMD      = 0.25   # m/s: bang-bang speed magnitude
Kp         = 1.5    # proportional gain for P-control
V_MAX      = 0.40   # m/s: clamp for P-control
LOOP_HZ    = 10.0   # control rate
USE_PCONTROL = True # False => bang-bang, True => P-control
FRONT_WINDOW = 5    # rays from each end of scan to average as "forward"
# ------------------------------------------

def bang_bang_control(state_dist, setpoint, v_cmd, deadband):
    """
    state_dist: current distance to wall (m)
    If too far => +v_cmd (forward), too close => -v_cmd (backward), inside band => 0
    """
    if state_dist is None or not np.isfinite(state_dist):
        return 0.0
    if state_dist > setpoint + deadband:
        return +abs(v_cmd)
    elif state_dist < setpoint - deadband:
        return -abs(v_cmd)
    else:
        return 0.0

def p_control(state_dist, setpoint, Kp, v_max, deadband):
    """
    Smooth speed: v = Kp*(state - setpoint), clamped to ±v_max, with deadband.
    Too far (state>setpoint) => positive v (forward). Too close => negative v (backward).
    """
    if state_dist is None or not np.isfinite(state_dist):
        return 0.0
    error = state_dist - setpoint
    if abs(error) <= abs(deadband):
        return 0.0
    v = Kp * error
    return float(np.clip(v, -abs(v_max), +abs(v_max)))

def forward_distance(ranges, angles, window=FRONT_WINDOW):
    """
    Estimate forward distance by averaging a few rays around 0 rad (front).
    Uses first 'window' and last 'window' samples as the front band.
    """
    if not ranges or not angles:
        return float('inf')
    r = np.array(ranges, dtype=float)
    a = np.array(angles, dtype=float)
    # Take rays near the front of the scan (start and end)
    idx = list(range(0, min(window, len(r)))) + list(range(max(0, len(r)-window), len(r)))
    r_sel = r[idx]
    a_sel = a[idx]
    # valid positive distances projected on robot x-axis
    valid = r_sel > 0
    if not np.any(valid):
        return float('inf')
    x = r_sel[valid] * np.cos(a_sel[valid])
    x = x[x > 0]  # forward only
    if x.size == 0:
        return float('inf')
    return float(np.mean(x))

# ---------- small adapters (kept minimal) ----------
def make_lidar_reader(bot):
    # Try common APIs: read_lidar(), get_scan(), read("lidar"/"LIDAR"/"lidar_scan"/"scan")
    if hasattr(bot, "read_lidar") and callable(getattr(bot, "read_lidar")):
        def rd():
            try:
                scan = bot.read_lidar()
                if isinstance(scan, dict):
                    ranges = scan.get("ranges") or scan.get("range")
                    angles = scan.get("angles") or scan.get("angle")
                    return ranges, angles
                return scan
            except Exception as e:
                if "no data on channel" in str(e).lower():
                    return None, None
                return None, None
        print("[INFO] LiDAR via bot.read_lidar()")
        return rd

    if hasattr(bot, "get_scan") and callable(getattr(bot, "get_scan")):
        def rd():
            try:
                scan = bot.get_scan()
                if scan is None:
                    return None, None
                if isinstance(scan, dict):
                    ranges = scan.get("ranges") or scan.get("range") or scan.get("distances")
                    angles = scan.get("angles") or scan.get("angle")
                    return ranges, angles
                try:
                    ranges, angles = scan
                    return ranges, angles
                except:
                    return scan, None
            except Exception as e:
                if "no data on channel" in str(e).lower():
                    return None, None
                return None, None
        print("[INFO] LiDAR via bot.get_scan()")
        return rd

    if hasattr(bot, "read") and callable(getattr(bot, "read")):
        chans = ("lidar", "LIDAR", "lidar_scan", "scan")
        print(f"[INFO] LiDAR via bot.read(), trying channels: {', '.join(chans)}")
        def rd():
            for ch in chans:
                try:
                    scan = bot.read(ch)
                    if scan is None:
                        continue
                    if isinstance(scan, dict):
                        ranges = scan.get("ranges") or scan.get("range") or scan.get("distances")
                        angles = scan.get("angles") or scan.get("angle")
                        return ranges, angles
                    try:
                        ranges, angles = scan
                        return ranges, angles
                    except:
                        return scan, None
                except Exception as e:
                    if "no data on channel" in str(e).lower():
                        return None, None
                    continue
            return None, None
        return rd

    print("[WARN] No known LiDAR API found; will report no data.")
    return lambda: (None, None)

def make_set_linear(bot):
    """
    Return a function set_vx(vx) that commands linear x only.
    Tries: set_vel(v,w), drive(vx,vy,wz), drive(vx,wz), motors(l,r). Else no-op.
    """
    if hasattr(bot, "set_vel"):
        print("[INFO] Drive via bot.set_vel(v,w)")
        return lambda vx: bot.set_vel(float(vx), 0.0)

    if hasattr(bot, "drive"):
        try:
            n = len(inspect.signature(bot.drive).parameters)
        except Exception:
            n = 3
        if n >= 3:
            print("[INFO] Drive via bot.drive(vx, vy, wz)")
            return lambda vx: bot.drive(float(vx), 0.0, 0.0)
        elif n == 2:
            print("[INFO] Drive via bot.drive(vx, wz)")
            return lambda vx: bot.drive(float(vx), 0.0)

    if hasattr(bot, "motors"):
        print("[INFO] Drive via bot.motors(l, r)")
        return lambda vx: bot.motors(float(vx), float(vx))

    print("[WARN] No known drive API; velocity commands will be ignored.")
    return lambda vx: None

def safe_stop(bot, set_vx):
    if hasattr(bot, "stop"):
        try:
            bot.stop(); return
        except: pass
    set_vx(0.0)

# --------------- main loop ----------------
def follow_1d():
    bot = MBot()
    read_lidar = make_lidar_reader(bot)
    set_vx = make_set_linear(bot)

    dt = 1.0 / LOOP_HZ
    print(f"[INFO] 1D follow starting (setpoint={SETPOINT} m, deadband={DEADBAND} m, "
          f"{'P-control' if USE_PCONTROL else 'Bang-Bang'})")

    try:
        while True:
            ranges, angles = read_lidar()
            if not ranges or not angles:
                # No LiDAR yet: hold and retry
                set_vx(0.0)
                time.sleep(0.1)
                continue

            dist = forward_distance(ranges, angles, window=FRONT_WINDOW)

            if USE_PCONTROL:
                vx = p_control(dist, SETPOINT, Kp, V_MAX, DEADBAND)
            else:
                vx = bang_bang_control(dist, SETPOINT, V_CMD, DEADBAND)

            set_vx(vx)
            print(f"dist={dist if np.isfinite(dist) else float('inf'):.3f} m | "
                  f"target={SETPOINT:.3f} m | cmd_vx={vx:+.3f} m/s")
            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        safe_stop(bot, set_vx)
        print("[INFO] Stopped.")

if __name__ == "__main__":
    follow_1d()
