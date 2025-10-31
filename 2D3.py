import time
import numpy as np

# Constants
SETPOINT = 0.25  # meters - desired distance from obstacle
TOLERANCE = 0.05  # meters - acceptable range around setpoint

def find_min_dist(lidar_scan):
    """Find the minimum value in a lidar scan vector."""
    if not lidar_scan or len(lidar_scan) == 0:
        return None
    return min(lidar_scan)

def find_min_nonzero_dist(lidar_scan):
    """Find the minimum value in a lidar scan, ignoring zeros (bad readings)."""
    if not lidar_scan or len(lidar_scan) == 0:
        return None
    nonzero_distances = [d for d in lidar_scan if d > 0]
    if len(nonzero_distances) == 0:
        return None
    return min(nonzero_distances)

def follow_2d_loop(bot):
    """Follow 2D: maintains setpoint distance from nearest obstacle using lidar scan."""
    while True:
        # Get lidar scan - try different API patterns
        try:
            if hasattr(bot, 'get_lidar_scan'):
                distances, angles = bot.get_lidar_scan()
            elif hasattr(bot, 'get_scan'):
                scan = bot.get_scan()
                distances = scan.get('distances') if isinstance(scan, dict) else scan
                angles = scan.get('angles') if isinstance(scan, dict) else None
            else:
                distances, angles = None, None
        except:
            distances, angles = None, None
        
        if distances is None:
            bot.stop()
            time.sleep(0.2)
            continue
        
        # Find minimum nonzero distance and its angle
        min_dist = None
        min_angle = None
        for i, dist in enumerate(distances):
            if dist > 0:
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                    min_angle = angles[i] if angles and i < len(angles) else None
        
        if min_dist is None:
            bot.stop()
            time.sleep(0.2)
            continue
        
        # Maintain setpoint distance
        if min_dist < SETPOINT - TOLERANCE:
            # Too close: move away from obstacle
            vx = -0.2 * np.cos(min_angle) if min_angle is not None else 0.0
            vy = -0.2 * np.sin(min_angle) if min_angle is not None else 0.0
            bot.drive(vx, vy, 0.0)
            time.sleep(0.1)
        elif abs(min_dist - SETPOINT) <= TOLERANCE:
            # At setpoint: stop
            bot.stop()
            time.sleep(0.1)
        else:
            # Too far: move toward obstacle
            vx = 0.2 * np.cos(min_angle) if min_angle is not None else 0.2
            vy = 0.2 * np.sin(min_angle) if min_angle is not None else 0.0
            bot.drive(vx, vy, 0.0)
            time.sleep(0.1)
