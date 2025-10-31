import time

# Constants
SETPOINT = 0.25  # meters - desired distance from wall
TOLERANCE = 0.05  # meters - acceptable range around setpoint

def get_front_range_m(bot):
    """
    Get front range reading from lidar sensor.
    Returns distance in meters or None if reading unavailable.
    """
    try:
        if hasattr(bot, 'get_range'):
            return bot.get_range("front")
        elif hasattr(bot, 'get_front_distance'):
            return bot.get_front_distance()
        elif hasattr(bot, 'get_distance'):
            return bot.get_distance()
        else:
            return bot.get_range("front") 
    except Exception as e:
        return None

def follow_wall_loop(bot):
    """
    Follow controller using lidar sensor: maintains setpoint distance from wall/object.
    If object moves closer, bot backs up. If object moves away, bot moves forward.
    """
    while True:
        dist = get_front_range_m(bot)
        
        if dist is None:
            bot.stop()
            time.sleep(0.2)
            continue

        # If too close to wall/object (or object moved closer), back up to maintain setpoint
        if dist < SETPOINT - TOLERANCE:
            bot.drive(-0.2, 0.0, 0.0)  # back up
            time.sleep(0.1)  # Check more frequently for responsiveness
        # If at setpoint (within tolerance), maintain position
        elif abs(dist - SETPOINT) <= TOLERANCE:
            bot.stop()
            time.sleep(0.1)
        # If too far from wall/object (or object moved away), drive forward to reach setpoint
        else:
            bot.drive(0.2, 0.0, 0.0)  # forward
            time.sleep(0.1)  # Check more frequently for responsiveness
