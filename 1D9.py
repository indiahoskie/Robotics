from mbot_bridge.api import MBot
import time

bot = MBot()
TARGET_DIST = 0.5  # meters
SPEED = 0.2

while True:
    dist = bot.read_ultrasonic()  # read distance sensor
    error = dist - TARGET_DIST

    if error > 0.05:
        # too far from wall → move closer
        bot.drive(SPEED, 0)
    elif error < -0.05:
        # too close → move away
        bot.drive(-SPEED, 0)
    else:
        # within tolerance → stop
        bot.drive(0, 0)
    
    time.sleep(0.1)
