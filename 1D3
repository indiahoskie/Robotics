import time
from mbot_bridge.api import MBot

bot = MBot()

try:
    print("Moving forward...")
    bot.drive(0.2, 0.0, 0.0)   # forward
    time.sleep(2)

    print("Moving backward...")
    bot.drive(-0.2, 0.0, 0.0)  # backward
    time.sleep(2)

    print("Stopping.")
    bot.stop()
    time.sleep(1)

    print("Done!")

finally:
    bot.stop()
