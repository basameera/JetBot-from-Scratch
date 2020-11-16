import setup_path
from jetbot_fs import Robot
import time

robot = Robot()

robot.left(0.3)
time.sleep(0.5)
robot.stop()
