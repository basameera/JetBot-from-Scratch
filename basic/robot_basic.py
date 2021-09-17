import setup_path
from jetbot_fs import Robot
import time

robot = Robot()

robot.forward(0.3)
time.sleep(5)
robot.backward(0.5)
time.sleep(5)
robot.stop()
