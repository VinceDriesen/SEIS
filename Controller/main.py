import numpy as np
from controller import Robot
from src.turtleBot import TurtleBot
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')

TIME_STEP = 64
MAX_SPEED = 6.28


def serialize(obj):
    if hasattr(obj, "__dict__"):
        return obj.__dict__  # Convert objects with __dict__ to dictionaries
    return str(obj)  # Convert unknown objects to strings


def main():
    robot = Robot()
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED)
    
    n = robot.getNumberOfDevices()
    for i in range(n):
        device = robot.getDeviceByIndex(i)
        print(device.getName())
    
        
    while robot.step(TIME_STEP) != -1:
        bot.movePosition(.25, .2, 45)
        bot._scan_lidar_event()
        pass

if __name__ == "__main__":
    main()
