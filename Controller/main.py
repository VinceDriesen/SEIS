import numpy as np
from controller import Robot
from src.turtleBot import TurtleBot
import matplotlib
import threading
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")

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

    robot_thread = threading.Thread(target=robot_loop, args=(robot, bot))
    robot_thread.start()

    bot.display_occupancy_map()

    robot_thread.join()


def robot_loop(robot: Robot, bot: TurtleBot):
    while robot.step(TIME_STEP) != -1:
        bot._scan_lidar_event()
        print(
            f"Bot positions: x={bot.position[0]}, y={bot.position[1]}, theta={bot.position[2]}"
        )
        bot.movePosition(0.25, 0.2, 45)
        pass


if __name__ == "__main__":
    main()
