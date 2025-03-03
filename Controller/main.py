import json

from controller import Robot, Motor, DistanceSensor
from src.turtleBot import TurtleBot

TIME_STEP = 64
MAX_SPEED = 6.28


def serialize(obj):
    if hasattr(obj, "__dict__"):
        return obj.__dict__  # Convert objects with __dict__ to dictionaries
    return str(obj)  # Convert unknown objects to strings


def main():
    robot = Robot()
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED)
    print(bot.getPosition())
    bot.movePosition(0.25, 0.25, 30)

    while robot.step(TIME_STEP) != -1:
        bot.movePosition(.25, .2, 45)
        pass

if __name__ == "__main__":
    main()
