import math
from controller import Robot, Supervisor, Node
from src.turtleBot import TurtleBot
import matplotlib
import threading

matplotlib.use("TkAgg")

TIME_STEP = 64
MAX_SPEED = 6.28


def main():
    # robot = Robot()
    robot = Supervisor()
    
    robotNode = robot.getFromDef('robot')
    
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED, get_global_pose(robotNode))

    n = robot.getNumberOfDevices()
    for i in range(n):
        device = robot.getDeviceByIndex(i)
        print(device.getName())

    robot_thread = threading.Thread(target=robot_loop, args=(robot, bot))
    robot_thread.start()
    robot_thread.join()


def robot_loop(robot: Robot, bot: TurtleBot):
    while robot.step(TIME_STEP) != -1:        
        bot.move_position(0.25, 0.2, 45)
        print(f'Position: X: {bot.position[0]}, Y: {bot.position[1]}, Thetha: {bot.position[2]}')
        pass


def get_global_pose(node: Node):
    """Return (x, y, yaw) in global coordinates"""
    position = node.getPosition()
    orientation = node.getOrientation()
    yaw = math.atan2(orientation[3], orientation[0])
    print(position, yaw)
    return (position[0], position[1], yaw)

if __name__ == "__main__":
    main()
