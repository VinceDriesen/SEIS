import math
import sys
from controller import Robot, Supervisor, Node
from src.turtleBot import TurtleBot, visualize_map
import matplotlib
import time

matplotlib.use("TkAgg")

print(sys.path)

TIME_STEP = 64
MAX_SPEED = 6.28

def main():
    # Use Supervisor to have additional simulation control.
    robot = Supervisor()  
    robotNode = robot.getFromDef('robot')
    
    # Get the initial global pose from the simulation node.
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED, get_global_pose(robotNode))
    
    # Print available device names for debugging.
    n = robot.getNumberOfDevices()
    for i in range(n):
        device = robot.getDeviceByIndex(i)
        print(device.getName())
    
    movements = [
        (1.0, 0.0, 0),   # Move forward
        (0.0, 0.0, 90),  # Rotate right
        (1.0, 0.0, 0),   # Move forward
        (0.0, 0.0, -90), # Rotate left
    ]
    
    while robot.step(TIME_STEP) != -1:
        for dx, dy, dtheta in movements:
            bot.move_position(dx, dy, dtheta)
            bot.scan_lidar()
            print(f"Position: {bot.position}")
            time.sleep(0.5)


def get_global_pose(node: Node):
    """
    Returns the initial (x, y, yaw) in global coordinates.
    Correctly handles Webots' 3x3 rotation matrix format.
    """
    position = node.getPosition()
    orientation = node.getOrientation()
    
    R00, R01, R02, R10, R11, R12, _, _, _ = orientation
    yaw = math.atan2(R01, R00)
    
    return (position[0], position[1], yaw)


if __name__ == "__main__":
    main()