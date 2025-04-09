import math
import os
from controller import Robot, Supervisor, Node
from src.turtleBot import TurtleBot
import matplotlib
import threading
import time

matplotlib.use("TkAgg")

TIME_STEP = 64
MAX_SPEED = 6.28

def main():
    # Use Supervisor to have additional simulation control.
    robot = Supervisor()  
    robotNode = robot.getFromDef('robot')
    
    # Get the initial global pose from the simulation node.
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED)
    
    # Print available device names for debugging.
    n = robot.getNumberOfDevices()
    for i in range(n):
        device = robot.getDeviceByIndex(i)
        print(device.getName())
        
        
    robot_loop(robot, bot, robotNode)
    # Start the simulation (movement/odometry) thread.
    # simulation_thread = threading.Thread(target=robot_loop, args=(robot, bot, robotNode))
    # simulation_thread.start()
    
    # visualization_thread = threading.Thread(target=visualization_loop, args=(bot,))
    # visualization_thread.daemon = True
    # visualization_thread.start()
    
    # simulation_thread.join()
    # visualization_thread.join()


def robot_loop(robot: Robot, bot: TurtleBot, supervisor_node: Node):
    """Modified movement pattern for better mapping"""
    print("currentPosition", bot.get_position())
    movements = [
        (0.5, 0.0, 0),   # Move forward
        (0.0, 0.0, 90), # Rotate left
        (1, 0.0, 0),   # Move forward
        (0, 0.0, 90),  # Rotate right
        (0.25, 0.0, 0),  # Rotate right
        (0, 0.0, 180),  # Rotate right
        (2, 0.0, 0),  # Rotate right
    ]
    
    bot.start_lidar()
    # bot.move_position(0, 0, -180)
    bot.move_position(0, 0, 90)
    bot.move_position(0, 0, 90)
    bot.move_position(0, 0, 90)
    bot.move_position(0, 0, 90)
    bot.move_to_position(1, 1)
    # bot.move_position(0.25,0,0)
    # bot.move_position(0,0.25,0)
    # time.sleep(5000)
    
    # while robot.step(TIME_STEP) != -1:
    #     for dx, dy, dtheta in movements:
    #         bot.move_position(dx, dy, dtheta)
    #         time.sleep(1)
    #     time.sleep(500)




if __name__ == "__main__":
    main()
