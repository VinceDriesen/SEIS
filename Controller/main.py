import math
import os
from controller import Robot, Supervisor, Node
from src.turtleBot import TurtleBot, visualize_map
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
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED, get_global_pose(robotNode))
    
    # Print available device names for debugging.
    n = robot.getNumberOfDevices()
    for i in range(n):
        device = robot.getDeviceByIndex(i)
        print(device.getName())
        
    # Start the simulation (movement/odometry) thread.
    simulation_thread = threading.Thread(target=robot_loop, args=(robot, bot, robotNode))
    simulation_thread.start()
    
    visualization_thread = threading.Thread(target=visualization_loop, args=(bot,))
    visualization_thread.daemon = True
    visualization_thread.start()
    
    simulation_thread.join()
    visualization_thread.join()


def robot_loop(robot: Robot, bot: TurtleBot, supervisor_node: Node):
    """Modified movement pattern for better mapping"""
    movements = [
        (1.0, 0.0, 0),   # Move forward
        (0.0, 0.0, 90),  # Rotate right
        (1.0, 0.0, 0),   # Move forward
        (0.0, 0.0, -90), # Rotate left
    ]
    
    while robot.step(TIME_STEP) != -1:
        for dx, dy, dtheta in movements:
            bot.move_position(dx, dy, dtheta)
            print(f"Position: {bot.position}")
            time.sleep(0.5)


def visualization_loop(bot: TurtleBot):
    """Loop to periodically update the visualization"""
    map_dir = "occupancy_maps"
    os.makedirs(map_dir, exist_ok=True)
    
    counter = 0
    while True:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(map_dir, f"occupancy_map_{timestamp}_{counter:04d}.png")
        visualize_map(bot, save_path)
        time.sleep(1.0)  # Update every second


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
