import math
import numpy as np
from controller import Robot, Supervisor, Node
from src.turtleBot import TurtleBot, transform_lidar_scan
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
    simulation_thread = threading.Thread(target=robot_loop, args=(robot, bot))
    simulation_thread.start()
    
    # Start the visualization thread for the lidar scan.
    # visualization_thread = threading.Thread(target=visualization_loop, args=(bot, robotNode))
    visualization_thread = threading.Thread(target=visualization_loop, args=(bot, robotNode))
    visualization_thread.daemon = True  # Daemonize so it shuts down with the main thread.
    visualization_thread.start()
    
    simulation_thread.join()


def robot_loop(robot: Robot, bot: TurtleBot):
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


def visualization_loop(bot: TurtleBot, supervisor_node: Node):
    """Visualize using supervisor's ground truth positioning"""
    import matplotlib.pyplot as plt
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    
    while True:
        # Get GROUND TRUTH position and orientation
        true_pos = supervisor_node.getPosition()
        true_ori = supervisor_node.getOrientation()
        true_yaw = math.atan2(true_ori[3], true_ori[0])  # From 3x3 rotation matrix
        
        # Transform using ground truth values
        current_scan = np.array(bot.lidarSens.getRangeImage(), dtype=float)
        points_map = transform_lidar_scan(
            current_scan,
            (true_pos[0],  # World X
            true_pos[1],),  # World Y
            true_yaw      # World Yaw
        )
        
        # Update plot with ground truth data
        ax.clear()
        if points_map.size > 0:
            ax.scatter(points_map[:, 0], points_map[:, 1], s=2, c='blue')
        ax.scatter(true_pos[0], true_pos[1], s=50, c='red', marker='x')
        ax.quiver(true_pos[0], true_pos[1], 
                 math.cos(true_yaw), math.sin(true_yaw),
                 color='red', scale=10)
        plt.pause(0.05)


def get_global_pose(node: Node):
    """
    Returns the initial (x, y, yaw) in global coordinates.
    Correctly handles Webots' 3x3 rotation matrix format.
    """
    position = node.getPosition()  # [x, y, z]
    orientation = node.getOrientation()  # 3x3 rotation matrix as list[9]
    
    R00, R01, R02, R10, R11, R12, R20, R21, R22 = orientation
    
    # Calculate yaw from rotation matrix (Z-axis rotation)
    yaw = math.atan2(R10, R00)  # yaw = atan2(sinθ, cosθ)
    
    return (position[0], position[1], yaw)


if __name__ == "__main__":
    main()
