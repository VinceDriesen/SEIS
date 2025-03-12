import math
import numpy as np
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
    visualization_thread = threading.Thread(target=visualization_loop, args=(bot,))
    visualization_thread.daemon = True  # Daemonize so it shuts down with the main thread.
    visualization_thread.start()
    
    simulation_thread.join()


def robot_loop(robot: Robot, bot: TurtleBot):
    """
    Main simulation loop: issues move commands and updates the robot's pose.
    The TurtleBot's _move() function internally uses the lidar scan for odometry correction.
    """
    while robot.step(TIME_STEP) != -1:
        bot.move_position(0.25, 0.2, 45)
        print(f'Position: X: {bot.position[0]:.2f}, Y: {bot.position[1]:.2f}, Theta: {bot.position[2]:.2f}')
        # You can add a short sleep if needed to slow the simulation.
        time.sleep(0.1)


def visualization_loop(bot: TurtleBot):
    """
    Continuously grabs the current lidar scan from the TurtleBot,
    transforms it to world coordinates, and updates a live plot.
    """
    import matplotlib.pyplot as plt
    plt.ion()  # Turn on interactive mode.
    fig, ax = plt.subplots()
    
    while True:
        # Retrieve the current lidar scan from the sensor.
        current_scan = bot.lidarSens.getRangeImage()  # Returns a list of distances.
        current_scan = np.array(current_scan, dtype=float)
        
        # Import the transform function from your lidar module.
        from src.turtleBot import transform_lidar_scan
        
        # Transform the scan into world coordinates using the robot's current pose.
        points_map = transform_lidar_scan(
            current_scan,
            (bot.position[0], bot.position[1]),
            (math.cos(bot.position[2]), math.sin(bot.position[2]))
        )
        
        ax.clear()
        if points_map.size > 0:
            ax.scatter(points_map[:, 0], points_map[:, 1], s=2, c='blue', label="Lidar Points")
        ax.scatter(bot.position[0], bot.position[1], s=50, c='red', marker='x', label="Robot Position")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Lidar Map Visualization")
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        plt.pause(0.05)  # Pause briefly to update the plot.


def get_global_pose(node: Node):
    """
    Returns the initial (x, y, yaw) in global coordinates for the given simulation node.
    """
    position = node.getPosition()
    orientation = node.getOrientation()
    yaw = math.atan2(orientation[3], orientation[0])
    print("Initial position:", position, "Yaw:", yaw)
    return (position[0], position[1], yaw)


if __name__ == "__main__":
    main()
