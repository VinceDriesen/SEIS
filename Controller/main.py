import os
from queue import Queue
from controller import Robot, Supervisor, Node
from src.turtleBot import TurtleBot, MQTTController
import matplotlib
import threading
import time

matplotlib.use("TkAgg")

ROBOT_ID = os.getenv("ROBOT_ID", -1)

if ROBOT_ID == -1:
    raise ValueError("ROBOT_ID environment variable is not set or invalid.")

TIME_STEP = 64
MAX_SPEED = 6.28


def main():
    # Use Supervisor to have additional simulation control.
    robot = Supervisor()
    robotNode = robot.getFromDef(f"robot_{ROBOT_ID}")

    # Get the initial global pose from the simulation node.
    bot: TurtleBot = TurtleBot(robot, TIME_STEP, MAX_SPEED)

    # Print available device names for debugging.
    # n = robot.getNumberOfDevices()
    # for i in range(n):
    #     device = robot.getDeviceByIndex(i)
    #     print(device.getName())

    # Create command queue, connect to mqtt broker
    command_queue = Queue()
    mqtt_client = MQTTController(ROBOT_ID, command_queue)

    # Start the threads.
    simulation_thread = threading.Thread(
        target=robot_loop, args=(robot, bot, command_queue, mqtt_client)
    )
    mqtt_thread = threading.Thread(target=mqtt_client.run)

    simulation_thread.start()
    mqtt_thread.start()

    simulation_thread.join()
    mqtt_thread.join()


def robot_loop(
    robot: Robot, bot: TurtleBot, command_queue: Queue, mqtt_client: MQTTController
):
    print("currentPosition", bot.get_position())

    # bot.explore_environment()
    # time.sleep(20000)

    while robot.step(TIME_STEP) != -1:
        while not command_queue.empty():
            cmd_type, data = command_queue.get()
            if cmd_type == "move":
                x, y = data
                if bot.move_to_position(x, y):
                    mqtt_client.publish_done((x, y))
                else:
                    print(f"Failed To Complete Task: {x}, {y}")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
