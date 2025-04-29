from time import sleep
from src.turtleBot.mqtt_client import MQTTController
from .turtleBotStateMachine import TASK_EXPLORE, TASK_MOVE_TO, TurtleBotSM
from controller import Robot
import os
import queue
from threading import Thread


class Process:
    def __init__(self, name, pid, explore=False):
        self.name = name
        self.pid = pid
        self.robot_id = os.getenv("ROBOT_ID", -1)
        if self.robot_id == -1:
            raise ValueError("ROBOT_ID not set. Please set it in the environment.")

        self.robot = Robot()
        if not self.robot:
            raise ConnectionError("Failed to connect to Webots simulation")

        print(f"Connected to Webots: {self.robot.getName()}")
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        print(f"timestep: {self.TIME_STEP}")
        self.MAX_SPEED = 6.28
        self.tasks = queue.Queue()

        if explore:
            self._add_task({"type": TASK_EXPLORE, "params": {}})

        self.start_robot()

    def __repr__(self):
        return f"Process(name={self.name}, pid={self.pid})"

    def start_robot(self):
        self.bot = TurtleBotSM(
            name=f"exploration_bot",
            robot=self.robot,
            time_step=self.TIME_STEP,
            max_speed=self.MAX_SPEED,
        )

        self.mqtt_client = MQTTController(self.robot_id, self._add_task)
        simulation_thread = Thread(target=self.start, daemon=True)
        mqtt_thread = Thread(target=self.mqtt_client.run, daemon=True)

        simulation_thread.start()
        mqtt_thread.start()

        while True:
            sleep(1)

    def start(self):
        print(f"Starting process {self.name} with PID {self.pid}")
        try:
            print("\n--- Entering main Webots simulation loop ---")
            current_task = None
            print(f"first time_step: {self.robot.step(self.TIME_STEP)}")
            while self.robot.step(self.TIME_STEP) != -1:
                robot_is_currently_busy = self.bot.updateTaskExecution(
                    self.mqtt_client.publish_location
                )

                if not robot_is_currently_busy:
                    if current_task is not None:
                        if current_task["type"] == TASK_MOVE_TO:
                            x = current_task["params"].get("x")
                            y = current_task["params"].get("y")
                            self.mqtt_client.publish_done((x, y))
                            current_task = None

                    # Get task from queue in thread-safe manner
                    try:
                        new_task = self.tasks.get(block=False)
                    except queue.Empty:
                        print(f"Robot {self.name} is idle. No tasks available.")
                    else:
                        print(f"Executing new task: {new_task}")
                        self.bot.executeTask(new_task)
                        current_task = new_task

            print("Main Webots simulation loop finished.")
            self.stop()
            return True

        except Exception as e:
            print(f"Error in process {self.name} start sequence: {e}")
            import traceback

            traceback.print_exc()
            return False

    def stop(self):
        print(f"Stopping process {self.name}")
        print(f"Process {self.name} stopped.")
        return True

    def _add_task(self, task):
        """Thread-safe task addition"""
        self.tasks.put(task)
