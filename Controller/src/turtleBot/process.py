from multiprocessing import Queue
from .turtleBotStateMachine import TASK_EXPLORE, TASK_MOVE_TO, TurtleBotSM
from controller import Robot, Supervisor, Node
import os
from threading import Thread


class Process:
    def __init__(self, name, pid):
        self.name = name
        self.pid = pid
        self.robot: Supervisor = Supervisor()
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.MAX_SPEED = 6.28
        # self.ROBOT_ID = os.getenc("ROBOT_ID", -1)
        self.tasks = []
        self.add_task({"type": TASK_EXPLORE, "params": {}})
        self.ROBOT_ID = 0
        if self.ROBOT_ID == -1:
            raise ValueError("ROBOT_ID not set. Please set it in the environment.")
        self.start_robot()

    def __repr__(self):
        return f"Process(name={self.name}, pid={self.pid})"

    def start_robot(self):
        self.bot: TurtleBotSM = TurtleBotSM(
            name=f"exploration_bot",
            robot=self.robot,
            time_step=self.TIME_STEP,
            max_speed=self.MAX_SPEED,
        )
        self.queue = Queue()
        # mqtt_client = MQTTController(self.ROBOT_ID, queue)
        simulation_thread = Thread(target=self.start)
        # mqtt_thread = Thread(
        #     target=self.start_mqtt.run
        # )
        simulation_thread.start()
        # mqtt_thread.start()
        simulation_thread.join()
        # mqtt_thread.join()

    def start(self):
        print(f"Starting process {self.name} with PID {self.pid}")
        try:
            print("\n--- Entering main Webots simulation loop ---")
            while self.robot.step(self.TIME_STEP) != -1:
                self.checkForTasks()
                robot_is_currently_busy = self.bot.updateTaskExecution()
                if not robot_is_currently_busy:
                    print(f"Robot {self.name} is idle.")
                    if self.tasks:
                        newTask = self.tasks.pop(0)
                        self.bot.executeTask(newTask)
                    else:
                        print("No tasks available for execution.")

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

    def checkForTasks(self):
        while not self.queue.empty():
            task = self.queue.get()
            print(f"Received task: {task}")
            self._add_task(task)

    def add_task(self, task):
        """Internal: Adds a task to the scheduler."""
        self.tasks.append(task)
