from src.turtleBot.mqtt_client import MQTTController
from .turtleBotStateMachine import TASK_EXPLORE, TASK_MOVE_TO, TurtleBotSM
from controller import Supervisor
import os
from threading import Thread


class Process:
    def __init__(self, name, pid, explore = False):
        self.name = name
        self.pid = pid
        self.robot_id = os.getenv("ROBOT_ID", -1)
        if self.robot_id == -1:
            raise ValueError("ROBOT_ID not set. Please set it in the environment.")
        self.robot: Supervisor = Supervisor()
        print(f"Robot Supervisor Object: {self.robot}, Name: {self.robot.getName()}, Time Step: {self.robot.getBasicTimeStep()}")
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.MAX_SPEED = 6.28
        self.tasks = []
        if explore:
            self._add_task({"type": TASK_EXPLORE, "params": {}})
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
        
        self.mqtt_client = MQTTController(self.robot_id, self.tasks, self._add_task)
        simulation_thread = Thread(target=self.start)
        mqtt_thread = Thread(target=self.mqtt_client.run, daemon=True)
        
        simulation_thread.start()
        mqtt_thread.start()
        
        simulation_thread.join()
        # mqtt_thread.join()


    def start(self):
        print(f"Starting process {self.name} with PID {self.pid}")
        try:
            print("\n--- Entering main Webots simulation loop ---")
            current_task = None
            while self.robot.step(self.TIME_STEP) != -1:
                robot_is_currently_busy = self.bot.updateTaskExecution(self.mqtt_client.publish_location)
                if not robot_is_currently_busy:
                    if current_task is not None:
                        if current_task['type'] == TASK_MOVE_TO:
                            x = current_task['params'].get('x')
                            y = current_task['params'].get('y')
                            self.mqtt_client.publish_done((x, y))
                            current_task = None
                    print(f"Robot {self.name} is idle.")
                    
                    if self.tasks:
                        newTask = self.tasks.pop(0)
                        self.bot.executeTask(newTask)
                        current_task = newTask
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


    def _add_task(self, task):
        """Internal: Adds a task to the scheduler."""
        self.tasks.append(task)
