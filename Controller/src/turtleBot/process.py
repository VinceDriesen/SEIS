from src.turtleBot.scheduler import Scheduler
from .turtleBotStateMachine import TASK_EXPLORE, TASK_MOVE_TO, TurtleBotSM
from controller import Robot, Supervisor, Node

class Process:
    def __init__(self, name, pid):
        self.name = name
        self.pid = pid
        self.robot: Supervisor = Supervisor()
        self.scheduler = Scheduler()
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.MAX_SPEED = 6.28


    def __repr__(self):
            return f"Process(name={self.name}, pid={self.pid})"

    def start(self):
        print(f"Starting process {self.name} with PID {self.pid}")

        try:

            self.scheduler = Scheduler() # Bijvoorbeeld, als Scheduler init leeg kan zijn
            print("Scheduler initialized.")


            print(f"Scheduler worker thread started for process {self.name}")
        
            self._start_process_exploration()
            self.scheduler.start() # Start de scheduler worker thread

            print("\n--- Entering main Webots simulation loop ---")
            while self.robot.step(self.TIME_STEP) != -1:
                for robot in self.scheduler.getManagedRobots(): # Of self.scheduler.get_managed_robots() als scheduler de master lijst heeft
                    # print(f"Processing robot: {robot.name if hasattr(robot, 'name') else robot}")
                    robot_is_currently_busy = robot.updateTaskExecution()
                    if not robot_is_currently_busy: # Voeg get_robot_status toe aan Scheduler
                        print(f"Detected robot {robot.name} finished task, marking as free.")
                pass # De main loop wacht hier op de volgende simulatie stap

            # --- Cleanly stop when simulation ends ---
            print("Main Webots simulation loop finished.")
            self.stop() # Roept scheduler.stop() aan
            return True

        except Exception as e:
            print(f"Error in process {self.name} start sequence: {e}")
            import traceback
            traceback.print_exc()
            # ... cleanup logic ...
            return False



    def stop(self):
        print(f"Stopping process {self.name}")
        self.scheduler.stop()
        print(f"Process {self.name} stopped.")
        return True

    def _start_process_exploration(self):
        try:
            bot: TurtleBotSM = self._create_robot(name=f"exploration_bot")
            print(f"Robot '{bot.name if hasattr(bot, 'name') else bot}' created for {self.name}.")

            self._add_robot(bot) 
            print(f"Robot '{bot.name if hasattr(bot, 'name') else bot}' added to scheduler.")

            initial_task = {
                'type': TASK_MOVE_TO,
                'params': {'x': 1, 'y': 0},
            }
            self._add_task(initial_task)
            print(f"Initial task '{initial_task['type']}' added to queue for {self.name}.")
            return True

        except Exception as e:
            print(f"Error during initial process exploration setup for {self.name}: {e}")
            import traceback
            traceback.print_exc()
            return False 


    def _create_robot(self, name):
        bot: TurtleBotSM = TurtleBotSM(name=name, robot=self.robot, time_step=self.TIME_STEP, max_speed=self.MAX_SPEED)
        return bot

    def _add_robot(self, robot):
        """Internal: Adds a robot to the scheduler."""
        self.scheduler.add_robot(robot)
        
    def _remove_bot(self, robot):
        """Internal: Removes a robot from the scheduler."""
        self.scheduler.remove_robot(robot)

    def _add_task(self, task):
        """Internal: Adds a task to the scheduler."""
        self.scheduler.add_task(task)
