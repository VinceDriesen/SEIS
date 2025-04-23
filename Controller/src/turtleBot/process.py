from src.turtleBot.scheduler import Scheduler
from src.turtleBot.turtleBot import TurtleBot
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

        exploration_setup_ok = self._start_process_exploration()
        self.scheduler.start()

        if not exploration_setup_ok:
            print("Exploration setup failed. Exiting.")
            return False

        print(f"Scheduler worker thread started for process {self.name}")

        print("\n--- Entering main Webots simulation loop ---")
        while self.robot.step(self.TIME_STEP) != -1:
            pass

        print("Main Webots simulation loop finished.")

        self.stop()
        return True


    def stop(self):
        print(f"Stopping process {self.name}")
        self.scheduler.stop()
        print(f"Process {self.name} stopped.")
        return True

    def _start_process_exploration(self):
        try:
            bot: TurtleBot = self._create_robot(name=f"exploration_bot")
            print(f"Robot '{bot.name if hasattr(bot, 'name') else bot}' created for {self.name}.")

            self._add_robot(bot) 
            print(f"Robot '{bot.name if hasattr(bot, 'name') else bot}' added to scheduler.")

            initial_task = {
                'type': 'explore_environment',
                'params': {}
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
        bot: TurtleBot = TurtleBot(name=name, robot=self.robot, timeStep=self.TIME_STEP, maxSpeed=self.MAX_SPEED)
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
