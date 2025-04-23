import threading
import queue
import time
from typing import Dict, List

from .turtleBot import TurtleBot


class Scheduler:
    def __init__(self):
        self.robots: List[TurtleBot] = list()
        self.task_queue = queue.Queue()
        self.robot_status: Dict[TurtleBot, str] = {robot: 'free' for robot in self.robots}
        self.robot_task_threads = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker_thread = threading.Thread(target=self._worker)
        self._worker_thread.daemon = True
        
    def add_robot(self, robot: TurtleBot):
        self.robots.append(robot)
        self.robot_status[robot] = 'free'
        
    def remove_robot(self, robot: TurtleBot):
        if robot in self.robots:
            self.robots.remove(robot)
            del self.robot_status[robot]
            if robot in self.robot_task_threads:
                del self.robot_task_threads[robot]
        
    def add_task(self, task: dict):
        """"
        Add a task to the queue.
        De dict bestaat uit: {type: 'move_to', 'parames': {'x': 1, 'y': 2}} Dit is dus voor de A* dus ABSOLUTE COORDINATEN
        """
        print(f"Adding task to queue: {task}")
        self.task_queue.put(task)
        
    def _worker(self):
        """
        Worker thread that processes tasks from the queue.
        This method runs in a separate thread and continuously checks for new tasks.
        """
        print("Scheduler worker thread started.")
        while not self._stop_event.is_set():
            task = None
            free_robot = None

            with self._lock:
                finished_robots = [r for r, status in self.robot_status.items()
                                   if status == 'busy' and r in self.robot_task_threads and not self.robot_task_threads[r].is_alive()]
                for robot in finished_robots:
                    self.robot_status[robot] = 'free'
                    print(f"Robot {robot.name if hasattr(robot, 'name') else robot} is now free.")
                    del self.robot_task_threads[robot]

                for robot in self.robots: 
                    if self.robot_status.get(robot) == 'free':
                        free_robot = robot
                        break 
            if free_robot is not None or not self.robots: 
                 try:
                     task = self.task_queue.get(timeout=0.1)
                 except queue.Empty:
                     pass 

            if task is not None and free_robot is not None:
                with self._lock:
                     if self.robot_status.get(free_robot) == 'free':
                         print(f"Assigning task {task.get('type')} to Robot {free_robot.name if hasattr(free_robot, 'name') else free_robot}")
                         self.robot_status[free_robot] = 'busy'
                         task_thread = threading.Thread(target=self._run_robot_task, args=(free_robot, task))
                         self.robot_task_threads[free_robot] = task_thread
                         task_thread.start()
                     else:
                         self.task_queue.put(task)

        print("Scheduler worker thread finished.")
        
    def _run_robot_task(self, robot: TurtleBot, task: dict):
        """
        Run a task on a robot. This method is run in a separate thread.
        """
        print("Running task on robot:", robot.name if hasattr(robot, 'name') else robot)
        try:
            success = robot.execute_task(task) # <- Deze lijn wordt nu uitgevoerd
            if success:
                print(f"Task {task.get('type')} completed successfully ...")
            else:
                 print(f"Task {task.get('type')} failed ...") 
        except Exception as e:
            print(f"Task {task.get('type')} failed due to exception {e}")
    
    def start(self):
        """
        Start the scheduler worker thread.
        """
        if not self._worker_thread.is_alive():
            self._stop_event.clear()
            self._worker_thread.start()
            print("Scheduler started.")
        else:
            print("Scheduler is already running.")
            
    def stop(self):
        """
        Stop the scheduler worker thread.
        """
        print("Stopping scheduler...")
        self._stop_event.set() 
        if self._worker_thread.is_alive():
            self._worker_thread.join()
        print("Scheduler stopped.")
        
    def are_all_robots_free(self) -> bool:
        """
        Checks if all robots managed by the scheduler are free.
        """
        with self._lock:
             finished_robots = [r for r, status in self.robot_status.items()
                                if status == 'busy' and not self.robot_task_threads.get(r).is_alive()]
             for robot in finished_robots:
                 self.robot_status[robot] = 'free'
                 print(f"Robot {robot.name if hasattr(robot, 'name') else robot} is now free (checked by are_all_robots_free).")
                 del self.robot_task_threads[robot]

             return all(status == 'free' for status in self.robot_status.values())

    def is_task_queue_empty(self) -> bool:
        """
        Checks if the task queue is empty.
        """
        return self.task_queue.empty()

    def is_busy(self) -> bool:
        """
        Checks if there are tasks in the queue or if any robot is busy.
        """
        return not self.is_task_queue_empty() or not self.are_all_robots_free()
        