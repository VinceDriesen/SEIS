import threading
import time
from typing import Callable, Any

class TaskScheduler:
    def __init__(self):
        self.tasks = []
        self.running = False
        self.lock = threading.Lock()

    