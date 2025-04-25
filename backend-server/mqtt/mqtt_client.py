import os
import paho.mqtt.client as mqtt
import logging
import threading
import time
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
import uuid


@dataclass
class RobotJob:
    job_id: str
    x: float
    y: float
    created: float
    completed: Optional[float] = None


@dataclass
class RobotPosition:
    x: float
    y: float
    timestamp: float
    heading: Optional[float] = None

MQTT_BROKER_URL = os.getenv("MQTT_BROKER_URL", "localhost")

class MQTTManager(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.broker = MQTT_BROKER_URL
        self.port = 1883
        self.client = mqtt.Client()
        self.robots: Dict[str, Dict] = {}  # robot_id: {jobs, position}
        self.lock = threading.Lock()
        self._setup_logging()
        self._running = True
        # Initialize robots with IDs 0, 1, 2
        for robot_id in ["0", "1", "2"]:
            self.robots[robot_id] = {"jobs": [], "position": None}

    def _setup_logging(self):
        logging.basicConfig(
            format="%(asctime)s - %(levelname)s - %(message)s", level=logging.INFO
        )
        self.logger = logging.getLogger("MQTTJobManager")

    def _on_connect(self, client, userdata, flags, rc):
        self.logger.info(f"Connected to MQTT broker (code {rc})")
        client.subscribe("robot/+/jobs")
        client.subscribe("robot/+/pos")

    def _on_message(self, client, userdata, msg):
        try:
            topic_parts = msg.topic.split("/")
            robot_id = topic_parts[1]
            msg_type = topic_parts[2]

            with self.lock:
                if robot_id not in self.robots:
                    self.robots[robot_id] = {"jobs": [], "position": None}

                payload = msg.payload.decode()

                if msg_type == "jobs":
                    self._handle_job_message(robot_id, payload)
                elif msg_type == "pos":
                    self._handle_position_message(robot_id, payload)

        except Exception as e:
            self.logger.error(f"Error processing message: {str(e)}")

    def _handle_job_message(self, robot_id: str, payload: str):
        if payload.startswith("job:"):
            _, coords = payload.split(":", 1)
            x, y = map(float, coords.split(","))
            job = RobotJob(job_id=str(uuid.uuid4()), x=x, y=y, created=time.time())
            self.robots[robot_id]["jobs"].append(job)
            self.logger.info(f"New job for {robot_id} at ({x}, {y})")

        elif payload.startswith("done:"):
            _, job_id = payload.split(":", 1)
            for job in self.robots[robot_id]["jobs"]:
                if job.job_id == job_id and not job.completed:
                    job.completed = time.time()
                    self.logger.info(f"Job {job_id} completed by {robot_id}")
                    break

    def _handle_position_message(self, robot_id: str, payload: str):
        try:
            parts = payload.split(",")
            x, y = map(float, parts[:2])
            heading = float(parts[2]) if len(parts) > 2 else None

            self.robots[robot_id]["position"] = RobotPosition(
                x=x, y=y, heading=heading, timestamp=time.time()
            )
        except ValueError:
            self.logger.error(f"Invalid position format: {payload}")

    def run(self):
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.on_connect = self._on_connect
            self.client.on_message = self._on_message
            self.client.loop_start()

            while self._running:
                time.sleep(0.1)

        finally:
            self.client.loop_stop()
            self.client.disconnect()
            self.logger.info("MQTT connection closed")

    def stop(self):
        self._running = False

    def create_job(self, x: float, y: float, robot_id: str) -> Optional[str]:
        """Create new job and return job ID"""
        topic = f"robot/{robot_id}/jobs"
        job_id = str(uuid.uuid4())
        message = f"job:{x},{y}"

        try:
            result = self.client.publish(topic, message, qos=1)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                with self.lock:
                    self.robots.setdefault(robot_id, {"jobs": [], "position": None})
                    self.robots[robot_id]["jobs"].append(
                        RobotJob(job_id=job_id, x=x, y=y, created=time.time())
                    )
                return job_id
            return None
        except Exception as e:
            self.logger.error(f"Job creation failed: {str(e)}")
            return None

    def get_job_status(self, job_id: str) -> bool:
        with self.lock:
            for robot in self.robots.values():
                for job in robot["jobs"]:
                    if job.job_id == job_id:
                        return job.completed is not None
            return False

    def get_robot_job_status(self, robot_id: str) -> Tuple[list, list]:
        """Return (pending_jobs, completed_jobs)"""
        with self.lock:
            robot = self.robots.get(robot_id, {"jobs": []})
            pending = [j for j in robot["jobs"] if not j.completed]
            completed = [j for j in robot["jobs"] if j.completed]
            return pending, completed

    def get_position(self, robot_id: str) -> Optional[RobotPosition]:
        """Get latest known position"""
        with self.lock:
            pos = self.robots.get(robot_id, {}).get("position")
            return pos if pos and (time.time() - pos.timestamp < 60) else None

    def get_all_robots(self) -> list:
        """Get list of all tracked robot IDs"""
        with self.lock:
            return list(self.robots.keys())
