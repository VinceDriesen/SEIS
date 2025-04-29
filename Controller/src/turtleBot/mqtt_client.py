import os
import time
import paho.mqtt.client as mqtt
import threading
from .turtleBotStateMachine import TASK_MOVE_TO

MQTT_BROKER = os.getenv("MQTT_BROKER_URL", "localhost")


class MQTTController(threading.Thread):
    def __init__(self, robot_id, add_function: callable):
        super().__init__(daemon=True)
        self.robot_id = robot_id
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.add_task = add_function
        self.lock = threading.Lock()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        self.client.subscribe(f"robot/{self.robot_id}/jobs")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            if payload.startswith("job:"):
                _, coords = payload.split(":")
                x, y = map(float, coords.split(","))
                print(f"Received job: Move to ({x}, {y})")
                self.add_task(
                    {
                        "type": TASK_MOVE_TO,
                        "params": {
                            "x": x,
                            "y": y,
                        },
                    }
                )
        except Exception as e:
            print(f"Error processing message: {e}")

    def publish_done(self, coordinates):
        with self.lock:
            x, y = coordinates
            self.client.publish(
                f"robot/{self.robot_id}/jobs", f"done:{x:.2f},{y:.2f}", qos=1
            )

    def publish_location(self, coordinates):
        with self.lock:
            x, y = coordinates
            self.client.publish(
                f"robot/{self.robot_id}/pos", f"pos:{x:.2f},{y:.2f}", qos=1
            )

    def run(self):
        while True:
            try:
                self.client.connect(MQTT_BROKER, 1883, 60)
                self.client.loop_start()
                while True:
                    time.sleep(0.01)  # Yield control
            except Exception as e:
                print(f"Connection lost: {e}, reconnecting in 5s...")
                time.sleep(5)
