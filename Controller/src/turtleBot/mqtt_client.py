import os
import paho.mqtt.client as mqtt
import threading
from queue import Queue

MQTT_BROKER = os.getenv("MQTT_BROKER_URL", "localhost")


class MQTTController(threading.Thread):
    def __init__(self, robot_id, command_queue):
        super().__init__(daemon=True)
        self.robot_id = robot_id
        self.command_queue = command_queue
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        self.client.subscribe(f"robot/{self.robot_id}/job")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            if payload.startswith("job:"):
                _, coords = payload.split(":")
                x, y = map(float, coords.split(","))
                print(f"Received job: Move to ({x}, {y})")
                self.command_queue.put(("move", (x, y)))
        except Exception as e:
            print(f"Error processing message: {e}")

    def publish_done(self, coordinates):
        x, y = coordinates
        self.client.publish(f"robot/{self.robot_id}", f"done:{x:.2f},{y:.2f}")

    def run(self):
        self.client.connect(MQTT_BROKER, 1883, 60)
        self.client.loop_forever()
