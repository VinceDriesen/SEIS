import os
import time
import paho.mqtt.client as mqtt
import threading
from .turtleBotStateMachine import TASK_MOVE_TO
import json

MQTT_BROKER = os.getenv("MQTT_BROKER_URL", "localhost")


class MQTTController(threading.Thread):
    def __init__(self, robot_id, add_function: callable, update_map_func: callable):
        super().__init__(daemon=True)
        self.robot_id = robot_id
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self._last_coordinates = [0, 0]
        self.add_task = add_function
        self.update_map = update_map_func
        self.lock = threading.Lock()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        self.client.subscribe(f"robot/{self.robot_id}/jobs")
        self.client.subscribe(f"robot/{self.robot_id}/racks")
        self.client.subscribe(f"occupancy_map")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
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
            elif payload.startswith("occupancy_map"):
                _, occupancy_map_json = payload.split(":")
                occupancy_map = json.loads(occupancy_map_json)
                self.update_map(occupancy_map)

        except Exception as e:
            print(f"Error processing message: {e}")

    def publish_done(self, coordinates):
        with self.lock:
            x, y = coordinates
            self.client.publish(
                f"robot/{self.robot_id}/jobs", f"done:{x:.2f},{y:.2f}", qos=1
            )
            
    def publish_exploration_done(self, img_base64):
        with self.lock:
            self.client.publish(
                f"robot/{self.robot_id}/jobs", f"done:exploration_image:{img_base64}", qos=1
            )
            
    def sync_occupancy_map(self, occupancy_map):
        with self.lock:
            occupancy_map_json = json.dumps(occupancy_map)
            self.client.publish(
                f"occupancy_map", f"occupancy_map:{occupancy_map_json}", qos=1
            )

    def publish_location(self, coordinates):
        with self.lock:
            if coordinates == self._last_coordinates:
                return
            self._last_coordinates = coordinates
            x, y = coordinates
            self.client.publish(
                f"robot/{self.robot_id}/pos", f"{x:.2f},{y:.2f}", qos=1
            )

    def handle_rack_reservation(self, racks: set, reservation=True) -> bool:
        response_event = threading.Event()
        response = None

        def on_rack_reply(client, userdata, msg):
            nonlocal response
            try:
                response = msg.payload.decode().strip()
                print(f"Received rack response: {response}")
                response_event.set()
            except Exception as e:
                print(f"Failed to decode rack reply: {e}")

        topic = f"robot/{self.robot_id}/racks"
        rack_list_str = ",".join(map(str, sorted(racks)))
        
        if reservation:
            # Publish Message
            self.client.publish(topic, f"reserve:{rack_list_str}", qos=1)
            print("Reserving Racks")
        else:
            self.client.publish(topic, f"free:{rack_list_str}", qos=1)
            print("Freeing Racks")
            
        # Subscribe for callback
        self.client.subscribe(topic)
        self.client.message_callback_add(topic, on_rack_reply)

        # Start the loop in a separate thread if it's not already running
        self.client.loop_start()
        print("Waiting for response...")
        
        success = response_event.wait(timeout=5)
        self.client.message_callback_remove(topic)
        self.client.unsubscribe(topic)
        self.client.loop_stop()

        if not success:
            print("Rack reservation timed out")
            return False

        return response.lower() == "done"        
    
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
