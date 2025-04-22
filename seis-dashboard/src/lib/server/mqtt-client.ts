import mqtt, { MqttClient, type IClientOptions, type Packet } from 'mqtt';

// const MQTT_URL = 'mqtt://mosquitto-broker:1883';
const MQTT_URL = 'mqtt://localhost:1883'

interface MqttMessage {
	topic: string;
	message: string;
	timestamp: Date;
}

interface MqttOptions {
	brokerUrl: string;
	options?: IClientOptions;
}

class MqttService {
	private client: MqttClient | null = null;
	private messageStore = new Map<string, MqttMessage>();
	private config: MqttOptions;

	constructor(config: MqttOptions) {
		this.config = config;
		this.initClient();
	}

	private initClient() {
		this.client = mqtt.connect(this.config.brokerUrl, this.config.options);

		this.client.on('connect', () => console.log('Connected To Broker!'));

		this.client.on('message', (topic, message) => {
			this.storeMessage(topic, message.toString());
		});

		this.client.on('error', (err) => {
			console.error('MQTT error:', err);
		});
	}

	private storeMessage(topic: string, message: string) {
		this.messageStore.set(topic, {
			topic,
			message,
			timestamp: new Date()
		});
	}

	public async publish(topic: string, message: string, options: IClientOptions = {}): Promise<Packet | undefined> {
		if (!this.client?.connected) {
			throw new Error('MQTT client not connected');
		}

		return new Promise((resolve, reject) => {
			this.client?.publish(topic, message, options, (err, packet) => {
				if (err) reject(err);
				else resolve(packet);
			});
		});
	}

	public async getLastMessage(topic: string): Promise<MqttMessage | undefined> {
		if (!this.client?.connected) {
			throw new Error('MQTT client not connected');
		}

		// Subscribe if not already subscribed
		if (!this.messageStore.has(topic)) {
			await new Promise((resolve, reject) => {
				this.client?.subscribe(topic, (err) => {
					if (err) reject(err);
					else resolve(null);
				});
			});
		}

		return this.messageStore.get(topic);
	}

	public disconnect() {
		this.client?.end();
	}
}

// Create a singleton instance
const mqttService = new MqttService({
	brokerUrl: MQTT_URL,
	options: {
		clientId: `sveltekit-server-${Math.random().toString(16).substr(2, 8)}`,
	}
});

export default mqttService;
