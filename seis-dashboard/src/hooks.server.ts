import mqttService from '$lib/server/mqtt-client';

import type { Handle } from '@sveltejs/kit';

export const handle: Handle = async ({ event, resolve }) => {
	// Clean up on server shutdown
	process.once('SIGINT', () => {
		mqttService.disconnect();
		process.exit();
	});

	return resolve(event);
};
