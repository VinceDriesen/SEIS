import type { Actions } from '@sveltejs/kit';
import type { PageServerLoad } from './$types';
import mqttService from '$lib/server/mqtt-client';

export const load = (async () => {
	return {};
}) satisfies PageServerLoad;


export const actions = {
    publishMessage: async ({ request }) => {
        const formdata = await request.formData()
        const topic = formdata.get('topic') as string
        const message = formdata.get('message') as string

        await mqttService.publish(topic, message)
    },

    getLatestMessage: async ({ request }) => {
        const formdata = await request.formData()
        const topic = formdata.get('topic') as string

        const message = await mqttService.getLastMessage(topic)

        console.log(message)

        if (message) {
            return {
                success: true,
                message: message,
            }
        } else {
            return {
                success: false,
            }
        }
    },
} satisfies Actions;