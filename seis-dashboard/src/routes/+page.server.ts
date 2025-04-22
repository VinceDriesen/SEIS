import type { Actions } from '@sveltejs/kit';
import type { PageServerLoad } from './$types';
import { createJob, getRobotJobLists, getRobots } from '$lib/server/backendClient';

export const load = (async () => {
    const { data: robots } = await getRobots()

    const jobs = robots ? await Promise.all(robots.map( async (value) => {
        const { data } = await getRobotJobLists(value);
        if (data) {
            return data;
        }
        return null;
    })) : [];

    return {
        robots,
        jobs,
    };
}) satisfies PageServerLoad;


export const actions = {
    createJob: async ({ request }) => {
        const formdata = await request.formData()
        const x_pos_raw = formdata.get('x_pos')
        const y_pos_raw = formdata.get('y_pos')
        const robot_id_raw = formdata.get('robot_id');

        const x_pos = x_pos_raw ? Number(x_pos_raw) : null
        const y_pos = y_pos_raw ? Number(y_pos_raw) : null 
        const robot_id = robot_id_raw ? Number(robot_id_raw) : null;

        if(x_pos === null || y_pos === null || robot_id === null) {
            return {
                success: false,
                message: "Failed to get position/robot_id"
            }
        }

        const { data, error } = await createJob(x_pos, y_pos, robot_id)

        if(error && data !== undefined) {
            return {
                success: false,
                message: error.message
            }
        } else {
            return {
                success: true,
                data: data,
            }
        }
    },
} satisfies Actions;