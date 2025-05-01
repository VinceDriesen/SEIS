import type { paths, components } from "$lib/types/schema";
import { env } from "$env/dynamic/private";
import createClient from "openapi-fetch";

// console.log(env.BASE_URL, "wieee")
const base_url = 'http://backend:8000'; 

const client = createClient<paths>({ baseUrl: base_url });

export type Job = components['schemas']['RobotJob']

export async function createJob(x: number, y: number, robot_id: number) {
    const { data, error } = await client.POST("/api/createJob", {
        params: {
            query: {
                x: x,
                y: y,
                robot_id: robot_id,
            }
        }
    })

    return { data, error }
}

export async function isJobCompleted(job_id: string) {
    const { data, error } = await client.GET('/api/isJobCompleted', {
        params: {
            query: {
                job_id: job_id,
            }
        }
    })

    return { data, error }
}

export async function getRobotPos(robot_id: number) {
    const { data, error } = await client.GET('/api/getRobotPos', {
        params: {
            query: {
                robot_id: robot_id,
            }
        }
    })

    return { data, error }
}

// TODO
// export async function getRobotStatus(robot_id: number) {
// 
// }

export async function getRobotJobLists(robot_id: number) {
    const { data, error } = await client.GET("/api/getRobotJobList", {
        params: {
            query: {
                robot_id: robot_id
            }
        }
    })

    return { data, error }
}

export async function getRobots() {
    const { data, error } = await client.GET('/api/getRobots', {})
    return { data, error }
}

export const getOccupancyMap = async () => {
    const { data, error } = await client.GET("/api/occupancy-map", {});
    if (!error) {
        // Veronderstel dat de server enkel de base64-gegevens terugstuurt
        const base64Data = await data.trim('"');
        
        // Voeg de juiste prefix toe
        return `data:image/png;base64,${base64Data}`;
    }
    
    return null;
};
