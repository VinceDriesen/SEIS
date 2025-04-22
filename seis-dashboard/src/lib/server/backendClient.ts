import type { paths, components } from "$lib/types/schema";
import createClient from "openapi-fetch";

const BASE_URL = process.env.BASE_URL || "http://localhost:8000";
const client = createClient<paths>({ baseUrl: BASE_URL });

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