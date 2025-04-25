<script lang="ts">
	import type { Job } from "$lib/server/backendClient";

    let { job }: { job: Job } = $props()
</script>

<tr>
    <td>{job.job_id}</td>
    <td>X: {job.x}, Y: {job.y}</td>
    <td>{job.completed}</td>
    <td>{job.created}</td>
</tr>