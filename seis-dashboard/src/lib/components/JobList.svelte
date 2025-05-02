<script lang="ts">
	import type { Job } from '$lib/server/backendClient';
	import JobRow from './JobRow.svelte';

	let { jobList, title }: { jobList: Job[]; title: string } = $props();
</script>

<table>
	<caption>{title}</caption>
	<thead>
		<tr>
			<th>Job ID</th>
			<th>Position</th>
			<th>Completed</th>
			<th>Created</th>
		</tr>
	</thead>
	<tbody>
		{#each jobList as job}
			<tr>
				<td><JobRow {job} /></td>
			</tr>
		{/each}
	</tbody>
</table>
