<script lang="ts">
	import { onMount } from 'svelte';
	import JobList from '$lib/components/JobList.svelte';
	import type { PageProps } from './$types';

	let { data, form }: PageProps = $props();
	let occupancyMap = data?.occupancyMap ?? null;

	// ── CONFIGURE BASED ON YOUR MAP SIZE ──
	const mapSize = 5;
	const minX = -mapSize / 2,
		maxX = mapSize / 2;
	const minY = -mapSize / 2,
		maxY = mapSize / 2;

	let imgEl: HTMLImageElement | undefined = $state();

	type RobotPos = { id: number; px: number; py: number };
	let robotPositions: RobotPos[] = $state([]);

	const colors = ['#e63946', '#2a9d8f', '#264653'];

	// normalize world→[0,1], project to pixels
	function project(list: { robot_id: number; x: number; y: number }[]) {
		if (!imgEl || !imgEl.complete) return;
		const { width, height } = imgEl.getBoundingClientRect();

		robotPositions = list.map((r) => {
			const nx = (r.x - minX) / (maxX - minX);
			const ny = (maxY - r.y) / (maxY - minY); // flip Y
			return {
				id: r.robot_id,
				px: nx * width,
				py: ny * height
			};
		});
	}

	onMount(() => {
		// open SSE to your Django‑Ninja stream
		const url = 'http://backend:8000/api/robotStream';
		const es = new EventSource(url);

		es.onmessage = (e) => {
			try {
				const list = JSON.parse(e.data) as { robot_id: number; x: number; y: number }[];
				project(list);
			} catch {
				// ignore parse errors
			}
		};

		es.onerror = () => {
			console.error('SSE error, retrying…');
			// browser auto‑reconnects by default
		};

		return () => es.close();
	});
</script>

<div class="prose flex w-full w-screen max-w-none flex-col gap-4 md:flex-row">
	<!-- ── Send Job Form ── -->
	<div class="flex-1">
		<h1>Send Job</h1>
		<form method="post" action="?/createJob">
			<div>
				<label for="robot-id">Robot:</label>
				<select name="robot-id" id="robot-id" class="select">
					{#each data.robots ?? [] as robot}
						<option value={robot}>{robot}</option>
					{/each}
				</select>
			</div>
			<div>
				<label for="x-pos">X Position:</label>
				<input type="number" name="x-pos" id="x-pos" class="input" step="any" />
			</div>
			<div>
				<label for="y-pos">Y Position:</label>
				<input type="number" name="y-pos" id="y-pos" class="input" step="any" />
			</div>
			<button type="submit" class="btn btn-primary">Submit</button>
		</form>
	</div>

	<!-- ── Job Lists ── -->
	<div class="flex-1">
		{#each data.jobs ?? [] as jobList, index}
			<div>
				<p>Robot {index}</p>
				<JobList jobList={jobList?.[0] ?? []} title="Pending" />
				<JobList jobList={jobList?.[1] ?? []} title="Completed" />
			</div>
		{/each}
	</div>

	<!-- ── Occupancy Map + Live Robot Overlay ── -->
	<div class="flex-1">
		{#if occupancyMap}
			<div class="relative" style="width:100%; height:400px;">
				<img
					bind:this={imgEl}
					src={occupancyMap}
					alt="Occupancy Map"
					style="width:100%; height:100%; object-fit:contain;"
				/>

				<svg class="pointer-events-none absolute inset-0" style="width:100%; height:100%;">
					{#each robotPositions as { id, px, py } (id)}
						<circle
							cx={px}
							cy={py}
							r="6"
							fill={colors[id] ?? 'white'}
							stroke="black"
							stroke-width="1"
						/>
						<text
							x={px}
							y={py - 10}
							font-size="12"
							text-anchor="middle"
							fill={colors[id] ?? 'black'}>{id}</text
						>
					{/each}
				</svg>
			</div>
		{:else}
			<p>Loading occupancy map…</p>
		{/if}
	</div>
</div>
