<script lang="ts">
	import JobList from "$lib/components/JobList.svelte";
	import type { PageProps } from "./$types";

    let { data, form }: PageProps = $props()

    let occupancyMap = data?.occupancyMap || null;
</script>

<div class="prose max-w-none flex flex-col md:flex-row gap-4 w-full w-screen">
    <div class="flex-1">
        <h1>Send Job</h1>
        <form method="post" action="?/createJob">
            <div>
                <label for="topic">Robot:</label>
                <select name="robot-id" id="robot-id" class="select">
                    {#each data.robots || [] as robot (robot)}
                    <option value="robot">{robot}</option>
                    {/each}
                </select>
            </div>
            <div>
                <label for="message">X Position:</label>
                <input type="number" name="x-pos" id="x-pos" class="input" step="any"/>
            </div>
            <div>
                <label for="message">Y Position:</label>
                <input type="number" name="y-pos" id="y-pos" class="input" step="any"/>
            </div>
            <button type="submit" class="btn btn-primary">Submit</button>
        </form>
    </div>

    <div class="flex-1">
        {#each data.jobs || [] as jobList, index}
             <div>
                <p>Robot {index}</p>
                <JobList jobList={jobList?.[0] ?? []} title={"Pending"}/>
                <JobList jobList={jobList?.[1] ?? []} title={"Completed"}/>
             </div>
        {/each}
    </div>

    <div class="flex-1">
        {#if occupancyMap}
            <!-- Afbeelding van de occupancy map -->
            <div class="occupancy-map" style="width: 100%; height: 400px; background-color: #e0e0e0; border: 2px solid #333;">
                <!-- Gebruik de base64-string direct in de src -->
                <img src={occupancyMap} alt="Occupancy Map" style="width: 100%; height: 100%; object-fit: contain;" />
            </div>
        {:else}
            <p>Loading occupancy map...</p>
        {/if}
    </div>
    
    
</div>
