<script lang="ts">
	import { fromStore } from "svelte/store";
	import type { PageProps } from "./$types";

    let { data, form }: PageProps = $props()
</script>

<h1>Welcome to SvelteKit</h1>
<p>Visit <a href="https://svelte.dev/docs/kit">svelte.dev/docs/kit</a> to read the documentation</p>

<div class="prose">
    <h1>Publish Message</h1>
    <form method="post" action="?/publishMessage">
        <div>
            <label for="topic">Topic:</label>
            <input type="text" id="topic" name="topic" class="input"/>
        </div>
        <div>
            <label for="message">Message:</label>
            <textarea id="message" name="message" class="textarea"></textarea>
        </div>
        <button type="submit" class="btn btn-primary">Submit</button>
    </form>
    
    <h1>Get Last Message</h1>    
    <form method="post" action="?/getLatestMessage">
        <div>
            <label for="topic">Topic:</label>
            <input type="text" name="topic" id="topic" class="input" />
        </div>
        <button type="submit" class="btn btn-primary">Get</button>
    </form>

    {#if form?.success}
        <h2>Last Message:</h2>
        <p>Topic: {form.message?.topic} at {form.message?.timestamp}</p>
        <p>{form.message?.message}</p>
    {/if}
</div>
