# Stage 1: Build the SvelteKit app using Bun
FROM docker.io/oven/bun:latest AS builder
WORKDIR /app

# Copy package manifest files and install dependencies
COPY package.json bun.lock svelte.config.js ./
RUN bun install

# Copy the rest of the project source code
COPY . .

# Build the application (ensure your package.json defines the "build" script)
RUN bun run build

# Stage 2: Create the production image
FROM docker.io/oven/bun:latest
WORKDIR /app

# Copy built assets and necessary files from the builder stage
COPY --from=builder /app/.svelte-kit .svelte-kit
COPY --from=builder /app/static static
COPY --from=builder /app/package.json package.json
COPY --from=builder /app/bun.lock bun.lock

RUN bun add vite

# Expose the port (adjust if needed)
EXPOSE 5173

# Run the SvelteKit preview server (or replace with your production start command)
CMD ["bun", "run", "preview", "--", "--host", "0.0.0.0", "--port", "5173"]
