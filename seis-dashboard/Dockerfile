# 1) builder
FROM oven/bun:latest AS builder
WORKDIR /app
COPY package.json bun.lock svelte.config.js ./
RUN bun install
COPY . .
RUN bun run build

# 2) runner
FROM node:23-slim
WORKDIR /app

COPY --from=builder /app/package.json ./package.json
COPY --from=builder /app/build       ./build

# Tell the Node server to listen on 5173
ENV PORT=5173

# install only prod deps
RUN npm install --omit=dev

EXPOSE 5173
CMD ["node", "build/index.js"]
