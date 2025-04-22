# Use a multi-stage build with uv for dependency management
FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim AS builder
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy
ENV UV_PYTHON_DOWNLOADS=0

WORKDIR /app

# Create a non-root user
RUN useradd -m app

COPY pyproject.toml uv.lock /app/

# Use caching for dependencies
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-install-project --no-dev

# Copy the project files and install dependencies
ADD . /app
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-dev

# Ensure saved_models directory exists and set correct permissions
RUN mkdir -p /app/saved_models && chown -R app:app /app/saved_models

# Change ownership to the app user
RUN chown -R app:app /app

# Final stage: slim Python image without uv
FROM python:3.12-slim-bookworm

WORKDIR /app

# Create the app user
RUN useradd -m app

# Copy built application from builder stage
COPY --from=builder --chown=app:app /app /app

# Ensure saved_models directory exists and set correct permissions
RUN mkdir -p /app/saved_models && chown -R app:app /app/saved_models

# Set up environment
ENV PATH="/app/.venv/bin:$PATH"
ENV PYTHONPATH=/app:$PYTHONPATH
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Switch to the app user
USER app

# Expose port for FastAPI or Django app
EXPOSE 8000

# Run the application
CMD ["python", "-u", "/app/backend/manage.py", "runserver"]
