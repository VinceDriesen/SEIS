FROM ubuntu:24.04

# Create app user and directory structure first
RUN useradd -m app && \
    mkdir /app && \
    chown app:app /app

# Set working directory (now properly owned by app)
WORKDIR /app

# Install system dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3 python3-pip bash ca-certificates wget curl gnupg2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install uv
RUN curl -LsSf https://astral.sh/uv/install.sh | bash && \
    mv /root/.local/bin/uv /usr/local/bin/uv

# Copy application files
COPY --chown=app:app . .

# Create virtual environment as app user
USER app

ENV USER=app

RUN uv venv .venv && \
    uv sync --frozen --no-dev

# Environment variables
ENV PATH="/app/.venv/bin:$PATH"
ENV PYTHONPATH="/app:$PYTHONPATH"
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Run controller
CMD ["/bin/bash", "run-controller.sh"]