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

# Install Webots
RUN mkdir -p /etc/apt/keyrings && \
    wget -qO- https://cyberbotics.com/Cyberbotics.asc | gpg --dearmor > /etc/apt/keyrings/cyberbotics.gpg && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian binary-amd64/" | tee /etc/apt/sources.list.d/Cyberbotics.list && \
    apt-get update -y && \
    apt-get install -y webots

# Copy application files
COPY --chown=app:app . .

# Create virtual environment as app user
USER app
RUN uv venv .venv && \
    uv sync --frozen --no-dev

# Environment variables
ENV PATH="/app/.venv/bin:$PATH"
ENV PYTHONPATH="/app:$PYTHONPATH"
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# Run controller
CMD ["/bin/bash", "run-controller.sh"]