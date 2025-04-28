#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------
#   run_controller.sh
#
#   Usage:
#     ./run_controller.sh [ROBOT_ID] [PORT] [MODE]
#
#   - SOURCES .env if present
#   - Sets ROBOT_ID (defaults to 0)
#   - MODE: "debug" (default) or "nodebug"
#   - Invokes the webots-controller wrapper with --robot-name
# ------------------------------------------------------------

# Load your .env file if it exists
if [ -f .env ]; then
  # shellcheck disable=SC1091
  source .env
fi

# Parse arguments
ROBOT_ID="${1:-0}"
DEBUG_PORT="${2:-5678}"
MODE="${3:-debug}"

export ROBOT_ID

ROBOT_NAME="robot_${ROBOT_ID}"

# Ensure WEBOTS_HOME is set
if [ -z "${WEBOTS_HOME:-}" ]; then
  echo "Error: WEBOTS_HOME is not set. Please export it (e.g. export WEBOTS_HOME=/usr/local/webots)"
  exit 1
fi

# Add Webots Python API to PYTHONPATH
export PYTHONPATH="${WEBOTS_HOME}/lib/controller/python:${PYTHONPATH:-}"

# Launch the controller
if [[ "$MODE" == "nodebug" ]]; then
  echo "▶ Running without debugging"
  exec "${WEBOTS_HOME}/webots-controller" \
    --robot-name="${ROBOT_NAME}" \
    .venv/bin/python "$(pwd)/main.py"
else
  echo "▶ Running with debug adapter on port $DEBUG_PORT"
  export ENABLE_DEBUG=1
  export DEBUG_PORT
  exec "${WEBOTS_HOME}/webots-controller" \
    --robot-name="${ROBOT_NAME}" \
    .venv/bin/python "$(pwd)/main.py"
fi
