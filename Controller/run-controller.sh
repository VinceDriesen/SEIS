#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------
#   run_controller.sh
#
#   Usage:
#     ./run_controller.sh [ROBOT_ID] [PORT] [MODE] [IP_ADDRESS]
#
#   - SOURCES .env if present
#   - Sets ROBOT_ID (defaults to 0)
#   - Exports WEBOTS_ROBOT_NAME (if needed, can also use --robot-name)
#   - MODE: "debug" (default) or "nodebug"
#   - Invokes the controller
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
IP_ADDRESS="${4:-"172.0.0.1"}"

export ROBOT_ID

echo $ROBOT_ID

WEBOTS_ROBOT_NAME="robot_${ROBOT_ID}"

# Check WEBOTS_HOME
if [ -z "${WEBOTS_HOME:-}" ]; then
  echo "Error: WEBOTS_HOME is not set. Please export it (e.g. export WEBOTS_HOME=/usr/local/webots)"
  exit 1
fi

# Add Webots Python API to PYTHONPATH
export PYTHONPATH="${WEBOTS_HOME}/lib/controller/python:${PYTHONPATH:-}"

# Prepare executable
WEBOTS_CONTROLLER_BIN="${WEBOTS_HOME}/webots-controller"

# Check if webots-controller binary exists
if [ ! -x "$WEBOTS_CONTROLLER_BIN" ]; then
  echo "Warning: webots-controller not found at $WEBOTS_CONTROLLER_BIN"
  echo "Assuming we're running directly without webots-controller."
  WEBOTS_CONTROLLER_BIN=""
fi

# Change to project root
cd "${PWD}"

# Launch controller
if [[ "$MODE" == "nodebug" ]]; then
  echo "▶ Running without debugging"
  if [ -n "$WEBOTS_CONTROLLER_BIN" ]; then
    exec "$WEBOTS_CONTROLLER_BIN" --protocol=tcp --ip-address="$IP_ADDRESS" --robot-name="$WEBOTS_ROBOT_NAME" \
      .venv/bin/python "${PWD}/main.py"
  else
    exec .venv/bin/python "${PWD}/main.py"
  fi
else
  echo "▶ Running with debug adapter on port $DEBUG_PORT"
  export ENABLE_DEBUG=1
  export DEBUG_PORT
  if [ -n "$WEBOTS_CONTROLLER_BIN" ]; then
    exec "$WEBOTS_CONTROLLER_BIN" --robot-name="$WEBOTS_ROBOT_NAME" \
      .venv/bin/python "${PWD}/main.py"
  else
    exec .venv/bin/python "${PWD}/main.py"
  fi
fi
