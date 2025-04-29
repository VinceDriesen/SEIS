#!/usr/bin/env bash
set -euo pipefail

# Parse arguments
ROBOT_ID="${ROBOT_ID:-0}"
DEBUG_PORT="${DEBUG_PORT:-5678}"
MODE="${MODE:-nodebug}"
WEBOTS_ROBOT_NAME="${WEBOTS_ROBOT_NAME:-ROBOT_0}"
IP_ADDRESS="${IP_ADDRESS:-0.0.0.0}"
WEBOTS_HOME="${WEBOTS_HOME}"
# LD_LIBRARY_PATH="${LD_LIBRARY_PATH}"
WEBOTS_ROBOT_NAME="robot_${ROBOT_ID}"

export ROBOT_ID
export WEBOTS_ROBOT_NAME
export WEBOTS_HOME
export PYTHONPATH
# export LD_LIBRARY_PATH

# Validate Webots installation
if [ -z "${WEBOTS_HOME:-}" ]; then
  echo "Error: WEBOTS_HOME is not set"
  exit 1
fi

# Configure Python paths
export PYTHONPATH="${WEBOTS_HOME}/lib/controller/python:${PYTHONPATH:-}"

# Webots controller binary
WEBOTS_CONTROLLER_BIN="${WEBOTS_HOME}/webots-controller"
[ ! -x "$WEBOTS_CONTROLLER_BIN" ] && WEBOTS_CONTROLLER_BIN=""

# Execution logic
cd "${PWD}"

export WEBOTS_CONTROLLER_URL="tcp://$IP_ADDRESS:1234/$WEBOTS_ROBOT_NAME"

# Use absolute path to virtual environment Python
PYTHON_BIN="/app/.venv/bin/python"

if [[ "$MODE" == "nodebug" ]]; then
  echo "▶ Running without debugging"
  if [ -n "$WEBOTS_CONTROLLER_BIN" ]; then
    echo "Webots Found!"
    exec "$WEBOTS_CONTROLLER_BIN" --protocol=tcp --ip-address="$IP_ADDRESS" --robot-name="$WEBOTS_ROBOT_NAME" "${PYTHON_BIN}" -u "${PWD}/main.py"
  else
    exec "${PYTHON_BIN}" -u "${PWD}/main.py"
  fi
else
  echo "▶ Running with debug adapter on port $DEBUG_PORT"
  export ENABLE_DEBUG=1 DEBUG_PORT
  if [ -n "$WEBOTS_CONTROLLER_BIN" ]; then
    echo "Webots Found!"
    exec "$WEBOTS_CONTROLLER_BIN" --protocol=tcp --ip-address="$IP_ADDRESS" --robot-name="$WEBOTS_ROBOT_NAME" \
      "${PYTHON_BIN}" "${PWD}/main.py"
  else
    exec "${PYTHON_BIN}" -u "${PWD}/main.py"
  fi
fi