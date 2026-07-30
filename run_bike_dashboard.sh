#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

mkdir -p startup_logs
LOG_FILE="${SCRIPT_DIR}/startup_logs/$(date '+%Y%m%d_%H%M%S')_bike_dashboard.log"

echo "Starting Bike ADAS dashboard"
echo "Logging output to ${LOG_FILE}"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "--- $(date '+%Y-%m-%d %H:%M:%S') ---"

VENV_DIR="${SCRIPT_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python"

if [ ! -d "${VENV_DIR}" ]; then
    echo "Virtual environment not found; creating .venv"
    if ! python3 -m venv "${VENV_DIR}"; then
        echo "ERROR: Unable to create virtual environment. Install python3-venv and rerun."
        exit 1
    fi
fi

if [ -f "${VENV_DIR}/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "${VENV_DIR}/bin/activate"
else
    echo "ERROR: Virtual environment activation script missing."
    exit 1
fi

echo "Using Python: $(command -v python3)"
echo "Checking Python dependencies..."
"${VENV_PYTHON}" -m pip install --upgrade pip setuptools wheel

if ! "${VENV_PYTHON}" -c 'import serial' >/dev/null 2>&1; then
    echo "Python dependencies missing; installing from requirements.txt"
    "${VENV_PYTHON}" -m pip install -r requirements.txt
else
    echo "Required Python dependencies already installed."
fi

ROS_SETUP=""
for candidate in /opt/ros/humble/setup.bash /opt/ros/iron/setup.bash /opt/ros/foxy/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/rolling/setup.bash; do
    if [ -f "${candidate}" ]; then
        ROS_SETUP="${candidate}"
        break
    fi
done

if [ -z "${ROS_SETUP}" ]; then
    echo "No supported ROS 2 installation found in /opt/ros. Please install ROS 2 or edit this script."
    exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"

if [ -f install/setup.bash ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
else
    echo "Workspace install/setup.bash not found. Run 'colcon build' first."
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
exec ros2 launch display_node bike_dashboard.launch.py
