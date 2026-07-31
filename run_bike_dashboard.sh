#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

mkdir -p startup_logs
LOG_FILE="${SCRIPT_DIR}/startup_logs/$(date +%Y%m%d_%H%M%S)_bike_dashboard.log"

echo "Starting Bike ADAS dashboard"
echo "Logging output to ${LOG_FILE}"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "--- $(date +%Y-%m-%d_%H:%M:%S) ---"

VENV_DIR="${SCRIPT_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python"
USE_VENV=1
PIP_SCOPE=""

if [ ! -d "${VENV_DIR}" ]; then
    echo "Virtual environment not found; creating .venv"
    if ! python3 -m venv "${VENV_DIR}"; then
        echo "WARNING: Unable to create virtual environment. Falling back to system Python."
        USE_VENV=0
    fi
fi

if [ "${USE_VENV}" -eq 1 ] && [ -f "${VENV_DIR}/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "${VENV_DIR}/bin/activate"
else
    VENV_PYTHON="$(command -v python3)"
    PIP_SCOPE="--user"
fi

echo "Using Python: $(command -v python3)"
echo "Checking Python dependencies..."
if ! "${VENV_PYTHON}" -m pip --version >/dev/null 2>&1; then
    echo "ERROR: pip is not available for ${VENV_PYTHON}."
    exit 1
fi

"${VENV_PYTHON}" -m pip install ${PIP_SCOPE} --upgrade pip setuptools wheel

if ! "${VENV_PYTHON}" -c "import serial" >/dev/null 2>&1; then
    echo "Python dependencies missing; installing from requirements.txt"
    "${VENV_PYTHON}" -m pip install ${PIP_SCOPE} -r requirements.txt
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
    echo "Workspace install/setup.bash not found. Run colcon build first."
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
exec ros2 launch display_node bike_dashboard.launch.py
