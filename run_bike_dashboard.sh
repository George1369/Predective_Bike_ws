#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

mkdir -p startup_logs
LOG_FILE="${SCRIPT_DIR}/startup_logs/$(date +%Y%m%d_%H%M%S)_bike_dashboard.log"

echo "Starting Bike ADAS dashboard"
echo "Logging output to ${LOG_FILE}"
exec > >(tee -a "${LOG_FILE}") 2>&1

VENV_DIR="${SCRIPT_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python"

if [ ! -d "${VENV_DIR}" ]; then
    echo "Virtual environment not found; creating .venv"
    if ! python3 -m venv "${VENV_DIR}"; then
        echo "WARNING: could not create .venv, using system python instead"
        VENV_PYTHON="$(command -v python3)"
    fi
fi

if [ -f "${VENV_DIR}/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "${VENV_DIR}/bin/activate"
else
    VENV_PYTHON="$(command -v python3)"
fi

echo "Using Python: ${VENV_PYTHON}"

if ! "${VENV_PYTHON}" -m pip --version >/dev/null 2>&1; then
    echo "ERROR: pip is not available for ${VENV_PYTHON}"
    exit 1
fi

"${VENV_PYTHON}" -m pip install --upgrade pip setuptools wheel

if ! "${VENV_PYTHON}" -c "import serial" >/dev/null 2>&1; then
    echo "Python dependencies missing; installing from requirements.txt"
    "${VENV_PYTHON}" -m pip install -r requirements.txt
fi

ROS_SETUP=""
for candidate in \
    /opt/ros/lyrical/setup.bash \
    /opt/ros/humble/setup.bash \
    /opt/ros/iron/setup.bash \
    /opt/ros/jazzy/setup.bash \
    /opt/ros/rolling/setup.bash
do
    if [ -f "${candidate}" ]; then
        ROS_SETUP="${candidate}"
        break
    fi
done

if [ -z "${ROS_SETUP}" ]; then
    echo "ERROR: no ROS 2 setup.bash found under /opt/ros"
    exit 1
fi

# shellcheck disable=SC1091
source "${ROS_SETUP}"

if [ -f install/setup.bash ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
else
    echo "ERROR: install/setup.bash not found. Run colcon build first."
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
exec ros2 launch display_node bike_dashboard.launch.py
