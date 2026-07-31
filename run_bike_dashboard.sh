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

VENV_SITE_PACKAGES="$("${VENV_PYTHON}" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
# ROS entry points use the system interpreter, so expose packages installed in
# this workspace venv to the interpreter that runs the generated ROS scripts.
export PYTHONPATH="${VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

if ! "${VENV_PYTHON}" -c "import serial, smbus2" >/dev/null 2>&1; then
    echo "Python dependencies missing; installing from requirements.txt"
    "${VENV_PYTHON}" -m pip install -r requirements.txt
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 is not on PATH. Source your ROS 2 environment first."
    exit 1
fi

if [ -f install/setup.bash ]; then
    # shellcheck disable=SC1091
    set +u
    source install/setup.bash
    set -u
else
    echo "ERROR: install/setup.bash not found. Run colcon build first."
    exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
exec ros2 launch display_node bike_dashboard.launch.py
