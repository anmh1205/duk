#!/usr/bin/env bash
set -euo pipefail

mode="${1:---quick}"

if [[ "${mode}" != "--health" && "${mode}" != "--quick" && "${mode}" != "--full" ]]; then
  echo "usage: bash tools/environment-check/verify-dev-container.sh [--health|--quick|--full]" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"

if [[ ! -f "${repo_root}/compose.yaml" || ! -d "${repo_root}/robot/software/core" ]]; then
  echo "DUK repository is not available at expected workspace root: ${repo_root}" >&2
  exit 1
fi

if [[ "$(id -u)" == "0" ]]; then
  echo "development container must not run as root" >&2
  exit 1
fi

if [[ "$(id -un)" != "duk" ]]; then
  echo "unexpected development user: $(id -un)" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO:?ROS_DISTRO is not set}/setup.bash" ]]; then
  echo "ROS setup file is missing for ROS_DISTRO=${ROS_DISTRO}" >&2
  exit 1
fi

if [[ "${VIRTUAL_ENV:-}" != "/opt/duk-venv" ]]; then
  echo "unexpected VIRTUAL_ENV: ${VIRTUAL_ENV:-<unset>}" >&2
  exit 1
fi

for ros_path in \
  "/opt/duk-ros/build" \
  "/opt/duk-ros/install" \
  "/opt/duk-ros/log"; do
  if [[ ! -d "${ros_path}" || ! -w "${ros_path}" ]]; then
    echo "ROS workspace volume is not writable: ${ros_path}" >&2
    exit 1
  fi
done

if [[ "${mode}" == "--health" ]]; then
  if [[ ! -x "/opt/duk-venv/bin/python" ]]; then
    echo "DUK Python virtual environment is not executable" >&2
    exit 1
  fi
  PYTHONPATH="${repo_root}/robot/software/core/src" \
    /opt/duk-venv/bin/python -c "import duk_core" >/dev/null
  exit 0
fi

# ROS environment scripts are not required for the recurring health probe.
# Disable nounset while sourcing because upstream setup scripts are not
# guaranteed to be compatible with a caller's `set -u`.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

for command_name in python git colcon ros2 rosdep vcs jupyter; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "required command is missing: ${command_name}" >&2
    exit 1
  fi
done

python - "${repo_root}" <<'PY'
from pathlib import Path
import sys

repo_root = Path(sys.argv[1]).resolve()
expected_source = (repo_root / "robot/software/core/src").resolve()
expected_prefix = Path("/opt/duk-venv").resolve()

if Path(sys.prefix).resolve() != expected_prefix:
    raise SystemExit(f"unexpected Python prefix: {sys.prefix}")

import duk_core
import matplotlib
import numpy
import rclpy
import scipy
import sympy

module_path = Path(duk_core.__file__).resolve()
if expected_source not in module_path.parents:
    raise SystemExit(
        f"duk_core is not loaded from workspace source: {module_path}"
    )

print(f"python={sys.version.split()[0]}")
print(f"python_prefix={sys.prefix}")
print(f"duk_core={module_path}")
print(f"rclpy={Path(rclpy.__file__).resolve()}")
print(f"numpy={numpy.__version__}")
print(f"scipy={scipy.__version__}")
print(f"sympy={sympy.__version__}")
print(f"matplotlib={matplotlib.__version__}")
PY

if [[ "${mode}" == "--full" ]]; then
  if [[ "${COLCON_DEFAULTS_FILE:-}" != "/etc/duk-colcon-defaults.yaml" ]]; then
    echo "unexpected COLCON_DEFAULTS_FILE: ${COLCON_DEFAULTS_FILE:-<unset>}" >&2
    exit 1
  fi

  (
    cd "${repo_root}/robot/software/ros2_ws"
    colcon build >/dev/null
  )

  (
    cd "${repo_root}"
    python robot/software/core/examples/duk_leg_kinematics.py \
      --angle-deg 75 \
      --branch 1 \
      >/dev/null

    python -m pytest -q -p no:cacheprovider robot/software/core/tests
  )
fi

echo "DUK development container verification: OK (${mode#--})"
