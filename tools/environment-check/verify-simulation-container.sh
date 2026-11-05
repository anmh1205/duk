#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

# Dev Containers starts lifecycle hooks concurrently with the image CMD.
# Wait until duk-dev-start has initialized ownership of all three ROS volumes.
ready=false
for ((attempt=0; attempt<100; attempt++)); do
  if bash "${repo_root}/tools/environment-check/verify-dev-container.sh" --health >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 0.2
done
if [[ "${ready}" != true ]]; then
  echo "Simulation container did not finish volume initialization within 20 seconds" >&2
  exit 1
fi
bash "${repo_root}/tools/environment-check/verify-dev-container.sh" --quick
python -c 'from mpl_toolkits.mplot3d import Axes3D; print("Matplotlib 3D import: OK")'
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
command -v rviz2
command -v gz
