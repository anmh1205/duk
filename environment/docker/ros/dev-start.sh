#!/usr/bin/env bash
set -euo pipefail

target_uid="$(id -u)"
target_gid="$(id -g)"

for path in /opt/duk-ros/build /opt/duk-ros/install /opt/duk-ros/log; do
  sudo mkdir -p "${path}"
  owner="$(stat -c '%u:%g' "${path}")"
  if [[ "${owner}" != "${target_uid}:${target_gid}" ]]; then
    sudo chown -R "${target_uid}:${target_gid}" "${path}"
  fi
done

exec sleep infinity
