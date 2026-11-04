# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=jazzy
ARG DEV_USER=duk
ARG DEV_UID=1000
ARG DEV_GID=1000

ENV ROS_DISTRO=${ROS_DISTRO} \
    VIRTUAL_ENV=/opt/duk-venv \
    PATH=/opt/duk-venv/bin:${PATH} \
    PYTHONPATH=/workspace/robot/software/core/src \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The official ROS ros-base image already provides the ROS build toolchain,
# git, colcon, rosdep, and vcstool. Add only development-container essentials
# that are not guaranteed by that image.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-venv \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Create one non-root developer account. Ubuntu 24.04-derived images may
# already contain UID/GID 1000, so reuse/rename those identities when present.
RUN set -eux; \
    existing_group="$(getent group "${DEV_GID}" | cut -d: -f1 || true)"; \
    if [[ -n "${existing_group}" && "${existing_group}" != "${DEV_USER}" ]]; then \
        groupmod --new-name "${DEV_USER}" "${existing_group}"; \
    elif [[ -z "${existing_group}" ]]; then \
        groupadd --gid "${DEV_GID}" "${DEV_USER}"; \
    fi; \
    existing_user="$(getent passwd "${DEV_UID}" | cut -d: -f1 || true)"; \
    if [[ -n "${existing_user}" && "${existing_user}" != "${DEV_USER}" ]]; then \
        usermod --login "${DEV_USER}" --home "/home/${DEV_USER}" --move-home "${existing_user}"; \
    elif [[ -z "${existing_user}" ]]; then \
        useradd --uid "${DEV_UID}" --gid "${DEV_GID}" --create-home --shell /bin/bash "${DEV_USER}"; \
    fi; \
    usermod --gid "${DEV_GID}" --shell /bin/bash "${DEV_USER}"; \
    echo "${DEV_USER} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${DEV_USER}"; \
    chmod 0440 "/etc/sudoers.d/${DEV_USER}"

# Keep Python research/test dependencies in the image. The workspace source is
# bind-mounted at runtime and takes precedence through PYTHONPATH, so source
# edits are immediately visible without an editable-install bootstrap step.
RUN python3 -m venv --system-site-packages "${VIRTUAL_ENV}"

COPY robot/software/core/pyproject.toml /tmp/duk-core/pyproject.toml
COPY robot/software/core/README.md /tmp/duk-core/README.md
COPY robot/software/core/src /tmp/duk-core/src

RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip install "/tmp/duk-core[test,research]" \
    && rm -rf /tmp/duk-core

# ROS and a built DUK overlay are sourced for interactive Bash terminals.
RUN printf '%s\n' \
    'if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then source "/opt/ros/$ROS_DISTRO/setup.bash"; fi' \
    'if [ -f "/workspace/robot/software/ros2_ws/install/setup.bash" ]; then source "/workspace/robot/software/ros2_ws/install/setup.bash"; fi' \
    > /etc/duk-ros.bash \
    && printf '%s\n' 'source /etc/duk-ros.bash' >> /etc/bash.bashrc

WORKDIR /workspace
USER ${DEV_USER}

CMD ["sleep", "infinity"]
