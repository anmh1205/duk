# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO=jazzy
ARG DEV_USER=duk
ARG DEV_UID=1000
ARG DEV_GID=1000

ENV ROS_DISTRO=${ROS_DISTRO} \
    VIRTUAL_ENV=/opt/duk-venv \
    PATH=/opt/duk-venv/bin:${PATH} \
    PYTHONPATH=/workspace/robot/software/core/src \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    COLCON_DEFAULTS_FILE=/etc/duk-colcon-defaults.yaml

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The official ROS ros-base image already provides the ROS build toolchain,
# git, colcon, rosdep, and vcstool. Add only development-container essentials
# that are not guaranteed by that image.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-venv \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Create one non-root developer account. Ubuntu-derived images can already
# contain UID/GID 1000, so reuse/rename those identities when present.
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

# Keep ROS Python packages visible while isolating DUK's Python dependency set.
RUN python3 -m venv --system-site-packages "${VIRTUAL_ENV}"

# Humble is based on Ubuntu 22.04 and exposes older system packaging modules
# through --system-site-packages. Install a modern toolchain into the venv first
# so PEP 517 builds consistently shadow those system copies.
RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip install --upgrade \
        "pip>=25,<26" \
        "setuptools>=77,<80" \
        "wheel>=0.45,<1" \
        "packaging>=24,<26"

# Install the core's declared test/research dependency set into the image.
# At runtime the bind-mounted workspace source takes precedence via PYTHONPATH,
# so ordinary source edits require no editable-install bootstrap.
COPY robot/software/core/pyproject.toml /tmp/duk-core/pyproject.toml
COPY robot/software/core/README.md /tmp/duk-core/README.md

# Resolve the declared extras without copying the live source into the image.
# A minimal package scaffold satisfies setuptools package discovery; the real
# package always comes from the bind-mounted workspace through PYTHONPATH.
RUN mkdir -p /tmp/duk-core/src/duk_core \
    && printf '%s\n' '"""Build-time dependency scaffold; runtime source is bind-mounted."""' \
       > /tmp/duk-core/src/duk_core/__init__.py

RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip install "/tmp/duk-core[test,research]" \
    && rm -rf /tmp/duk-core

COPY environment/docker/ros/dev-start.sh /usr/local/bin/duk-dev-start
COPY environment/docker/ros/colcon-defaults.yaml /etc/duk-colcon-defaults.yaml
RUN chmod 0755 /usr/local/bin/duk-dev-start \
    && mkdir -p /workspace \
    && chown "${DEV_UID}:${DEV_GID}" /workspace

# ROS and a built DUK overlay are sourced for interactive Bash terminals.
RUN printf '%s\n' \
    'if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then source "/opt/ros/$ROS_DISTRO/setup.bash"; fi' \
    'if [ -f "/opt/duk-ros/install/setup.bash" ]; then source "/opt/duk-ros/install/setup.bash"; fi' \
    > /etc/duk-ros.bash \
    && printf '%s\n' 'source /etc/duk-ros.bash' >> /etc/bash.bashrc

WORKDIR /workspace
USER ${DEV_USER}

CMD ["duk-dev-start"]

# Opt-in desktop simulation. Keep the default development target lightweight.
FROM base AS simulation
USER root
RUN test "${ROS_DISTRO}" = jazzy \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
        ros-jazzy-ros-gz \
        ros-jazzy-robot-state-publisher \
        mesa-utils \
        xauth \
    && rm -rf /var/lib/apt/lists/*
# Ubuntu's legacy namespace bootstrap otherwise forces mpl_toolkits to its old
# system Matplotlib even when the modern DUK venv takes precedence on sys.path.
# PEP 420 namespace discovery works for both the system and venv interpreters.
RUN rm -f /usr/lib/python3/dist-packages/matplotlib-*-nspkg.pth \
          /usr/lib/python3/dist-packages/mpl_toolkits/__init__.py
USER duk

FROM base AS development
