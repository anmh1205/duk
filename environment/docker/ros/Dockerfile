ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Isolated Python tools; ROS system packages remain visible.
RUN python3 -m venv --system-site-packages /opt/duk-venv
ENV VIRTUAL_ENV=/opt/duk-venv
ENV PATH="/opt/duk-venv/bin:${PATH}"

# Enable ROS in interactive shells; source DUK overlay only after it exists.
RUN printf '%s\n' \
    'source /opt/ros/${ROS_DISTRO}/setup.bash' \
    'if [ -f /workspace/robot/software/ros2_ws/install/setup.bash ]; then source /workspace/robot/software/ros2_ws/install/setup.bash; fi' \
    >> /etc/bash.bashrc

WORKDIR /workspace
CMD ["sleep", "infinity"]
