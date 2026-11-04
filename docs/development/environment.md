# Development Environment

## Contract

DUK uses one containerized development environment with two supported front ends:

1. VS Code Dev Containers;
2. Docker Compose CLI.

Both consume the same `compose.yaml` service and the same Dockerfile. Editor lifecycle hooks must not be required to install project dependencies.

## Image composition

The development Dockerfile is `environment/docker/ros/Dockerfile`.

The base image is the official ROS `${ROS_DISTRO}-ros-base` image. The official ROS image owns the ROS/Ubuntu pairing for the selected distro. DUK adds only project-specific development requirements.

Python research/test dependencies are installed into `/opt/duk-venv` while the image is built. The package source copied during image build exists only to resolve/install the package dependency set. At runtime the bind-mounted workspace source takes precedence through:

```text
PYTHONPATH=/workspace/robot/software/core/src
```

This separates dependency-image rebuilds from ordinary source editing:

```text
pyproject/development image change -> rebuild image
ordinary Python source change       -> immediately visible
```

## User and file ownership

The image runs as the non-root `duk` account. Its default build UID/GID is `1000:1000`, matching the normal first-user identity on Ubuntu workstations.

Dev Containers declare `remoteUser=duk` and enable Linux UID/GID remapping. Compose CLI users whose host identity is not `1000:1000` can override `DEV_UID` and `DEV_GID` through a local `.env` before rebuilding the image.

The developer account has passwordless sudo for interactive development tasks. Persistent tool/dependency changes still belong in the Dockerfile rather than in manual container state.

## Persistent and ephemeral state

The source repository is bind-mounted at `/workspace`.

ROS workspace output is separated into named volumes:

- `ros-build`;
- `ros-install`;
- `ros-log`.

The Compose project name is `duk-${ROS_DISTRO}`, so these named volumes are distro-specific.

Container-local manual package changes are ephemeral and should not be relied upon.

## ROS shell environment

Interactive Bash shells source the selected ROS installation and source the DUK ROS overlay when `robot/software/ros2_ws/install/setup.bash` exists.

The core numerical package remains ROS-independent.

## Environment verification

`tools/environment-check/verify-dev-container.sh` checks:

- the selected ROS installation;
- the DUK virtual environment;
- required developer commands;
- NumPy/SciPy/SymPy/Matplotlib availability;
- that `duk_core` resolves from the mounted workspace rather than a stale image copy.

Its full mode additionally runs the DUK linkage example and package tests.

The `Development environment` GitHub Actions workflow builds and exercises the Compose environment for the ROS distributions declared by that workflow.

## Deliberately separate future concerns

GPU/NVIDIA runtime configuration, Jetson/JetPack images, GUI forwarding, direct USB/CAN access and ROS 2 communication with external physical networks are not implicit in the base image. Add them as explicit platform or Compose extensions when their requirements are established.
