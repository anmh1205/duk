# Development Environment

## Contract

DUK uses one containerized development environment with two supported front ends:

1. VS Code Dev Containers;
2. Docker Compose CLI.

Both consume the same `compose.yaml` service and the same Dockerfile. Editor lifecycle hooks must not be required to install project dependencies.

## Image composition

The development Dockerfile is `environment/docker/ros/Dockerfile`.

The base image is the official ROS `${ROS_DISTRO}-ros-base` image. The official ROS image owns the ROS/Ubuntu pairing for the selected distro. DUK adds only project-specific development requirements.

Python research/test dependencies are installed into `/opt/duk-venv` while the image is built. NumPy is intentionally constrained to `>=1.26,<2` while the environment supports ROS distributions whose apt-provided binary Python extensions may still target the NumPy 1.x ABI. The package source copied during image build exists only to resolve/install the package dependency set. At runtime the bind-mounted workspace source takes precedence through:

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

The source repository is bind-mounted at `/workspace`. ROS build artifacts are intentionally kept outside that source tree. The image sets `COLCON_DEFAULTS_FILE=/etc/duk-colcon-defaults.yaml`, so a normal `colcon build` writes to `/opt/duk-ros` without adding build/install/log directories to the repository.

ROS workspace output is separated into named volumes:

- `ros-build`;
- `ros-install`;
- `ros-log`.

The named volumes are mounted at `/opt/duk-ros/build`, `/opt/duk-ros/install` and `/opt/duk-ros/log`. The Compose project name is `duk-${ROS_DISTRO}`, so these volumes are distro-specific.

Container-local manual package changes are ephemeral and should not be relied upon.

## ROS shell environment

Interactive Bash shells source the selected ROS installation and source the DUK ROS overlay when `/opt/duk-ros/install/setup.bash` exists.

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


## Current Dev Container toolchain caveat

Docker Buildx 0.37.2 began enforcing filesystem read entitlements for Compose/Bake. Current Dev Container CLI builds can generate a temporary `Dockerfile-with-features` outside the repository build context; affected Docker Compose versions can therefore fail before the DUK Dockerfile itself is evaluated, with an error requesting `--allow=fs.read=.../Dockerfile-with-features`.

This is an upstream Docker Compose/Buildx interoperability issue, not a DUK container-layout requirement. DUK CI uses `BUILDX_BAKE_ENTITLEMENTS_FS=0` **only for the Dev Container lifecycle job** so the repository configuration can still be exercised while affected runner versions are present. The normal Compose jobs do not use this override.

For a local workstation, prefer updating Docker Compose to a release containing the upstream fix. If an affected toolchain must be used temporarily, launch VS Code from a shell with:

```bash
BUILDX_BAKE_ENTITLEMENTS_FS=0 code .
```

This disables Buildx filesystem entitlement checking for builds started by that process, so it should be treated as a temporary compatibility workaround rather than persisted globally. Remove it once the local Docker Compose version includes the upstream fix.
