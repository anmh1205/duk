# Testing

DUK uses independent core tests and end-to-end development-environment tests. Both are required because a Python package can pass in isolation while the ROS/Docker environment is still broken.

## Python core

`.github/workflows/python-core.yml` installs `duk-core[test,symbolic]` directly on an Ubuntu 24.04 runner and verifies:

- package unit tests;
- the standalone DUK leg-kinematics example;
- symbolic geometry agreement, Lagrange identities, inverse kinematics and
  second-order derivatives; synthetic test parameters are not robot data;
- the standalone symbolic model with every physical template value still unknown;
- Python source compilation.

The workflow runs for pull requests and relevant changes pushed to `main`. This keeps the numerical package independently testable without ROS or Docker.

## Development environment

`.github/workflows/environment.yml` validates the actual container packaging.

The primary Compose matrix exercises:

| ROS 2 | Architecture | Runner |
| --- | --- | --- |
| Jazzy | amd64 | Ubuntu 24.04 |
| Humble | amd64 | Ubuntu 24.04 |
| Jazzy | arm64 | Ubuntu 24.04 ARM |
| Humble | arm64 | Ubuntu 24.04 ARM |

Each matrix job:

1. validates the resolved Compose configuration;
2. builds the development image natively on the target architecture;
3. starts the service and waits for the lightweight healthcheck;
4. runs `tools/environment-check/verify-dev-container.sh --full`;
5. tears down the service and test volumes.

Full verification checks the non-root developer identity, ROS and Python toolchain, workspace-source import path, NumPy/SciPy/SymPy/Matplotlib and `rclpy` imports, `pip check`, writable external colcon volumes, `colcon build`, the DUK model example and core unit tests.

The workflow also validates:

- the Jazzy/amd64 Dev Container lifecycle with the Dev Container CLI, including post-create commands and the full verifier;
- the Jazzy/amd64 Compose environment from an Ubuntu 26.04 host;
- shell syntax and Dev Container JSON before expensive image builds.

CI concurrency cancels superseded runs for the same ref so stale environment builds do not consume runners or obscure the current result.

The environment workflow also runs weekly on the default branch. This is intentional: ROS base-image tags and allowed Python dependency ranges can change outside the repository, so scheduled verification detects upstream drift even when no DUK commit has changed.

## Evidence boundaries

The separate [simulation workflow](../../.github/workflows/simulation.yml) builds
the opt-in Jazzy image, checks geometry and exercises the static preview's ROS
description, TF and advancing clock without a GUI. It does not test VMware
graphics, accelerated rendering or real robot dynamics. Local GUI observations
are recorded separately in [Platform compatibility](../platforms/compatibility.md).

A green Python-core job does not prove the Docker image is valid. A green development-environment workflow establishes only the combinations explicitly exercised above.

It does **not** establish NVIDIA GPU acceleration, Jetson/JetPack/L4T, simulation GUI forwarding, USB/serial/CAN passthrough, external ROS 2 DDS networking or physical-robot access. Those require separate runtime definitions and tests.
