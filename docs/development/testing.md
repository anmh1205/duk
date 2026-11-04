# Testing

DUK currently has two complementary automated checks.

## Python core

`.github/workflows/python-core.yml` installs `duk-core[test]` directly on the GitHub runner and verifies:

- package unit tests;
- the standalone DUK leg-kinematics example;
- Python source compilation.

This keeps the numerical package independently testable without ROS or Docker.

## Development environment

`.github/workflows/environment.yml` validates the actual development-container packaging. For each configured ROS distribution it:

1. resolves `docker compose config`;
2. builds the development image;
3. starts the Compose service and waits for its healthcheck;
4. runs `tools/environment-check/verify-dev-container.sh --full` inside the service;
5. tears down containers and named test volumes.

The full environment verification checks the Python research stack, confirms that `duk_core` is imported from the mounted workspace, runs the model example and executes the core unit tests.

A green Python-core job does not prove the Docker image is valid; a green environment job does. Neither proves GPU, Jetson, hardware I/O, simulation GUI or physical-robot compatibility.
