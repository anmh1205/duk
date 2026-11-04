# Getting Started

The supported development workflow is container-first. A host does not need the DUK Python research stack or ROS installed locally.

## Host prerequisites

For the VS Code workflow:

- Git;
- Docker Engine;
- the Docker Compose plugin; Compose 5.7.0 or newer is recommended for Dev Container builds when using Buildx 0.37.2 or newer;
- Visual Studio Code;
- the Dev Containers extension.

For the terminal-only workflow, VS Code and the extension are not required. Older Compose releases can still run the normal Compose workflow; if Dev Containers hit the Buildx temporary-Dockerfile filesystem-entitlement regression, use the bounded workaround documented in [Development Environment](environment.md) until Compose can be upgraded.

## Clone

```bash
git clone https://github.com/anmh1205/duk.git
cd duk
```

No host-side Python virtual environment or project `pip install` is required.

## Option A — Dev Container

Open the repository in VS Code and run **Dev Containers: Reopen in Container**.

The Dev Container uses the root Compose service, connects as the non-root `duk` user and selects `/opt/duk-venv/bin/python` as the Python interpreter.

Inside the container, verify the environment with:

```bash
bash tools/environment-check/verify-dev-container.sh --full
```

## Option B — Docker Compose

```bash
docker compose up -d --build --wait
docker compose exec dev bash
```

The terminal should already have the DUK Python dependencies available. Verify:

```bash
bash tools/environment-check/verify-dev-container.sh --full
```

No `pip install -e` command is part of the normal container startup workflow.

Stop the service with:

```bash
docker compose down
```

Use `docker compose down --volumes` only when intentionally discarding the persisted ROS build/install/log volumes for that ROS distro.

## Change ROS distribution

Jazzy is the default. For a Humble Compose environment:

```bash
ROS_DISTRO=humble docker compose up -d --build --wait
docker compose exec dev bash
```

For a Dev Container, set a local `.env`:

```text
ROS_DISTRO=humble
```

then use **Dev Containers: Rebuild Container**.

Because the Compose project name includes the ROS distro, Jazzy and Humble use separate named ROS workspace volumes.

## Core model smoke test

Inside either container workflow:

```bash
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75 --branch 1
python -m pytest -q robot/software/core/tests
```

The branch argument in this example selects a mathematical assembly branch for inspection; the physical CAD branch remains a separate item to verify.
