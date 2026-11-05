# Environments

`environment/` owns development-image definitions, Compose extensions, platform declarations and environment-level dependency policy.

## Development model

The root [Compose file](../compose.yaml) and [Dev Container configuration](../.devcontainer/devcontainer.json) use the **same** development image. Dev Containers are an editor integration layer over that Compose service, not a second setup path.

The image contains:

- the selected ROS 2 `ros-base` environment;
- the ROS build tooling supplied by the official ROS image;
- a non-root `duk` developer account;
- an isolated Python virtual environment at `/opt/duk-venv`;
- the DUK core test/research Python dependency set.

The repository is bind-mounted at `/workspace`. `PYTHONPATH` points at `/workspace/robot/software/core/src`, so core source edits are immediately visible without running an editable install after the container starts.

ROS colcon outputs do not live in the source tree. Build/install/log data is stored in distro-scoped named volumes mounted under `/opt/duk-ros`, so switching ROS distributions cannot reuse an incompatible overlay and normal development does not dirty the repository.

## Entry points

### VS Code / Dev Containers

Clone the repository, open it in VS Code and select **Dev Containers: Reopen in Container**. The container is built from `compose.yaml`; no project `pip install` step is required.

### Docker Compose CLI

From the repository root:

```bash
docker compose up -d --build --wait
docker compose exec dev bash
```

The same image and Python/ROS environment are used. To stop it:

```bash
docker compose down
```

Named ROS build/install/log volumes are scoped by the Compose project name, which includes `ROS_DISTRO`. Jazzy and Humble therefore do not share ROS overlay artifacts.

## ROS distribution

Jazzy is the default:

```bash
docker compose up -d --build --wait
```

Select another configured distro through the environment, for example:

```bash
ROS_DISTRO=humble docker compose up -d --build --wait
```

For Dev Containers, put `ROS_DISTRO=humble` in a local `.env` and rebuild the container.

## Verified portability

CI exercises the same Compose image on native amd64 and arm64 for both Jazzy and Humble. Jazzy/amd64 is additionally exercised from an Ubuntu 26.04 host, and the VS Code Dev Container lifecycle is tested separately. See [Platform compatibility](../docs/platforms/compatibility.md) for the exact evidence boundary.

## What is not claimed

An opt-in Jazzy [VMware simulation preview](../docs/development/simulation-preview.md)
adds RViz/Gazebo and explicit X11/DRI or software-rendering settings. It is
separate from the base environment described above.

The base development image does not by itself establish GPU acceleration, Gazebo/RViz GUI forwarding, USB/CAN access, host-network DDS behavior, Jetson/JetPack compatibility or physical-robot access. Those require explicit platform/runtime configuration and validation.

[Development environment](../docs/development/environment.md) · [Platform compatibility](../docs/platforms/compatibility.md).
