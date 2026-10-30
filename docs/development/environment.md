# Development Environment

DUK uses a single repository as the development surface for the robot's mechanical and electronic assets, numerical models, ROS integration, simulation, research and applications. Individual disciplines retain their native toolchains; the common container environment primarily standardizes the software and ROS workflow.

## Host prerequisites

For the container workflow, install a supported Docker Engine and Docker Compose environment and optionally an IDE supporting Dev Containers. Docker cannot standardize all host-device behavior: GUI forwarding, CAN/USB access, GPU acceleration and real-time scheduling require target-specific configuration and verification.

CAD and PCB design work may depend on native desktop applications and vendor toolchains. Store editable source projects under `robot/hardware/` regardless of which workstation generated them.

## ROS 2 development container

The root `compose.yaml` provides the baseline `dev` service; `.devcontainer/devcontainer.json` connects an IDE to that service. The ROS distribution is selected through `.env` and must correspond to an available container image.

```bash
cp .env.example .env
docker compose up -d dev
docker compose exec dev bash
```

Inside the container, use `ros2`, `colcon` and Python as standard shell commands. The interactive shell sources the selected ROS installation and, if present, the built workspace overlay. The workspace becomes usable once actual ROS packages have been created and built.

Switching ROS distributions requires rebuilding the image and its compiled workspace. Do not reuse a build/install cache between incompatible ROS distributions or Python/C++ binary interfaces.

## Python numerical workflow

The robotics core is installed as a normal Python package. From the repository root:

```bash
python -m pip install -e "./robot/software/core[research,test]"
python -m pytest robot/software/core/tests
```

Install the research extras only when notebooks or visualization are needed. These Python tools do not require starting ROS or Gazebo.

## Simulation and hardware targets

Simulation infrastructure belongs in `robot/simulation/` and ROS bridge code in `duk_gazebo/`. Graphics/GPU forwarding and simulator dependencies must be added and tested against the chosen platform before use.

Jetson deployment needs a configuration compatible with the board's JetPack/L4T stack and architecture, including access to hardware drivers. The standard development image does not establish NVIDIA runtime compatibility or timing behavior. Keep development convenience separate from the safety and performance requirements of a deployed robot.

## Troubleshooting boundaries

- **Import errors:** Confirm that the expected Python environment has the `duk-core` package installed.
- **ROS commands missing:** Confirm that the container image and active shell provide the selected ROS installation.
- **No ROS packages found:** Check that packages contain valid manifests and were built in the workspace.
- **Simulator GUI or device access unavailable:** Check host, container permissions, drivers, display/graphics setup and platform-specific overrides.
- **Incorrect robot behavior:** Verify model revision, units, frames, controller parameters and calibration before adjusting toolchain settings.

Platform support is established only by target-specific evidence recorded in the [compatibility matrix](../platforms/compatibility.md).
