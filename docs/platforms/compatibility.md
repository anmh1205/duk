# Platform Compatibility

This page records **verified development-environment combinations**, not broad platform promises. The authoritative executable check is [`.github/workflows/environment.yml`](../../.github/workflows/environment.yml); the machine-readable summary is [`environment/platforms/compatibility.yaml`](../../environment/platforms/compatibility.yaml).

## CI-verified development image

| Architecture | ROS 2 | Host used for verification | Status |
| --- | --- | --- | --- |
| amd64 | Jazzy | Ubuntu 24.04 | CI verified |
| amd64 | Humble | Ubuntu 24.04 | CI verified |
| arm64 | Jazzy | Ubuntu 24.04 ARM | CI verified |
| arm64 | Humble | Ubuntu 24.04 ARM | CI verified |
| amd64 | Jazzy | Ubuntu 26.04 | CI verified |

The four ROS/architecture matrix jobs build the Compose image natively on the corresponding GitHub-hosted CPU architecture, start it, wait for health, verify the non-root user and Python/ROS environment, run `colcon build`, execute the DUK core model example and run the core unit tests.

The Ubuntu 26.04 job additionally verifies that the same Jazzy Compose development environment can be built and exercised from an Ubuntu 26.04 amd64 host. This validates the container boundary against that host release; it does not mean the ROS distribution itself is installed natively on Ubuntu 26.04.

## Dev Container

The Jazzy/amd64 Dev Container lifecycle is also exercised with the Dev Container CLI: create the container, run lifecycle commands and run the full environment verifier.

Docker Compose 5.7.0 includes the upstream fix for the Buildx filesystem-entitlement regression that affected temporary Dev Container Dockerfiles outside the repository build context. The CI job detects the installed Compose version and enables the compatibility override only on older affected runners. See [Development environment](../development/environment.md) for the exact boundary and fallback.

## What ARM64 verification does and does not mean

Native generic Linux ARM64 is verified for both Jazzy and Humble. This is useful evidence for portability of the DUK development image and Python/modeling stack.

It does **not** establish Jetson compatibility. NVIDIA Jetson adds JetPack/L4T, GPU runtime, CUDA, device libraries, hardware access and potentially vendor-specific ROS/simulation constraints. `jetson-arm64` therefore remains **unconfigured** until a Jetson-specific image/runtime contract and physical validation are added.

## Local VMware simulation smoke test (2026-11-05)

The opt-in [simulation preview](../development/simulation-preview.md) was
manually exercised on Ubuntu 24.04 amd64 inside VMware, with an i7-6700K exposed
to the guest, two online logical CPUs and about 16 GiB RAM. This is separate from
the CI matrix and is not a guarantee for other VMware hosts.

Observed endpoints:

- Gazebo Sim 8.15.0 (Harmonic) and RViz2 opened the same two-leg static model;
- both leg assemblies were generated from the canonical geometry, including
  150 mm wheel diameter; regression checks cover metric conversion, frame
  mapping, rigid-carrier dimensions and generated visual endpoints;
- ROS received the robot description, world-to-base static TF and advancing
  Gazebo simulation clock;
- the server-only launch also passed that integration check; five short
  real-time-factor samples were approximately 0.83–1.00 for this static world;
- the simulation Dev Container CLI lifecycle and its post-create checks passed,
  including initialization with the generated local Compose override absent;
- DRI access and explicit software rendering were exercised. OpenGL reported
  Mesa llvmpipe / OpenGL 4.5, **not hardware acceleration**, on this guest;
- with both GUI windows open, RViz displayed about 19 FPS at the configured
  20 FPS limit. Short Gazebo real-time-factor samples varied approximately
  0.34–1.10 under concurrent desktop load. These are smoke-test observations,
  not a sustained benchmark or evidence for real-time robot dynamics.

The robot is static and has no assigned mass/inertia, controller or sensors.
This does not validate balance, closed-loop linkage dynamics, contact behavior
or a moving robot's performance. The base development container and its running
notebook session were left separate.

### Local moving RViz bench (2026-11-06)

The [fixed-body kinematic bench](../development/kinematic-bench.md) was exercised
on the same Ubuntu 24.04 VMware guest with explicit software rendering. The
rebuilt simulation image includes the Qt binding explicitly. Independent leg
controls, auto/pause/speed, three live plots and actual TF closure at C were
checked. The local test set passed 48 core/ROS tests and four configuration tests,
including a headless Qt widget test. RViz showed about 19 FPS in one live GUI
observation; this is not a sustained performance benchmark. The ROS integration
script passed both without windows and with the real panel/RViz running.

No mass/inertia, motor calibration, controller or contact law was added by this
bench. Its spanning-tree URDF is a visualization representation, not a closed-loop
dynamics model. These observations do not extend GPU or physical-platform support.

## Not yet established

The base development environment does not currently claim validated support for:

- NVIDIA GPU acceleration;
- Jetson/JetPack/L4T;
- Gazebo or RViz desktop GUI forwarding;
- USB, serial or CAN device passthrough;
- ROS 2 DDS communication between the container and external robot/network interfaces;
- physical actuator access;
- macOS or Windows hosts.

Those capabilities should be introduced as explicit platform/runtime extensions and given their own verification rather than being inferred from the generic development image.
