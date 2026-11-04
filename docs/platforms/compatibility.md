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

A temporary **CI-only** Buildx filesystem-entitlement compatibility override is currently required on the Ubuntu 24.04 GitHub runner because its Compose/Buildx combination is affected by an upstream Docker regression when Dev Container CLI generates a temporary Dockerfile outside the repository build context. See [Development environment](../development/environment.md) for the bounded workaround.

## What ARM64 verification does and does not mean

Native generic Linux ARM64 is verified for both Jazzy and Humble. This is useful evidence for portability of the DUK development image and Python/modeling stack.

It does **not** establish Jetson compatibility. NVIDIA Jetson adds JetPack/L4T, GPU runtime, CUDA, device libraries, hardware access and potentially vendor-specific ROS/simulation constraints. `jetson-arm64` therefore remains **unconfigured** until a Jetson-specific image/runtime contract and physical validation are added.

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
