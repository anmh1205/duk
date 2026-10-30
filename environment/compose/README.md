# Compose Configuration

The repository-root `compose.yaml` is the canonical entry point for development services. Compose extensions in this directory should be introduced only when a concrete simulation, application or target-specific workflow requires them.

Each extension must declare its purpose, host requirements, service dependencies, network assumptions, storage behavior and device privileges. Prefer explicit opt-in for hardware-facing services and preserve compatibility with the standard Dev Container workflow.

Avoid duplicating entire service definitions across ROS distribution variants when a clear shared base and controlled override can express the differences. Verify the merged Compose configuration and executed runtime behavior for every supported target.

See [Development environment](../../docs/development/environment.md) and [Platform compatibility](../../docs/platforms/compatibility.md).
