# Compose extensions

`compose.yaml` at the repository root is the single entry point.
When functional services are implemented, place their configurations here:
for example simulation, application, hardware and Jetson-specific overrides.
Integrate them deliberately through Compose include or merge semantics.

The baseline `dev` service contains neither GUI pass-through nor privileged
hardware access. Do not add empty services for unimplemented features.
