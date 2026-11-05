# Compose Extensions

The base entry point is [compose.yaml](../../compose.yaml).

[simulation.yaml](simulation.yaml) selects the opt-in Jazzy simulation target on
the same `dev` service, with a separate Compose project and volumes. The host
helper prepares a local override for X11 authentication and optional DRI device
access. See the [VMware preview workflow](../../docs/development/simulation-preview.md).

[Environment README](../README.md).
