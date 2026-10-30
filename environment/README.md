# Environments

`environment/` groups environment definitions and target-specific development assets.

| Directory | Scope |
| --- | --- |
| `docker/` | Container image definitions |
| `compose/` | Compose extensions |
| `platforms/` | Platform declarations |
| `dependencies/` | Environment-level dependencies |

The root [Compose file](../compose.yaml) and [Dev Container configuration](../.devcontainer/devcontainer.json) are the current configuration sources. Their presence does not establish compatibility with every host or robot platform.

[Environment documentation](../docs/development/environment.md) · [Platform documentation](../docs/platforms/README.md).
