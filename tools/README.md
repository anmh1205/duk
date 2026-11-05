# Engineering Tools

`tools/` owns maintained engineering utilities. Ad hoc tasks belong in [scripts](../scripts/README.md).

## Current tools

| Path | Purpose |
| --- | --- |
| `environment-check/verify-dev-container.sh` | Verify the built DUK development container, Python research stack, mounted core source and optional full model/test smoke suite |
| `dev/duk-dev.py` | Configure, launch and diagnose the opt-in Linux/VMware Gazebo/RViz preview; [workflow](../docs/development/simulation-preview.md) |
| `environment-check/verify-simulation-container.sh` | Wait for simulation-container volume initialization and verify the installed environment |
| `environment-check/verify-simulation-preview.py` | Check the running preview's robot description, static TF and advancing simulation clock |
| `model-generation/` | Reserved for maintained model-generation utilities |
| `validation/` | Reserved for reusable engineering-validation utilities |

[Repository layout](../docs/architecture/repository-layout.md).
