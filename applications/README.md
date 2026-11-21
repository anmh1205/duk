# Applications

`applications/` owns user-facing and externally connected DUK software.

| Directory | Scope |
| --- | --- |
| `operator-dashboard/` | [Local simulation dashboard](operator-dashboard/README.md) |
| `gateway/` | Reserved for gateway and external integration |
| `mobile/` | Reserved for a mobile application |

The application includes the browser dashboard and a local Qt workbench for
NB1–NB6. The workbench runs the Python core and publishes synchronized
geometry and force replay to RViz.

[Application documentation](../docs/applications/README.md) · [Repository layout](../docs/architecture/repository-layout.md).
