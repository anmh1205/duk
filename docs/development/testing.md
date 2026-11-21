# Testing

DUK uses independent core tests and end-to-end development-environment tests. Both are required because a Python package can pass in isolation while the ROS/Docker environment is still broken.

## Python source and numerical studies

The [Python workflow](../../.github/workflows/python-core.yml) installs
`duk-core[test,research,dev]` directly on an Ubuntu 24.04 runner and verifies:

- Ruff lint/import rules and formatting for Python and notebook sources;
- core unit tests, import-layer/cycle checks and dependency-free geometry imports;
- ROS-independent URDF/SDF geometry adapters and developer-tool configuration tests;
- local dashboard HTTP inputs, canonical poses, NB4 replay parity, and invalid combinations;
- workbench NB6 per-wheel grip, direct core parity, supervisor faults, NB1–NB5
  study coverage, and saved run round trips;
- the standalone DUK leg-kinematics example;
- symbolic geometry agreement, Lagrange identities, inverse kinematics and
  second-order derivatives; synthetic test parameters are not robot data;
- the standalone symbolic model with every physical template value still unknown;
- Python source compilation;
- all seven source notebooks in fresh Jupyter kernels, covering NB1–NB6.

Source-notebook tests also keep execution counts and outputs out of the committed
files. Ruff and fresh-kernel execution cover notebook syntax; unit tests do not
duplicate compilation or search notebook source for plotting-function names.
Geometry invariants are checked across the workspace rather than repeating a
single-pose test or hard-coding configuration dictionaries.
The runner checks saved Output-widget state for captured callback errors,
which otherwise need not fail a notebook cell. Presentation regressions verify
static initial figures, cached-case updates and timelines that end at the actual
stop instant. The numerical regressions cover off-grid impulse timing and invalid
sensor replay without partially updating the observer.
Planar CoM diagnostics are checked against the second derivative of the composed
CoM position, gravity-only flight and vertical contact-force balance. Boundary
cases exercise both hips near both geometric limits. Held-command plots are
checked for the correct integral over a partial last interval and for an
instantaneous stop without invented post-stop duration.
Controller regressions compare time-varying multi-input LQR and scalar-input MPC
against a full quadratic objective recovered by evaluation, including motor and
force bounds, empty force rows, zero state cost and malformed problem data.
Contact regressions check unilateral support, friction-cone bounds and passive
tangential power over batched 1D/2D inputs, and reject malformed rigid force inputs.
Real two-kernel runner tests cover stdout-only execution, saved execution and a
failure in the second kernel. They check source preservation, the original error,
embedded provenance and archived source hashes. NB6 reference derivatives are
compared with the admitted height path during entry, exit and capacity reduction,
away from the piecewise-linear ramp breakpoints.
NB3 includes a high-grip replay in the detector's negative population and reports
class counts. NB5 fails execution if a baseline case violates its declared
completion/lean criterion; intentionally unsuccessful NB6 stress cases still
report their separate outcomes.
The symmetric upper CoM angle derivatives are compared with composed mass
properties, then exercised near both branch boundaries. A one-interval height
study checks that a valid near-boundary posture does not fail because of an
outside derivative probe. The shared angle primitive also recovers the exact
speed and acceleration of prescribed rotating directions.
Friction-observer regressions rotate the same compliant contact through axis and
diagonal slip directions. The configured reserve must give the same inferred
coefficient in each direction. NB6 assertions also check the friction input
change and preserve the initial belief after later updates in the recorded trace.

The workflow runs for pull requests and relevant changes pushed to `main`. This keeps the numerical package independently testable without ROS or Docker.

From the repository root, run:

```bash
python -m pip install -e './robot/software/core[test,research,dev]'
ruff check .
ruff format --check .
OPENBLAS_NUM_THREADS=1 python -m pytest -q -m 'not ros'
OPENBLAS_NUM_THREADS=1 python robot/software/core/examples/run_duk_notebooks.py --output-dir data/results/duk-v3
```

The root pytest configuration supplies the core and adapter source paths. The
`ros` marker identifies the Qt/ROS panel test; run it in the simulation container
with `python -m pytest -q -m ros`. The simulation workflow's adapter test command
runs both geometry and panel tests. A locally deselected ROS test does not count
as a successful runtime check.

For the architecture refactor, six spatial scenarios were replayed before and
after changing the module structure. All 168 recorded channels matched exactly,
including injected faults and stop times; CPU timing was excluded. The usual
notebook assertions and physical-identity regressions remain the behavioral
checks for subsequent changes.

## Development environment

`.github/workflows/environment.yml` validates the actual container packaging.

The primary Compose matrix exercises:

| ROS 2 | Architecture | Runner |
| --- | --- | --- |
| Jazzy | amd64 | Ubuntu 24.04 |
| Humble | amd64 | Ubuntu 24.04 |
| Jazzy | arm64 | Ubuntu 24.04 ARM |
| Humble | arm64 | Ubuntu 24.04 ARM |

Each matrix job:

1. validates the resolved Compose configuration;
2. builds the development image natively on the target architecture;
3. starts the service and waits for the lightweight healthcheck;
4. runs `tools/environment-check/verify-dev-container.sh --full`;
5. tears down the service and test volumes.

Full verification checks the non-root developer identity, ROS and Python toolchain, workspace-source import path, NumPy/SciPy/SymPy/Matplotlib and `rclpy` imports, `pip check`, writable external colcon volumes, `colcon build`, the DUK model example and core unit tests.

The workflow also validates:

- the Jazzy/amd64 Dev Container lifecycle with the Dev Container CLI, including post-create commands and the full verifier;
- the Jazzy/amd64 Compose environment from an Ubuntu 26.04 host;
- shell syntax and Dev Container JSON before expensive image builds.

CI concurrency cancels superseded runs for the same ref so stale environment builds do not consume runners or obscure the current result.

The environment workflow also runs weekly on the default branch. This is intentional: ROS base-image tags and allowed Python dependency ranges can change outside the repository, so scheduled verification detects upstream drift even when no DUK commit has changed.

## Evidence boundaries

The separate [simulation workflow](../../.github/workflows/simulation.yml) builds
the opt-in Jazzy image, checks geometry and exercises the static preview's ROS
description, TF and advancing clock without a GUI. It does not test VMware
graphics, accelerated rendering or real robot dynamics. Local GUI observations
are recorded separately in [Platform compatibility](../platforms/compatibility.md).

A green Python-core job does not prove the Docker image is valid. A green development-environment workflow establishes only the combinations explicitly exercised above.

It does **not** establish NVIDIA GPU acceleration, Jetson/JetPack/L4T, simulation GUI forwarding, USB/serial/CAN passthrough, external ROS 2 DDS networking or physical-robot access. Those require separate runtime definitions and tests.

The simulation workflow also drives the [Qt workbench](../../applications/operator-dashboard/README.md)
from its Run button and time slider, then compares ROS marker origins and
terminal wheel transforms with the saved workbench samples. Run it inside the
Jazzy simulation image after sourcing ROS.

```bash
QT_QPA_PLATFORM=offscreen OPENBLAS_NUM_THREADS=1 \
  python -m pytest -q -m ros tests/integration/test_simulation_workbench.py
```
