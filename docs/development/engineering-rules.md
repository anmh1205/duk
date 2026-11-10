# Engineering Rules

These conventions apply to the implemented Python library, notebooks, ROS
adapters and development tools. They are enforced by the current source checks;
they do not establish a robot runtime or hardware interface.

## Ownership and dependencies

Follow the [repository layout](../architecture/repository-layout.md) and the
[core dependency boundaries](../software/core.md). Put equations in `modeling`,
feedback/allocation in `control`, sensor algorithms in `estimation` and experiment
orchestration in `simulation`. Notebooks compose those functions. ROS and GUI
lifecycle code stays in its adapter.

Keep one implementation of each equation or policy. Import explicit public
functions from their owning module. Underscore modules/functions are private;
production callers in another package should not depend on them. Keep import-time
work small and preserve the geometry API without optional research dependencies.

Use small functions or cohesive state objects when a routine combines distinct
responsibilities. Mathematical derivations may remain sequential when splitting
them would obscure the equations. Use named result fields for data passed between
stages, and document units, coordinate order, assumptions and failure outcomes at
public boundaries. Add type annotations when introducing or changing public APIs.

## Python and notebooks

Python code targets the package's declared minimum version, currently 3.10.
[Ruff configuration](../../ruff.toml) sets one import, lint and formatting policy
for Python files and committed notebook code cells. The `dev` extra pins the Ruff
version used in CI. Tests may delay scientific imports until after
`pytest.importorskip`; that exception is documented in the Ruff configuration.

From the repository root:

```bash
python -m pip install -e './robot/software/core[test,research,dev]'
ruff check .
ruff format --check .
```

Use `ruff check --fix .` and `ruff format .` to apply supported mechanical changes,
then review the diff. Source notebooks stay free of generated execution output;
write executed copies under `data/results/` or an external temporary directory.

## Verification and evidence

Run focused regressions during implementation, then the relevant package and
integration checks before committing. Use shared pytest fixtures instead of
importing another test module. Preserve existing behavioral assertions during
refactoring. Numerical changes need independent physical identities or measured
reference cases; a second transcription of the implementation is insufficient.

The root [pytest configuration](../../pytest.ini) collects core, adapter and tool
tests. `python -m pytest -q -m 'not ros'` runs the suite available without ROS.
The `ros` marker identifies the Qt/ROS panel integration test; the simulation
workflow runs it in the configured container. Report skipped or unavailable
runtime checks explicitly.

Keep synthetic study inputs separate from physical measurements. Document the
conditions of numerical results and stop cases. Real hardware operation, flashing
and deployment require explicit authorization under the repository's agent rules.

[Testing commands and CI scope](testing.md).
