# Robotics Core

`robot/software/core/` is the Python package area for ROS-independent numerical models, control computations, visualization and interactive notebooks.

| Directory | Scope |
| --- | --- |
| `src/duk_core/modeling/` | Mathematical models |
| `src/duk_core/control/` | Control algorithms |
| `src/duk_core/visualization/` | Reusable drawing and plotting utilities |
| `notebooks/` | Interactive exploration |
| `examples/` | Usage examples |
| `tests/` | Package-level tests |

The existing generic planar-chain example is not a validated model of DUK. For current implemented behavior, inspect the [source](src/duk_core/modeling/planar.py), [example](examples/planar_chain.py), [tests](tests/test_planar.py) and [package manifest](pyproject.toml).

[Core documentation](../../../docs/software/core.md) · [Repository layout](../../../docs/architecture/repository-layout.md).
