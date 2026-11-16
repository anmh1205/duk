"""Enforce core dependency direction and the dependency-free geometry API."""

import ast
import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / "src/duk_core"
ALLOWED = {
    "modeling": {"modeling"},
    "estimation": {"estimation"},
    "control": {"control", "modeling"},
    "simulation": {"simulation", "modeling", "control", "estimation"},
    "visualization": {"visualization", "modeling"},
}


def _modules():
    result = {}
    for path in PACKAGE.rglob("*.py"):
        parts = path.relative_to(PACKAGE.parent).with_suffix("").parts
        name = ".".join(parts[:-1] if parts[-1] == "__init__" else parts)
        result[name] = path
    return result


def _imports(name, path):
    package = name if path.name == "__init__.py" else name.rsplit(".", 1)[0]
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            prefix = node.module or ""
            if node.level:
                parents = package.split(".")
                base = parents[: len(parents) - node.level + 1]
                prefix = ".".join(base + ([prefix] if prefix else []))
            yield prefix
            yield from (f"{prefix}.{alias.name}" for alias in node.names)


def test_layer_dependencies_and_absence_of_import_cycles():
    modules = _modules()
    graph = {name: set() for name in modules}
    for name, path in modules.items():
        owner = name.split(".")[1] if "." in name else None
        for target in _imports(name, path):
            if target.startswith("duk_core."):
                dependency = target.split(".")[1]
                assert owner in ALLOWED and dependency in ALLOWED[owner], (
                    f"Forbidden layer dependency: {name} -> {target}"
                )
                if target in modules and target != name:
                    graph[name].add(target)
            external = target.split(".")[0]
            assert external not in {
                "rclpy",
                "launch",
                "launch_ros",
                "tf2_ros",
                "duk_description",
            } and not external.endswith("_msgs"), f"ROS dependency in core: {name}"
            if owner in {"modeling", "control", "estimation"}:
                assert external not in {
                    "matplotlib",
                    "IPython",
                    "ipywidgets",
                    "nbclient",
                }, f"Presentation dependency in {name}: {target}"

    visited = set()

    def visit(name, ancestors):
        assert name not in ancestors, "Import cycle: " + " -> ".join((*ancestors, name))
        if name in visited:
            return
        for dependency in sorted(graph[name]):
            visit(dependency, (*ancestors, name))
        visited.add(name)

    for name in sorted(graph):
        visit(name, ())


def test_geometry_imports_without_site_packages():
    # -I -S excludes installed optional packages and user environment paths.
    code = (
        "import sys; sys.path.insert(0, sys.argv[1]); "
        "from duk_core.modeling import load_leg_geometry_json, solve_leg_pose; "
        "from duk_core.modeling.kinematic_bench import KinematicBench"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-S", "-c", code, str(PACKAGE.parent)],
        capture_output=True,
        text=True,
        check=False,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr
