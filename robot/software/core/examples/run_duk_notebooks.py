"""Execute the DUK v3 notebook sequence in fresh Jupyter kernels."""

import hashlib
import json
import os
import platform
import subprocess
import tarfile
from argparse import ArgumentParser
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import nbformat
from nbclient import NotebookClient

NOTEBOOKS = (
    "kinematics/duk-leg-kinematics.ipynb",
    "motion/duk-leg-motion.ipynb",
    "dynamics/duk-hip-load.ipynb",
    "dynamics/duk-planar-contact.ipynb",
    "control/duk-fixed-hip-balance.ipynb",
    "control/duk-height-balance.ipynb",
    "control/duk-whole-body.ipynb",
)


def run_manifest(core: Path, selected: tuple[str, ...]) -> tuple[dict, Path]:
    """Identify the actual source and environment, including uncommitted files."""
    checkout = subprocess.run(
        ["git", "-C", str(core), "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        check=False,
    )
    repository = Path(checkout.stdout.strip()) if checkout.returncode == 0 else core
    files = {
        path
        for folder, suffix in (
            ("src", "*.py"),
            ("examples", "*.py"),
            ("notebooks", "*.ipynb"),
        )
        for path in (core / folder).rglob(suffix)
    }
    files.update(
        path
        for path in (
            core / "pyproject.toml",
            repository / "robot/config/physical/duk_leg_geometry.json",
            repository / "research/experiments/duk-v3/synthetic.json",
        )
        if path.is_file()
    )
    hashes = {
        str(path.relative_to(repository)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(files)
    }
    packages = {}
    for name in (
        "duk-core",
        "numpy",
        "scipy",
        "sympy",
        "matplotlib",
        "nbclient",
        "ipykernel",
        "ipywidgets",
    ):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    manifest = {
        "notebooks": list(selected),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "packages": packages,
        "files_sha256": hashes,
        "source_sha256": hashlib.sha256(
            json.dumps(hashes, sort_keys=True).encode()
        ).hexdigest(),
        "commit": None,
        "git_status": None,
    }
    if checkout.returncode == 0:
        manifest["commit"] = subprocess.check_output(
            ["git", "-C", str(repository), "rev-parse", "HEAD"], text=True
        ).strip()
        manifest["git_status"] = subprocess.check_output(
            ["git", "-C", str(repository), "status", "--porcelain"], text=True
        ).splitlines()
    return manifest, repository


def check_widget_outputs(notebook):
    """Fail on callback errors captured by Output widgets rather than nbclient."""
    state = (
        notebook.get("metadata", {})
        .get("widgets", {})
        .get("application/vnd.jupyter.widget-state+json", {})
        .get("state", {})
    )
    for widget in state.values():
        for output in widget.get("state", {}).get("outputs", []):
            if output.get("output_type") == "error":
                raise RuntimeError(
                    f"Widget callback failed: {output.get('ename', 'Error')}: "
                    f"{output.get('evalue', '')}"
                )


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--notebook",
        choices=NOTEBOOKS,
        action="append",
        help="Run only this notebook; repeat to select several (canonical order)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List selected notebooks without starting kernels",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=300,
        help="Timeout in seconds per code cell (default: 300)",
    )
    parser.add_argument(
        "--output-dir", type=Path, help="Write executed notebook copies here"
    )
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be a positive number of seconds")
    root = (Path(__file__).resolve().parents[1] / "notebooks").resolve()
    requested = set(args.notebook or NOTEBOOKS)
    selected = tuple(path for path in NOTEBOOKS if path in requested)
    if args.list:
        print("\n".join(selected))
        return
    output_dir = args.output_dir
    if output_dir is not None:
        # Resolve each destination as well as its parent to catch symlinks
        # pointing back into the source tree.
        if any((output_dir / path).resolve().is_relative_to(root) for path in selected):
            parser.error(
                "--output-dir must keep all executed copies outside the source notebooks"
            )
    manifest, repository = run_manifest(root.parent, selected)
    print("Source SHA-256:", manifest["source_sha256"], flush=True)
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "run-manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
        with tarfile.open(output_dir / "source-snapshot.tar.gz", "w:gz") as archive:
            for name in manifest["files_sha256"]:
                archive.add(repository / name, arcname=name, recursive=False)
    kernel_env = os.environ | {
        "MPLBACKEND": "module://matplotlib_inline.backend_inline"
    }
    for path in selected:
        print(f"RUN {path}", flush=True)
        source = root / path
        notebook = nbformat.read(source, as_version=4)
        nbformat.validate(notebook)
        client = NotebookClient(notebook, kernel_name="python3", timeout=args.timeout)
        notebook.metadata["duk_run"] = manifest
        try:
            client.execute(cwd=source.parent, env=kernel_env)
            check_widget_outputs(notebook)
        finally:
            if output_dir is not None:
                destination = output_dir / path
                destination.parent.mkdir(parents=True, exist_ok=True)
                nbformat.write(notebook, destination)
        for cell in notebook.cells:
            for cell_output in cell.get("outputs", []):
                if cell_output.output_type == "stream" and cell_output.name == "stdout":
                    print(cell_output.text.rstrip())
        print(f"PASS {path}", flush=True)


if __name__ == "__main__":
    main()
