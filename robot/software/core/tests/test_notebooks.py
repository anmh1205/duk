"""Static validation for committed DUK Jupyter notebooks."""

from __future__ import annotations

import json
from pathlib import Path

import pytest


NOTEBOOK_ROOT = Path(__file__).resolve().parents[1] / "notebooks"
NOTEBOOKS = tuple(sorted(NOTEBOOK_ROOT.rglob("*.ipynb")))


@pytest.mark.parametrize(
    "notebook_path",
    NOTEBOOKS,
    ids=lambda path: str(path.relative_to(NOTEBOOK_ROOT)),
)
def test_notebook_code_cells_compile(notebook_path: Path) -> None:
    notebook = json.loads(
        notebook_path.read_text(encoding="utf-8")
    )
    assert notebook.get("nbformat") == 4

    for index, cell in enumerate(notebook.get("cells", [])):
        if cell.get("cell_type") != "code":
            continue
        source = "".join(cell.get("source", []))
        compile(
            source,
            f"{notebook_path}::cell[{index}]",
            "exec",
        )


def test_motion_notebook_uses_real_animation_player() -> None:
    """Prevent regression to overlaid static mechanism snapshots."""
    notebook_path = NOTEBOOK_ROOT / "motion" / "duk-leg-motion.ipynb"
    notebook = json.loads(
        notebook_path.read_text(encoding="utf-8")
    )
    source = "\n".join(
        "".join(cell.get("source", []))
        for cell in notebook.get("cells", [])
    )

    assert "create_leg_animation" in source
    assert "to_jshtml()" in source
    assert "motion snapshots" not in source.lower()


def test_notebook_runner_rejects_an_output_symlink_into_the_source(tmp_path):
    import subprocess
    import sys

    pytest.importorskip("nbclient")
    output = tmp_path / "executed"
    output.mkdir()
    (output / "kinematics").symlink_to(
        NOTEBOOK_ROOT / "kinematics", target_is_directory=True
    )
    runner = NOTEBOOK_ROOT.parent / "examples" / "run_duk_notebooks.py"
    result = subprocess.run(
        [sys.executable, str(runner), "--output-dir", str(output)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 2
    assert "outside the source notebooks" in result.stderr
