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
