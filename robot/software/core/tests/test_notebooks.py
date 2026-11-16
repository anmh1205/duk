"""Source-output hygiene and end-to-end notebook runner regressions."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

NOTEBOOK_ROOT = Path(__file__).resolve().parents[1] / "notebooks"
NOTEBOOKS = tuple(sorted(NOTEBOOK_ROOT.rglob("*.ipynb")))


def test_source_notebooks_contain_no_execution_artifacts() -> None:
    for path in NOTEBOOKS:
        notebook = json.loads(path.read_text(encoding="utf-8"))
        for cell in notebook["cells"]:
            if cell["cell_type"] == "code":
                assert cell["execution_count"] is None, path
                assert cell["outputs"] == [], path


def test_notebook_runner_rejects_errors_captured_inside_widgets():
    import runpy

    pytest.importorskip("nbclient")
    runner = NOTEBOOK_ROOT.parent / "examples" / "run_duk_notebooks.py"
    check = runpy.run_path(str(runner))["check_widget_outputs"]
    output = {"output_type": "error", "ename": "ValueError", "evalue": "Invalid pose"}
    notebook = {
        "metadata": {
            "widgets": {
                "application/vnd.jupyter.widget-state+json": {
                    "state": {"pose-output": {"state": {"outputs": [output]}}}
                }
            }
        }
    }
    with pytest.raises(
        RuntimeError, match="Widget callback failed: ValueError: Invalid pose"
    ):
        check(notebook)
    check({"metadata": {}})


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
    command = [
        sys.executable,
        str(runner),
        "--output-dir",
        str(output),
        "--notebook",
        "kinematics/duk-leg-kinematics.ipynb",
    ]
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 2
    assert "outside the source notebooks" in result.stderr


@pytest.mark.parametrize(
    "save,fail_second",
    [(False, False), (True, False), (True, True)],
    ids=["stdout-only", "saved", "second-kernel-failure"],
)
def test_notebook_runner_handles_multiple_kernels_and_preserves_source(
    tmp_path, save, fail_second
):
    import shutil
    import subprocess
    import sys

    nbformat = pytest.importorskip("nbformat")
    pytest.importorskip("nbclient")
    pytest.importorskip("ipykernel")
    core = tmp_path / "core"
    runner = core / "examples/run_duk_notebooks.py"
    runner.parent.mkdir(parents=True)
    shutil.copyfile(NOTEBOOK_ROOT.parent / "examples/run_duk_notebooks.py", runner)
    names = ("kinematics/duk-leg-kinematics.ipynb", "motion/duk-leg-motion.ipynb")
    originals = {}
    for index, name in enumerate(names):
        source = core / "notebooks" / name
        source.parent.mkdir(parents=True)
        code = f"print('Notebook {index}')"
        if fail_second and index == 1:
            code += "\nraise RuntimeError('Expected second-kernel failure')"
        nbformat.write(
            nbformat.v4.new_notebook(cells=[nbformat.v4.new_code_cell(code)]), source
        )
        originals[source] = source.read_bytes()
    output = tmp_path / "executed"
    command = [sys.executable, str(runner), "--timeout", "15"]
    for name in reversed(names):
        command += ["--notebook", name]
    if save:
        command += ["--output-dir", str(output)]
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=90,
    )
    assert (result.returncode != 0) == fail_second, result.stderr
    assert result.stdout.index(f"RUN {names[0]}") < result.stdout.index(
        f"RUN {names[1]}"
    )
    assert f"PASS {names[0]}" in result.stdout
    if fail_second:
        assert "Expected second-kernel failure" in result.stderr
        assert "TypeError" not in result.stderr
        assert f"PASS {names[1]}" not in result.stdout
    else:
        assert f"PASS {names[1]}" in result.stdout
    assert all(
        source.read_bytes() == original for source, original in originals.items()
    )
    assert output.exists() == save
    if save:
        import hashlib
        import tarfile

        manifest = json.loads((output / "run-manifest.json").read_text())
        assert manifest["notebooks"] == list(names)
        for name in names:
            executed = nbformat.read(output / name, as_version=4)
            assert executed.metadata["duk_run"] == manifest
        with tarfile.open(output / "source-snapshot.tar.gz") as archive:
            for name, expected in manifest["files_sha256"].items():
                assert (
                    hashlib.sha256(archive.extractfile(name).read()).hexdigest()
                    == expected
                )
        if fail_second:
            assert (
                executed.cells[0].outputs[-1].evalue == "Expected second-kernel failure"
            )
