"""Execute the DUK v3 notebook sequence in fresh Jupyter kernels."""

import os
from argparse import ArgumentParser
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


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, help="Write executed notebook copies here"
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1] / "notebooks"
    kernel_env = os.environ | {
        "MPLBACKEND": "module://matplotlib_inline.backend_inline"
    }
    for path in NOTEBOOKS:
        source = root / path
        notebook = nbformat.read(source, as_version=4)
        nbformat.validate(notebook)
        client = NotebookClient(notebook, kernel_name="python3", timeout=300)
        client.execute(cwd=source.parent, env=kernel_env)
        for cell in notebook.cells:
            for output in cell.get("outputs", []):
                if output.output_type == "stream" and output.name == "stdout":
                    print(output.text.rstrip())
        if args.output_dir is not None:
            destination = args.output_dir / path
            destination.parent.mkdir(parents=True, exist_ok=True)
            nbformat.write(notebook, destination)
        print(f"PASS {path}", flush=True)


if __name__ == "__main__":
    main()
