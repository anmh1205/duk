"""Shared fixture locations without importing one test module from another."""

import json
from pathlib import Path

import pytest


@pytest.fixture(scope="session")
def geometry_path() -> Path:
    return Path(__file__).resolve().parents[3] / "config/physical/duk_leg_geometry.json"


@pytest.fixture(scope="session")
def research_inputs(geometry_path):
    path = geometry_path.parents[3] / "research/experiments/duk-v3/synthetic.json"
    return json.loads(path.read_text(encoding="utf-8"))
