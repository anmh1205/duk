"""Presentation preserves actual stop times and works without widget rendering."""

import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

import matplotlib.pyplot as plt

from duk_core.visualization.notebook import (
    case_selector,
    plot_held_signal,
    plot_status_timeline,
    result_table,
)


def test_result_table_escapes_labels_and_distinguishes_missing_metrics():
    html = result_table(
        [{"Case": "<trial>", "Peak": float("nan"), "Criterion": "FAIL"}],
        caption="A & B",
    )
    assert "<trial>" not in html
    assert "&lt;trial&gt;" in html
    assert "A &amp; B" in html
    assert ">n/a<" in html
    assert ">FAIL<" in html


def test_timeline_does_not_extend_a_stop_sample_past_the_terminal_time():
    fig, ax = plt.subplots()
    try:
        plot_status_timeline(
            ax,
            [0.0, 0.15, 0.3],
            ["normal", "fallback", "stop"],
            end_time=0.3,
            title="Supervisor",
        )
        assert ax.get_xlim() == pytest.approx((0.0, 0.3))
        assert [label.get_text() for label in ax.get_yticklabels()] == [
            "normal",
            "fallback",
            "stop",
        ]
        # The stop is a marker, not an interval with a made-up duration.
        assert ax.lines[-1].get_xdata() == pytest.approx([0.3])
        assert ax.lines[-1].get_ydata() == pytest.approx([2])
    finally:
        plt.close(fig)


def test_case_picker_keeps_a_static_initial_figure_and_replaces_it(monkeypatch):
    display_module = pytest.importorskip("IPython.display")
    pytest.importorskip("ipywidgets")
    from types import SimpleNamespace

    displayed, updates, drawn = [], [], []
    handle = SimpleNamespace(update=updates.append)

    def display(value, **kwargs):
        displayed.append(value)
        return handle if kwargs.get("display_id") else None

    def draw(name):
        drawn.append(name)
        fig, ax = plt.subplots()
        ax.set_title(name)
        return fig

    monkeypatch.setattr(display_module, "display", display)
    before = plt.get_fignums()
    picker = case_selector(("nominal", "stopped"), draw)
    try:
        assert drawn == ["nominal"]
        assert displayed[1].axes[0].get_title() == "nominal"
        picker.value = "stopped"
        assert drawn == ["nominal", "stopped"]
        assert updates[0].axes[0].get_title() == "stopped"
        assert plt.get_fignums() == before
    finally:
        picker.close()


def test_held_signal_includes_the_exact_partial_last_interval():
    import numpy as np

    fig, ax = plt.subplots()
    try:
        (line,) = plot_held_signal(
            ax, [0.0, 0.02, 0.04], [1.0, -2.0, 3.0], end_time=0.047
        )
        times, values = line.get_xdata(), line.get_ydata()
        assert times[-1] == 0.047
        assert values[-1] == 3.0
        # The plotted zero-order hold has the same integral as the held commands.
        assert np.diff(times) @ values[:-1] == pytest.approx(0.001)
    finally:
        plt.close(fig)


def test_held_signal_does_not_give_an_instantaneous_stop_a_duration():
    fig, ax = plt.subplots()
    try:
        lines = plot_held_signal(
            ax, [0.0, 0.02], [[1.0, -1.0], [0.0, 0.0]], end_time=0.02
        )
        assert len(lines) == 2
        for line in lines:
            assert list(line.get_xdata()) == [0.0, 0.02]
            assert line.get_ydata()[-1] == 0.0
    finally:
        plt.close(fig)


def test_held_signal_preserves_a_rejected_command_gap():
    import math

    fig, ax = plt.subplots()
    try:
        (line,) = plot_held_signal(ax, [0.0, 0.02], [1.0, float("nan")], end_time=0.02)
        assert math.isnan(line.get_ydata()[-1])
    finally:
        plt.close(fig)
