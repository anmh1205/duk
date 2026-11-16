"""Presentation helpers for already-computed notebook results.

No simulations run in widget callbacks. The initial figure is also a regular
notebook output, so executed copies remain readable without a widget manager.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from html import escape
from math import isfinite
from numbers import Real
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from ipywidgets import Dropdown
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D
    from numpy.typing import ArrayLike


def configure_notebook_plots() -> None:
    """Set a consistent, readable style for this notebook's Matplotlib figures."""
    import matplotlib.pyplot as plt
    from cycler import cycler

    plt.rcParams.update(
        {
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.18,
            "axes.titleweight": "bold",
            "axes.prop_cycle": cycler(
                color=["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00"]
            ),
            "font.size": 10,
            "figure.dpi": 110,
            "savefig.dpi": 160,
        }
    )


def result_table(rows: Sequence[Mapping[str, Any]], *, caption: str) -> str:
    """Return an escaped HTML table of scalar study metrics, without Pandas.

    Column order follows the first row. Nonfinite numeric results display as
    ``n/a``. PASS/FAIL describe only criteria explicitly computed by the caller.
    """
    if not rows:
        raise ValueError("A results table requires at least one row")
    columns = tuple(rows[0])
    if any(set(row) != set(columns) for row in rows):
        raise ValueError("All results rows must have the same columns")

    def cell(value: Any) -> str:
        if isinstance(value, bool):
            value = "Yes" if value else "No"
        elif isinstance(value, Real):
            value = f"{value:.4g}" if isfinite(value) else "n/a"
        content = escape(str(value))
        color = {
            "PASS": "#167245",
            "FAIL": "#b42318",
            "COMPLETED": "#167245",
            "STOPPED": "#9a6700",
        }.get(content)
        if color:
            content = f'<strong style="color:{color}">{content}</strong>'
        return f"<td>{content}</td>"

    header = "".join(f'<th scope="col">{escape(str(key))}</th>' for key in columns)
    body = "".join(
        "<tr>" + "".join(cell(row[key]) for key in columns) + "</tr>" for row in rows
    )
    return (
        "<style>.duk-results{border-collapse:collapse;font-size:14px;width:100%;"
        "font-variant-numeric:tabular-nums}.duk-results caption{text-align:left;"
        "font-weight:600;padding:10px 0}.duk-results th,.duk-results td{"
        "padding:8px 12px;border-bottom:1px solid #8884;text-align:left}"
        ".duk-results th{border-bottom:2px solid #8888}"
        ".duk-results tbody tr:nth-child(even){background:#8881}</style>"
        '<div style="overflow-x:auto"><table class="duk-results">'
        f"<caption>{escape(caption)}</caption><thead><tr>{header}</tr></thead>"
        f"<tbody>{body}</tbody></table></div>"
    )


def plot_status_timeline(
    ax: Axes,
    times: Sequence[float],
    statuses: Sequence[str],
    *,
    end_time: float,
    title: str,
) -> Axes:
    """Draw categorical sample-held states through the actual terminal time.

    A state logged at the stop instant has zero duration and is shown as a
    marker. It is never extended to the requested experiment horizon.
    """
    import numpy as np

    times = np.asarray(times, dtype=float)
    if (
        times.ndim != 1
        or not times.size
        or len(statuses) != len(times)
        or not np.isfinite(times).all()
        or not np.isfinite(end_time)
        or np.any(np.diff(times) <= 0)
        or end_time < times[-1]
    ):
        raise ValueError(
            "Timeline requires increasing finite times and a valid end time"
        )
    names = tuple(dict.fromkeys(statuses))
    colors = {
        "normal": "#009E73",
        "solved": "#009E73",
        "rolling": "#009E73",
        "reduced": "#E69F00",
        "fallback": "#D55E00",
        "sliding": "#D55E00",
        "stop": "#b42318",
        "flight": "#CC79A7",
        "deadline": "#E69F00",
        "infeasible": "#b42318",
    }
    ends = np.r_[times[1:], end_time]
    for row, name in enumerate(names):
        spans = [
            (start, end - start)
            for start, end, status in zip(times, ends, statuses)
            if status == name and end > start
        ]
        ax.broken_barh(spans, (row - 0.3, 0.6), color=colors.get(name, "#0072B2"))
        if statuses[-1] == name and ends[-1] == times[-1]:
            ax.plot(times[-1], row, "|", color=colors.get(name, "#0072B2"), ms=12)
    ax.set(
        yticks=range(len(names)),
        yticklabels=names,
        ylim=(-0.6, len(names) - 0.4),
        xlabel="Time (s)",
        title=title,
    )
    if end_time > times[0]:
        ax.set_xlim(times[0], end_time)
    ax.grid(False, axis="y")
    return ax


def plot_held_signal(
    ax: Axes,
    times: ArrayLike,
    values: ArrayLike,
    *,
    end_time: float,
    **kwargs: Any,
) -> list[Line2D]:
    """Plot sample-held values through the actual end of the last interval.

    Extend only when end_time exceeds the last command time. A stop command
    logged at end_time has no duration. NaN values retain gaps for rejected
    commands; no interpolation or replacement value is introduced.
    """
    import numpy as np

    times, values = np.asarray(times, dtype=float), np.asarray(values, dtype=float)
    if (
        times.ndim != 1
        or not times.size
        or values.ndim not in (1, 2)
        or values.shape[0] != len(times)
        or not np.isfinite(times).all()
        or not np.isfinite(end_time)
        or np.any(np.diff(times) <= 0)
        or end_time < times[-1]
    ):
        raise ValueError("Held signals require matching samples and a valid end time")
    if end_time > times[-1]:
        times = np.r_[times, end_time]
        values = np.concatenate((values, values[-1:]), axis=0)
    return ax.step(times, values, where="post", **kwargs)


def case_selector(
    cases: Sequence[str],
    draw: Callable[[str], Figure],
    *,
    initial: str | None = None,
) -> Dropdown:
    """Display a case picker and a static initial figure; return the widget.

    ``draw(name)`` consumes cached results and returns a fresh Figure. Changing
    the picker replaces that same figure output and closes the Matplotlib figure
    after display. This keeps both interactive use and headless exports compact.
    """
    import ipywidgets as widgets
    import matplotlib.pyplot as plt
    from IPython.display import display

    if not cases or len(set(cases)) != len(cases):
        raise ValueError("Specify distinct, nonempty case names")
    selected = cases[0] if initial is None else initial
    if selected not in cases:
        raise ValueError("Initial case must belong to the available cases")
    picker = widgets.Dropdown(
        options=list(cases),
        value=selected,
        description="Case:",
        layout=widgets.Layout(width="max-content"),
    )
    display(picker)
    figure = draw(selected)
    try:
        handle = display(figure, display_id=True)
    finally:
        plt.close(figure)

    def update(change):
        figure = draw(change["new"])
        try:
            handle.update(figure)
        finally:
            plt.close(figure)

    picker.observe(update, names="value")
    return picker
