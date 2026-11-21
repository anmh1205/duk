"""Versioned run records with distinct sampled and terminal evidence."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any

import numpy as np

SCHEMA = 1
REPLAY_TOLERANCE = {"atol": 1e-8, "rtol": 1e-7}


def finite_number(value: Any) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def reject_json_constant(value: str) -> None:
    raise ValueError(f"Invalid JSON number {value}")


def plain(value: Any) -> Any:
    """Convert NumPy outputs to strict JSON, preserving missing samples as null."""
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return plain(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def decimal(value: Any, decimals: int = 6) -> str:
    """Format values for display only, with Vietnamese decimal notation."""
    if isinstance(value, bool):
        return "Có" if value else "Không"
    if value is None:
        return "—"
    if isinstance(value, (int, float, np.number)):
        if not finite_number(value.item() if isinstance(value, np.number) else value):
            return "—"
        step = 10 ** (-decimals)
        if 0 < abs(value) < step:
            operator = "<" if value > 0 else ">"
            bound = step if value > 0 else -step
            return f"{operator} {bound:.{decimals}f}".replace(".", ",")
        return f"{value:.{decimals}f}".rstrip("0").rstrip(".").replace(".", ",")
    if isinstance(value, (tuple, list, np.ndarray)):
        return "[" + " · ".join(decimal(item, decimals) for item in value) + "]"
    return str(value)


@dataclass(frozen=True)
class Signal:
    unit: str
    scope: str
    values: list[Any]
    frame: str = ""
    held: bool = False

    def __post_init__(self) -> None:
        if (
            not all(
                isinstance(value, str) for value in (self.unit, self.scope, self.frame)
            )
            or not isinstance(self.values, list)
            or type(self.held) is not bool
        ):
            raise ValueError("Invalid signal metadata or sample list")

    def plot_components(self) -> np.ndarray | None:
        """Convert scalar/vector numeric samples to rows with missing values as NaN."""
        available = [sample for sample in self.values if sample is not None]
        if not available:
            return None
        if all(type(sample) is bool for sample in available):
            return np.array([self.values], dtype=float)
        if all(finite_number(sample) for sample in available):
            return np.array([self.values], dtype=float)
        first = available[0]
        if not isinstance(first, list) or not first:
            return None
        width = len(first)
        if not all(
            isinstance(sample, list)
            and len(sample) == width
            and all(value is None or finite_number(value) for value in sample)
            for sample in available
        ):
            return None
        if not any(any(value is not None for value in sample) for sample in available):
            return None
        return np.array(
            [
                sample if sample is not None else [None] * width
                for sample in self.values
            ],
            dtype=float,
        ).T

    def same_quantity(self, other: Signal) -> bool:
        return (self.unit, self.scope, self.frame, self.held) == (
            other.unit,
            other.scope,
            other.frame,
            other.held,
        )


@dataclass(frozen=True)
class RunRecord:
    study: str
    configuration: dict[str, Any]
    provenance: dict[str, Any]
    time_s: list[float]
    signals: dict[str, Signal]
    stop_reason: str
    events: list[dict[str, Any]]
    metadata: dict[str, Any]
    replay_tolerance: dict[str, float] = field(default_factory=REPLAY_TOLERANCE.copy)

    def __post_init__(self) -> None:
        if (
            not isinstance(self.study, str)
            or not isinstance(self.stop_reason, str)
            or not isinstance(self.configuration, dict)
            or self.configuration.get("study") != self.study
            or not isinstance(self.provenance, dict)
            or not isinstance(self.metadata, dict)
            or not isinstance(self.events, list)
            or any(
                self.configuration.get(name) is not None
                and not isinstance(self.configuration[name], str)
                for name in ("controller", "case")
            )
        ):
            raise ValueError("Invalid study, configuration or metadata")
        times = self.time_s
        if (
            not isinstance(times, list)
            or not times
            or not all(map(finite_number, times))
            or times[0] < 0
            or any(b <= a for a, b in zip(times, times[1:]))
        ):
            raise ValueError(
                "Run times must be finite, nonnegative and strictly increasing"
            )
        if not self.signals or any(
            len(s.values) != len(times) for s in self.signals.values()
        ):
            raise ValueError("Every signal must contain one value per run time")
        if self.replay_tolerance != REPLAY_TOLERANCE:
            raise ValueError("Unsupported numerical tolerance for this run format")
        for event in self.events:
            if (
                not isinstance(event, dict)
                or not isinstance(event.get("name"), str)
                or any(
                    key not in event
                    or (
                        event[key] is not None
                        and (not finite_number(event[key]) or event[key] < 0)
                    )
                    for key in ("actual_s", "requested_s")
                )
            ):
                raise ValueError("Invalid run event")
            when = event["actual_s"]
            if when is not None and not times[0] <= when <= times[-1]:
                raise ValueError("An event lies outside the recorded run")

    def save(self, path: Path) -> None:
        save_json(path, {"schema_version": SCHEMA, **asdict(self)})

    @classmethod
    def load(cls, path: Path) -> RunRecord:
        try:
            payload = json.loads(
                path.read_text(encoding="utf-8"), parse_constant=reject_json_constant
            )
            if (
                not isinstance(payload, dict)
                or type(payload.get("schema_version")) is not int
                or payload.pop("schema_version") != SCHEMA
            ):
                raise ValueError("Unsupported run format")
            payload["signals"] = {
                name: Signal(**value) for name, value in payload["signals"].items()
            }
            return cls(**payload)
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError("Incomplete DUK run record") from exc

    def export_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "time_s",
                    *(f"{name} [{data.unit}]" for name, data in self.signals.items()),
                ]
            )
            for index, time in enumerate(self.time_s):
                writer.writerow(
                    [
                        time,
                        *(
                            json.dumps(
                                data.values[index], ensure_ascii=False, allow_nan=False
                            )
                            if isinstance(data.values[index], (list, dict))
                            else data.values[index]
                            for data in self.signals.values()
                        ),
                    ]
                )


def save_json(path: Path, payload: dict) -> None:
    with NamedTemporaryFile(
        dir=path.parent, mode="w", encoding="utf-8", delete=False
    ) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(
                plain(payload), stream, indent=2, ensure_ascii=False, allow_nan=False
            )
            stream.write("\n")
            stream.close()
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


def event(name: str, requested: float | None, actual: float | None) -> dict[str, Any]:
    return {"name": name, "requested_s": requested, "actual_s": actual}


def transitions(times: list[float], states: list, name: str) -> list[dict[str, Any]]:
    return [
        event(f"{name} · {mode}", None, time)
        for time, previous, mode in zip(times[1:], states, states[1:])
        if mode is not None and mode != previous
    ]
