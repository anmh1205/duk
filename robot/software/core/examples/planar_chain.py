"""Smoke example using the published duk_core API (generic planar chain)."""

from math import pi

from duk_core.modeling.planar import forward_chain


def main() -> None:
    points = forward_chain([1.0, 0.5], [0.0, pi / 2])
    for index, (x, y) in enumerate(points):
        print(f"point[{index}] x={x:.3f}, y={y:.3f}")


if __name__ == "__main__":
    main()
