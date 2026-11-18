"""Start the DUK Qt workbench or run a saved scenario without Qt."""

import argparse
from pathlib import Path

from workbench.inputs import LEFT_GRIP, PROFILE, Inputs, read_object
from workbench.records import RunRecord, decimal
from workbench.session import Workbench


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--scenario", type=Path, help="Load a scenario JSON")
    parser.add_argument("--open", type=Path, help="Load a previously saved run")
    parser.add_argument(
        "--output", type=Path, help="Run without Qt and write the result"
    )
    parser.add_argument("--controller", help="Override the scenario controller")
    parser.add_argument(
        "--no-rviz", action="store_true", help="Open Qt without the ROS viewer"
    )
    args = parser.parse_args()
    if args.scenario and args.open:
        parser.error("Choose --scenario or --open")
    if args.controller and args.open:
        parser.error("--controller applies to a scenario")
    workbench = Workbench(Inputs(args.profile))
    options = read_object(args.scenario or LEFT_GRIP)
    if args.controller:
        options["controller"] = args.controller
    saved = RunRecord.load(args.open) if args.open else None
    if args.output:
        result = workbench.verify_replay(saved) if saved else workbench.run(options)
        result.save(args.output)
        print(f"{result.study} · {result.stop_reason} · {decimal(result.time_s[-1])} s")
        return

    from workbench.qt_panel import run_qt

    run_qt(workbench, options, saved, rviz=not args.no_rviz)


if __name__ == "__main__":
    main()
