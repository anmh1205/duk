#!/usr/bin/env python3
"""Local Linux/VMware simulation setup; never installs host drivers or packages."""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / ".local/simulation"
STATE = LOCAL / "settings.json"
OVERRIDE = LOCAL / "compose.json"


def run(args, **kwargs):
    return subprocess.run([str(a) for a in args], check=True, **kwargs)


def settings():
    try:
        config = (
            json.loads(STATE.read_text())
            if STATE.exists()
            else {"gpu": "auto", "gui": "x11"}
        )
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Invalid JSON in {STATE}") from exc
    validate_settings(config)
    return config


def validate_settings(config):
    if not isinstance(config, dict) or set(config) != {"gpu", "gui"}:
        raise RuntimeError("Settings must contain exactly gpu and gui")
    if config["gpu"] not in ("auto", "dri", "software") or config["gui"] not in (
        "x11",
        "none",
    ):
        raise RuntimeError(
            "Unsupported gpu/gui setting; run configure with valid flags"
        )


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def prepare(config=None):
    if sys.platform != "linux":
        raise RuntimeError(
            "Only local Linux/VMware is implemented; Windows/macOS are not configured."
        )
    # Compose also reads the root .env, so check the resolved configuration rather
    # than only the caller's shell environment.
    base = json.loads(
        run(
            [
                "docker",
                "compose",
                "-f",
                ROOT / "compose.yaml",
                "config",
                "--format",
                "json",
            ],
            capture_output=True,
            text=True,
        ).stdout
    )
    distro = base["services"]["dev"]["build"]["args"]["ROS_DISTRO"]
    if distro != "jazzy":
        raise RuntimeError(
            "This simulation extension currently supports ROS_DISTRO=jazzy only."
        )
    config = settings() if config is None else config
    validate_settings(config)
    requested = config["gpu"]
    render_nodes = sorted(Path("/dev/dri").glob("renderD*"))
    gpu = ("dri" if render_nodes else "software") if requested == "auto" else requested
    service = {"environment": {"DUK_GPU_MODE": gpu, "DUK_GUI_MODE": config["gui"]}}
    if gpu == "dri":
        if not render_nodes:
            raise RuntimeError("No /dev/dri/renderD* device; select --gpu software.")
        service["devices"] = [f"{p}:{p}" for p in render_nodes]
        service["group_add"] = sorted({str(p.stat().st_gid) for p in render_nodes})
    else:
        service["environment"]["LIBGL_ALWAYS_SOFTWARE"] = "1"
    if config["gui"] == "x11":
        display = os.environ.get("DISPLAY", "")
        match = re.fullmatch(r":(\d+)(?:\.\d+)?", display)
        if not match or not Path(f"/tmp/.X11-unix/X{match[1]}").exists():
            raise RuntimeError(
                "A local X11/XWayland DISPLAY is required; use --gui none for headless."
            )
        if not shutil.which("xauth"):
            raise RuntimeError(
                "Host xauth is missing. Install xauth on the host before GUI setup."
            )
        records = run(
            ["xauth", "nlist", display], capture_output=True, text=True
        ).stdout
        if not records.strip():
            raise RuntimeError(
                "No Xauthority cookie for the current DISPLAY. Run from your desktop terminal."
            )
        runtime = Path(os.environ.get("XDG_RUNTIME_DIR", f"/tmp/duk-{os.getuid()}"))
        digest = hashlib.sha256(str(ROOT).encode()).hexdigest()[:10]
        auth_dir = runtime / f"duk-sim-{digest}"
        auth_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        auth_dir.chmod(0o700)
        temp_auth = auth_dir / "Xauthority.next"
        temp_auth.write_bytes(b"")
        temp_auth.chmod(0o600)
        # FamilyWild permits the container hostname, without granting xhost access.
        records = "\n".join("ffff" + line[4:] for line in records.splitlines()) + "\n"
        run(
            ["xauth", "-f", temp_auth, "nmerge", "-"],
            input=records,
            text=True,
            capture_output=True,
        )
        auth = auth_dir / "Xauthority"
        # Keep the inode stable for a running container's read-only bind mount.
        auth.write_bytes(temp_auth.read_bytes())
        auth.chmod(0o600)
        temp_auth.unlink()
        service["environment"].update(DISPLAY=display, XAUTHORITY="/tmp/duk.Xauthority")
        service["volumes"] = [
            {
                "type": "bind",
                "source": "/tmp/.X11-unix",
                "target": "/tmp/.X11-unix",
                "read_only": True,
            },
            {
                "type": "bind",
                "source": str(auth),
                "target": "/tmp/duk.Xauthority",
                "read_only": True,
            },
        ]
    LOCAL.mkdir(parents=True, exist_ok=True)
    write_json(OVERRIDE, {"services": {"dev": service}})
    print(
        f"Simulation: jazzy, gpu={gpu} (requested {requested}), gui={config['gui']}",
        flush=True,
    )


def configure(config):
    # Preflight must succeed before replacing the last usable saved settings.
    prepare(config)
    write_json(STATE, config)


def compose(*args, **kwargs):
    return run(
        [
            "docker",
            "compose",
            "-f",
            ROOT / "compose.yaml",
            "-f",
            ROOT / "environment/compose/simulation.yaml",
            "-f",
            OVERRIDE,
            *args,
        ],
        **kwargs,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        choices=[
            "configure",
            "prepare",
            "status",
            "build",
            "up",
            "down",
            "stop",
            "doctor",
            "demo",
            "kinematics",
            "shell",
        ],
    )
    parser.add_argument("--preset", choices=["vmware", "headless"])
    parser.add_argument("--gpu", choices=["auto", "dri", "software"])
    parser.add_argument("--gui", choices=["x11", "none"])
    parser.add_argument(
        "--no-build",
        action="store_true",
        help="up: reuse an already-built dependency image",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="demo/kinematics: do not open preview windows",
    )
    parser.add_argument(
        "--angle-deg",
        type=float,
        help="demo: canonical linkage q, in degrees (default 75)",
    )
    args = parser.parse_args()
    if args.no_build and args.command != "up":
        parser.error("--no-build belongs to up")
    if args.command != "configure" and any([args.preset, args.gpu, args.gui]):
        parser.error("--preset/--gpu/--gui belong to configure")
    if args.command not in ("demo", "kinematics") and args.headless:
        parser.error("--headless belongs to demo or kinematics")
    if args.command != "demo" and args.angle_deg is not None:
        parser.error("--angle-deg belongs to demo")
    if args.angle_deg is not None and not math.isfinite(args.angle_deg):
        parser.error("--angle-deg must be finite")
    if args.command == "configure":
        config = {"gpu": "auto", "gui": "x11"} if args.preset else settings()
        if args.preset:
            config.update(
                gpu="auto" if args.preset == "vmware" else "software",
                gui="x11" if args.preset == "vmware" else "none",
            )
        for key in ("gpu", "gui"):
            if getattr(args, key):
                config[key] = getattr(args, key)
        configure(config)
        print(
            "Apply with: python3 tools/dev/duk-dev.py up; VS Code: select DUK — VMware Simulation and rebuild."
        )
    elif args.command == "status":
        print(json.dumps(settings(), indent=2))
        print(f"Local override: {OVERRIDE}")
    elif args.command == "down":
        if OVERRIDE.exists():
            compose("down")
    elif args.command == "stop":
        if OVERRIDE.exists():
            code = (
                "from pathlib import Path; import os, signal; "
                "p=Path('/tmp/duk-preview-launch.pid'); "
                "pid=int(p.read_text()) if p.exists() else 0; "
                "c=Path(f'/proc/{pid}/cmdline'); "
                "cmd=c.read_bytes() if c.exists() else b''; "
                "expected=(b'preview.launch.py' in cmd and b'duk_gazebo' in cmd) or (b'kinematics.launch.py' in cmd and b'duk_description' in cmd); "
                "os.kill(pid, signal.SIGINT) if expected else None"
            )
            compose("exec", "-T", "dev", "python3", "-c", code)
    else:
        prepare()
        if args.command == "build":
            compose("build", "dev")
        elif args.command == "up":
            compose(
                "up", "-d", *([] if args.no_build else ["--build"]), "--wait", "dev"
            )
        elif args.command == "shell":
            compose("exec", "dev", "bash")
        elif args.command == "doctor":
            compose(
                "exec",
                "-T",
                "dev",
                "bash",
                "-lc",
                'source /opt/ros/jazzy/setup.bash && bash /workspace/tools/environment-check/verify-dev-container.sh --quick && command -v rviz2 gz && if [ "$DUK_GUI_MODE" = x11 ]; then glxinfo -B; fi',
            )
        elif args.command in ("demo", "kinematics"):
            headless = args.headless or settings()["gui"] == "none"
            is_kinematic = args.command == "kinematics"
            target = (
                f"duk_description kinematics.launch.py gui:={'false' if headless else 'true'}"
                if is_kinematic
                else f"duk_gazebo preview.launch.py q_deg:={75.0 if args.angle_deg is None else args.angle_deg} "
                f"gui:={'false' if headless else 'true'} rviz:={'false' if headless else 'true'}"
            )
            launch = (
                "echo $$ > /tmp/duk-preview-launch.pid && exec ros2 launch " + target
            )
            command = (
                "source /opt/ros/jazzy/setup.bash && "
                "cd /workspace/robot/software/ros2_ws && "
                "colcon build --packages-select duk_description duk_gazebo --symlink-install && "
                "source /opt/duk-ros/install/setup.bash && " + launch
            )
            compose(
                "exec",
                "-T",
                "dev",
                "flock",
                "-n",
                "-E",
                "75",
                "/tmp/duk-preview.lock",
                "bash",
                "-lc",
                command,
            )


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"duk-dev: {exc}", file=sys.stderr)
        sys.exit(1)
