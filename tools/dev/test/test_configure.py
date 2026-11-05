"""Check graphics switching without requiring a real GPU or desktop session."""

import importlib.util
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    "duk_dev", Path(__file__).resolve().parents[1] / "duk-dev.py"
)
DEV = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DEV)


class ConfigurationTests(unittest.TestCase):
    def test_failed_preflight_preserves_saved_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory)
            state = local / "settings.json"
            override = local / "compose.json"
            state.write_text('{"gpu": "software", "gui": "none"}\n')
            override.write_text('{"services": {"dev": {}}}\n')
            previous = (state.read_bytes(), override.read_bytes())
            with (
                patch.multiple(DEV, STATE=state, OVERRIDE=override),
                patch.object(DEV, "prepare", side_effect=RuntimeError("no desktop")),
            ):
                with self.assertRaisesRegex(RuntimeError, "no desktop"):
                    DEV.configure({"gpu": "auto", "gui": "x11"})
            self.assertEqual(previous, (state.read_bytes(), override.read_bytes()))

    def test_rejects_invalid_saved_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            state = Path(directory) / "settings.json"
            with patch.object(DEV, "STATE", state):
                for content in ("{", "[]", '{"gpu":"nvidia","gui":"none"}'):
                    state.write_text(content)
                    with self.assertRaises(RuntimeError):
                        DEV.settings()

    def test_switching_to_software_removes_devices_and_groups(self):
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory)
            state = local / "settings.json"
            override = local / "compose.json"
            device = local / "renderD128"
            device.touch()
            response = subprocess.CompletedProcess(
                [],
                0,
                stdout=json.dumps(
                    {"services": {"dev": {"build": {"args": {"ROS_DISTRO": "jazzy"}}}}}
                ),
            )
            with (
                patch.multiple(DEV, LOCAL=local, STATE=state, OVERRIDE=override),
                patch.object(DEV, "run", return_value=response),
                patch.object(Path, "glob", return_value=iter([device])),
            ):
                state.write_text(json.dumps({"gpu": "dri", "gui": "none"}))
                DEV.prepare()
                service = json.loads(override.read_text())["services"]["dev"]
                self.assertEqual(service["devices"], [f"{device}:{device}"])
                self.assertEqual(service["group_add"], [str(device.stat().st_gid)])
            with (
                patch.multiple(DEV, LOCAL=local, STATE=state, OVERRIDE=override),
                patch.object(DEV, "run", return_value=response),
            ):
                state.write_text(json.dumps({"gpu": "software", "gui": "none"}))
                DEV.prepare()
                service = json.loads(override.read_text())["services"]["dev"]
                self.assertNotIn("devices", service)
                self.assertNotIn("group_add", service)
                self.assertNotIn("volumes", service)
                self.assertEqual(service["environment"]["LIBGL_ALWAYS_SOFTWARE"], "1")

    def test_rejects_humble_resolved_from_compose(self):
        response = subprocess.CompletedProcess(
            [],
            0,
            stdout=json.dumps(
                {"services": {"dev": {"build": {"args": {"ROS_DISTRO": "humble"}}}}}
            ),
        )
        with patch.object(DEV, "run", return_value=response):
            with self.assertRaisesRegex(RuntimeError, "jazzy only"):
                DEV.prepare()


if __name__ == "__main__":
    unittest.main()
