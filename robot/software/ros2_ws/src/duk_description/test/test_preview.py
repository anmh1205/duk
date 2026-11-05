"""Protect canonical dimensions and frame mapping at the ROS/SDF boundary."""

import json
import math
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

from duk_core.modeling import load_leg_geometry_json, solve_leg_pose
from duk_description.preview import build_preview

ROOT = next(
    p for p in Path(__file__).resolve().parents if (p / "compose.yaml").exists()
)
GEOMETRY = ROOT / "robot/config/physical/duk_leg_geometry.json"


class PreviewGeometryTests(unittest.TestCase):
    def test_dimensions_frames_and_wheel_ground_contact(self):
        geometry = load_leg_geometry_json(GEOMETRY)
        lower, upper = geometry.geometric_linkage_angle_limits_rad()
        angles = (lower + 0.1, (lower + upper) / 2, upper - 0.1, math.radians(75))
        for q in angles:
            with self.subTest(q=q):
                urdf, sdf, metadata = build_preview(GEOMETRY, math.degrees(q))
                expected = solve_leg_pose(geometry, q)
                xml = ET.fromstring(urdf)
                for side in ("left", "right"):
                    points = metadata["points_world_m"][side]
                    for name, value in points.items():
                        self.assertAlmostEqual(value[0] * 1000, expected.point(name)[0])
                        self.assertAlmostEqual(
                            (value[2] - metadata["mount_height_m"]) * 1000,
                            expected.point(name)[1],
                        )
                    for start, end, length in (
                        ("A", "B", geometry.ab_mm),
                        ("C", "D", geometry.cd_mm),
                        ("A", "D", geometry.ad_mm),
                        ("B", "C", geometry.bc_mm),
                        ("B", "H", geometry.bh_mm),
                        ("O", "H", geometry.oh_mm),
                        ("B", "O", geometry.bo_mm),
                        ("C", "O", geometry.co_mm),
                    ):
                        self.assertAlmostEqual(
                            math.dist(points[start], points[end]) * 1000, length
                        )
                    for name, start, end in (
                        ("AB", "A", "B"),
                        ("CD", "C", "D"),
                        ("carrier_BC", "B", "C"),
                        ("carrier_BO", "B", "O"),
                        ("carrier_CO", "C", "O"),
                    ):
                        visual = xml.find(f"link/visual[@name='{side}_{name}']")
                        length = float(visual.find("geometry/cylinder").get("length"))
                        origin = visual.find("origin")
                        center = list(map(float, origin.get("xyz").split()))
                        pitch = float(origin.get("rpy").split()[1])
                        direction = (math.sin(pitch), 0, math.cos(pitch))
                        for sign, point in ((-1, start), (1, end)):
                            actual = [
                                center[i] + sign * length / 2 * direction[i]
                                for i in range(3)
                            ]
                            actual[2] += metadata["mount_height_m"]
                            for a, b in zip(actual, points[point]):
                                self.assertAlmostEqual(a, b, places=10)
                    radius = float(
                        xml.find(
                            f"link/visual[@name='{side}_wheel']/geometry/cylinder"
                        ).get("radius")
                    )
                    self.assertAlmostEqual(radius * 1000, geometry.wheel_radius_mm)
                    self.assertAlmostEqual(points["O"][2] - radius, 0)
                self.assertEqual(sdf.find("model/static").text, "true")
                self.assertIsNone(xml.find(".//inertial"))
                # Both renderers must receive the same primitive dimensions.
                for visual in xml.findall("link/visual"):
                    counterpart = sdf.find(
                        f"model/link/visual[@name='{visual.get('name')}']"
                    )
                    self.assertIsNotNone(counterpart)
                    cylinder = visual.find("geometry/cylinder")
                    if cylinder is not None:
                        self.assertEqual(
                            float(cylinder.get("length")),
                            float(counterpart.findtext("geometry/cylinder/length")),
                        )
                        self.assertEqual(
                            float(cylinder.get("radius")),
                            float(counterpart.findtext("geometry/cylinder/radius")),
                        )

    def test_reads_selected_geometry_instead_of_hardcoding_wheel(self):
        data = json.loads(GEOMETRY.read_text())
        data["wheel_mm"] = {"diameter": 160, "radius": 80}
        with tempfile.TemporaryDirectory() as directory:
            selected = Path(directory) / "geometry.json"
            selected.write_text(json.dumps(data))
            urdf, _, metadata = build_preview(selected)
            self.assertEqual(metadata["wheel_radius_m"], 0.08)
            self.assertEqual(
                float(
                    ET.fromstring(urdf)
                    .find("link/visual[@name='left_wheel']/geometry/cylinder")
                    .get("radius")
                ),
                0.08,
            )


if __name__ == "__main__":
    unittest.main()
