#!/usr/bin/env python3

# Copyright 2026 OriginCar Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the field-to-map semantic coordinate generator."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'generate_map_semantics.py'
)


def load_generator_module():
    spec = importlib.util.spec_from_file_location('generate_map_semantics', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GenerateMapSemanticsTest(unittest.TestCase):
    def test_generates_rotated_map_coordinates_from_calibration_points(self):
        module = load_generator_module()

        source = {
            'schema_version': 1,
            'frames': {
                'field_to_map': {
                    'calibration_points': [
                        {'name': 'origin', 'field': [0.0, 0.0], 'map': [1.0, 2.0]},
                        {'name': 'x_axis', 'field': [1.0, 0.0], 'map': [1.0, 3.0]},
                        {'name': 'y_axis', 'field': [0.0, 1.0], 'map': [0.0, 2.0]},
                    ]
                }
            },
            'points': {
                'sample_pose': {
                    'pose_field': {'x': 2.0, 'y': 1.0, 'yaw': 0.5},
                    'polygon_field': [[0.0, 0.0], [1.0, 0.0]],
                }
            },
            'zones': {
                'sample_zone': {
                    'centerline_field': [[0.0, 0.0], [1.0, 0.0]],
                    'polygon_map': [],
                }
            },
        }

        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'field.yaml'
            dst = Path(tmp) / 'map.yaml'
            src.write_text(yaml.safe_dump(source, sort_keys=False))

            module.generate_file(src, dst)
            generated = yaml.safe_load(dst.read_text())

        transform = generated['frames']['field_to_map']
        self.assertAlmostEqual(transform['yaw_offset_rad'], math.pi / 2, places=6)
        self.assertAlmostEqual(transform['x_offset_m'], 1.0, places=6)
        self.assertAlmostEqual(transform['y_offset_m'], 2.0, places=6)
        self.assertTrue(transform['verified'])

        pose = generated['points']['sample_pose']['pose_map']
        self.assertAlmostEqual(pose['x'], 0.0, places=6)
        self.assertAlmostEqual(pose['y'], 4.0, places=6)
        self.assertAlmostEqual(pose['yaw'], 0.5 + math.pi / 2, places=6)

        self.assertEqual(
            generated['points']['sample_pose']['polygon_map'],
            [[1.0, 2.0], [1.0, 3.0]],
        )
        self.assertEqual(
            generated['zones']['sample_zone']['centerline_map'],
            [[1.0, 2.0], [1.0, 3.0]],
        )


if __name__ == '__main__':
    unittest.main()
