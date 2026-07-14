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

"""Generate map-frame semantic coordinates from field-frame calibration."""

import argparse
import copy
import math
from pathlib import Path

import yaml


def _as_point(value):
    return float(value[0]), float(value[1])


def _round_point(x, y):
    return [round(x, 6), round(y, 6)]


def _round_pose(x, y, yaw):
    return {'x': round(x, 6), 'y': round(y, 6), 'yaw': round(yaw, 6)}


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _load_yaml(path):
    with Path(path).open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _save_yaml(path, data):
    with Path(path).open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def compute_transform(calibration_points):
    if len(calibration_points) < 2:
        raise ValueError('field_to_map.calibration_points requires at least two points')

    field_points = [_as_point(p['field']) for p in calibration_points]
    map_points = [_as_point(p['map']) for p in calibration_points]

    field_cx = sum(p[0] for p in field_points) / len(field_points)
    field_cy = sum(p[1] for p in field_points) / len(field_points)
    map_cx = sum(p[0] for p in map_points) / len(map_points)
    map_cy = sum(p[1] for p in map_points) / len(map_points)

    cross = 0.0
    dot = 0.0
    for field, mapped in zip(field_points, map_points):
        fx = field[0] - field_cx
        fy = field[1] - field_cy
        mx = mapped[0] - map_cx
        my = mapped[1] - map_cy
        cross += fx * my - fy * mx
        dot += fx * mx + fy * my

    yaw = math.atan2(cross, dot)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    tx = map_cx - (cos_yaw * field_cx - sin_yaw * field_cy)
    ty = map_cy - (sin_yaw * field_cx + cos_yaw * field_cy)
    return tx, ty, yaw


def transform_point(point, transform):
    x, y = _as_point(point)
    tx, ty, yaw = transform
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        tx + cos_yaw * x - sin_yaw * y,
        ty + sin_yaw * x + cos_yaw * y,
    )


def transform_pose(pose, transform):
    x, y = transform_point([pose['x'], pose['y']], transform)
    return _round_pose(x, y, _normalize_angle(float(pose['yaw']) + transform[2]))


def _transform_point_list(points, transform):
    return [_round_point(*transform_point(p, transform)) for p in points]


def _maybe_transform_node(node, transform):
    if isinstance(node, dict):
        if 'pose_field' in node:
            node['pose_map'] = transform_pose(node['pose_field'], transform)
        if 'min_scan_pose_field' in node and 'pose' in node['min_scan_pose_field']:
            node['min_scan_pose_map'] = copy.deepcopy(node['min_scan_pose_field'])
            node['min_scan_pose_map']['pose'] = transform_pose(
                node['min_scan_pose_field']['pose'], transform
            )
        if (
            'recommended_scan_pose_field' in node
            and 'pose' in node['recommended_scan_pose_field']
        ):
            node['recommended_scan_pose_map'] = copy.deepcopy(
                node['recommended_scan_pose_field']
            )
            node['recommended_scan_pose_map']['pose'] = transform_pose(
                node['recommended_scan_pose_field']['pose'], transform
            )
        if 'polygon_field' in node and isinstance(node['polygon_field'], list):
            node['polygon_map'] = _transform_point_list(node['polygon_field'], transform)
        if 'centerline_field' in node and isinstance(node['centerline_field'], list):
            node['centerline_map'] = _transform_point_list(
                node['centerline_field'], transform
            )
        if 'diagonal_min_pose_field' in node:
            node['diagonal_min_pose_map'] = _round_point(
                *transform_point(node['diagonal_min_pose_field'], transform)
            )
        if 'diagonal_recommended_pose_field' in node:
            node['diagonal_recommended_pose_map'] = _round_point(
                *transform_point(node['diagonal_recommended_pose_field'], transform)
            )
        if 'center_field' in node:
            center_x, center_y = transform_point(
                [node['center_field']['x'], node['center_field']['y']], transform
            )
            node['center_map'] = {
                'x': round(center_x, 6),
                'y': round(center_y, 6),
            }
        for value in node.values():
            _maybe_transform_node(value, transform)
    elif isinstance(node, list):
        for value in node:
            _maybe_transform_node(value, transform)


def generate(data):
    output = copy.deepcopy(data)
    field_to_map = output.setdefault('frames', {}).setdefault('field_to_map', {})
    calibration_points = field_to_map.get('calibration_points', [])
    transform = compute_transform(calibration_points)

    field_to_map['x_offset_m'] = round(transform[0], 6)
    field_to_map['y_offset_m'] = round(transform[1], 6)
    field_to_map['yaw_offset_rad'] = round(transform[2], 6)
    field_to_map['verified'] = True

    _maybe_transform_node(output, transform)
    return output


def generate_file(input_path, output_path):
    data = _load_yaml(input_path)
    generated = generate(data)
    _save_yaml(output_path, generated)


def main():
    parser = argparse.ArgumentParser(
        description='Generate map-frame semantic coordinates from field-frame YAML.'
    )
    parser.add_argument('--input', required=True, help='Input field semantic YAML')
    parser.add_argument('--output', required=True, help='Output generated YAML')
    args = parser.parse_args()
    generate_file(Path(args.input), Path(args.output))


if __name__ == '__main__':
    main()
