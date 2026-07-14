import math
import re
import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
WAYPOINTS = ROOT / 'config' / 'waypoints_flowpath_custom_rpp.yaml'
PARAMS = ROOT.parent / 'origincar_bringup' / 'config' / 'competition.yaml'
TASK_SOURCE = ROOT / 'src' / 'task_master.cpp'
MISSION_LAUNCH = ROOT.parent / 'origincar_bringup' / 'launch' / 'mission.launch.py'


def load_routes():
    return yaml.safe_load(WAYPOINTS.read_text(encoding='utf-8'))['routes']


def contiguous_groups(indices):
    groups = []
    for index in indices:
        if not groups or index != groups[-1][-1] + 1:
            groups.append([index])
        else:
            groups[-1].append(index)
    return groups


def test_p_qr_b_route_is_forward_and_joins_b_entry():
    routes = load_routes()
    route = routes['p_to_qr_to_b_rpp']
    assert len(route) == 33
    assert all(point['motion'] == 'forward' for point in route)
    expected = (2.014753, 1.786151)
    assert math.dist((route[-1]['x'], route[-1]['y']), expected) <= 1e-6
    for name in ('ring_cw', 'ring_ccw'):
        first = routes[name][0]
        assert math.dist((first['x'], first['y']), expected) <= 1e-6
    gaps = [
        math.dist((a['x'], a['y']), (b['x'], b['y']))
        for a, b in zip(route, route[1:])
    ]
    assert all(0.0 < gap <= 0.30 for gap in gaps)


def test_qr_scan_window_and_deadline_are_exact():
    route = load_routes()['p_to_qr_to_b_rpp']
    scan_indices = [i + 1 for i, point in enumerate(route) if point.get('qr_scan')]
    deadline_indices = [
        i + 1 for i, point in enumerate(route) if point.get('qr_deadline')
    ]
    assert scan_indices == [21, 22, 23, 24, 25]
    assert deadline_indices == [25]
    assert all(route[index - 1]['pass_radius'] == 0.10 for index in scan_indices)


def test_explicit_turn_groups_and_capture_points():
    routes = load_routes()
    expected_turns = {
        'ring_cw': [(7, 11), (19, 23), (28, 32), (51, 57), (65, 69), (78, 82)],
        'ring_ccw': [(8, 12), (19, 24), (32, 35), (55, 60), (66, 71), (78, 81)],
    }
    for name, expected in expected_turns.items():
        route = routes[name]
        indices = [
            i + 1 for i, point in enumerate(route)
            if point.get('speed_gain') == 0.50
            and point.get('angular_gain') == 1.50
        ]
        groups = [(group[0], group[-1]) for group in contiguous_groups(indices)]
        assert groups == expected
        captures = [point for point in route if point.get('capture')]
        assert len(captures) == 2
        assert all(point.get('pause', 0.0) == 0.0 for point in captures)

    return_turns = [
        i + 1 for i, point in enumerate(routes['return_to_p_rpp'])
        if point.get('speed_gain') == 0.50
        and point.get('angular_gain') == 1.50
    ]
    assert return_turns == [1, 2, 3]


def test_fixed_competition_parameters():
    params = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))[
        'task_master']['ros__parameters']
    assert params['cruise_speed'] == 0.85
    assert params['qr_scan_speed_gain'] == 0.85
    assert params['turn_speed_gain'] == 0.50
    assert params['turn_angular_gain'] == 1.50
    assert params['capture_speed_gain'] == 0.60
    assert params['obstacle_slow_distance'] == 0.45
    assert params['obstacle_avoid_distance'] == 0.45
    assert params['obstacle_backup_distance_threshold'] == 0.20
    assert params['obstacle_backup_speed_gain'] == 0.70
    assert params['obstacle_backup_distance'] == 0.40
    assert params['obstacle_backup_timeout_sec'] == 1.50
    assert params['rear_clearance_distance'] == 0.25
    assert params['default_qr_direction'] == 'cw'


def test_parameter_file_contains_no_unknown_task_parameters():
    source = TASK_SOURCE.read_text(encoding='utf-8')
    declared = set(re.findall(
        r'declare_parameter<[^>]+>\(\s*"([^"]+)"', source, re.DOTALL))
    params = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))[
        'task_master']['ros__parameters']
    # use_sim_time is declared by rclcpp's time source.
    assert set(params) - {'use_sim_time'} <= declared


def test_launch_defaults_do_not_override_yaml_with_different_values():
    tree = ast.parse(MISSION_LAUNCH.read_text(encoding='utf-8'))
    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != 'DeclareLaunchArgument':
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        default = next(
            (item.value for item in node.keywords if item.arg == 'default_value'),
            None)
        if isinstance(default, ast.Constant):
            defaults[node.args[0].value] = default.value

    params = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))[
        'task_master']['ros__parameters']
    for name, value in params.items():
        if name not in defaults:
            continue
        if isinstance(value, bool):
            assert defaults[name].lower() == str(value).lower()
        elif isinstance(value, (int, float)):
            assert float(defaults[name]) == float(value)
        else:
            assert defaults[name] == value
