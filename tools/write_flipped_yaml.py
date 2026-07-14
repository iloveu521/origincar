#!/usr/bin/env python3
"""Generate the full YAML in flipped coordinate system."""
import json, math, yaml

with open('tools/map_data.json') as f:
    md = json.load(f)

# ===== Build YAML =====
data = {
    'field_to_map': {
        'source': 'flowpath/dev_ws/src/origincar/origincar_system/config/waypoints.yaml',
        'source_map': 'race_modify_flipped.pgm',
        'source_yaml': 'race_modify_flipped.yaml',
        'method': 'flipped_180_bottom_left_origin',
        'resolution': 0.05,
        'note': 'Map flipped 180 degrees, origin at bottom-left (0,0)',
        'world_bounds': {'x_min': 0, 'x_max': 5.75, 'y_min': 0, 'y_max': 5.70},
    },
    'waypoints': {},
    'routes': {},
    'custom_rpp': {
        'stuck_skip': {
            'enabled': False, 'timeout_sec': 0.8, 'max_distance': 0.45,
            'recovery_enabled': False, 'recovery_max_distance': 1.2,
            'recovery_duration_sec': 0.35, 'recovery_speed': 0.12,
            'recovery_angular_z': 0.25,
            'description': 'Stuck skip & recovery are DISABLED.',
        },
    },
}

# Waypoints
wp = md['waypoints']
data['waypoints'] = {
    'point_P': {'x': wp['point_P']['x'], 'y': wp['point_P']['y'], 'yaw': 0.0,
                'description': 'P start/parking center (flipped coords)'},
    'qr_point': {'x': wp['qr_point']['x'], 'y': wp['qr_point']['y'], 'yaw': 0.335969,
                 'description': 'QR sign map position'},
    'qr_turn_point': {'x': wp['qr_turn_point']['x'], 'y': wp['qr_turn_point']['y'], 'yaw': 0.78,
                      'description': 'Vehicle QR scan pose'},
    'qr_backup_point': {'x': wp['qr_backup_point']['x'], 'y': wp['qr_backup_point']['y'], 'yaw': 2.05,
                        'description': 'Single reverse pose'},
    'zone_b_a_side': {'x': wp['zone_b_a_side']['x'], 'y': wp['zone_b_a_side']['y'], 'yaw': 1.645396,
                      'description': 'B corridor A side'},
    'zone_b_c_side': {'x': wp['zone_b_c_side']['x'], 'y': wp['zone_b_c_side']['y'], 'yaw': 2.500253,
                      'description': 'C side after B crossing'},
}

# p_to_qr_backup_b_anchors
data['routes']['p_to_qr_backup_b_anchors'] = [
    {'x': wp['point_P']['x'], 'y': wp['point_P']['y'], 'yaw': 0.065384,
     'pause': 0, 'motion': 'forward', 'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
    {'x': wp['qr_turn_point']['x'], 'y': wp['qr_turn_point']['y'], 'yaw': 0.78,
     'pause': 0, 'motion': 'forward', 'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
    {'x': wp['qr_backup_point']['x'], 'y': wp['qr_backup_point']['y'], 'yaw': 2.05,
     'pause': 0, 'motion': 'reverse', 'pass_radius': 0.12, 'reverse_pass_radius': 0.12},
    {'x': wp['zone_b_a_side']['x'], 'y': wp['zone_b_a_side']['y'], 'yaw': 1.645396,
     'pause': 0, 'motion': 'forward', 'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
]

# ring_ccw (new flipped)
data['routes']['ring_ccw'] = md['ring_ccw']

# ring_cw: need to transform from original
# Load original ring_cw and transform each point
import yaml as y
with open('src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml') as f:
    orig = y.safe_load(f.read())

ox, oy = -0.866, -0.648
res = 0.05
w, h = 115, 114

def to_new(x, y):
    col = (x - ox) / res
    row = h - 1 - (y - oy) / res
    new_col = w - 1 - col
    new_row = h - 1 - row
    return round(new_col * res, 6), round(new_row * res, 6)

ring_cw_new = []
for p in orig['routes']['ring_cw']:
    nx, ny = to_new(p['x'], p['y'])
    ring_cw_new.append({
        'x': nx, 'y': ny,
        'yaw': round(p['yaw'] + math.pi if p['yaw'] <= math.pi else p['yaw'] - math.pi, 6),
        'pause': p.get('pause', 0),
        'motion': p.get('motion', 'forward'),
        'pass_radius': p.get('pass_radius', 0.3),
        'reverse_pass_radius': p.get('reverse_pass_radius', 0.3),
    })
data['routes']['ring_cw'] = ring_cw_new

# return_to_p_rpp (new flipped)
data['routes']['return_to_p_rpp'] = md['return_to_p_rpp']

# Write
import copy
# Deep copy to avoid reference issues
output_path = 'src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml'
with open(output_path, 'w') as f:
    yaml.dump(data, f, indent=2, allow_unicode=True, sort_keys=False, width=120)

# Verify
with open(output_path) as f:
    loaded = y.safe_load(f)

r = loaded['routes']
ccw = r['ring_ccw']
cw = r['ring_cw']
ret_p = r['return_to_p_rpp']

print(f"[OK] YAML written: {output_path}")
print(f"ring_ccw: {len(ccw)} points")
print(f"ring_cw: {len(cw)} points (transformed)")
print(f"return_to_p_rpp: {len(ret_p)} points")
print(f"CW/CCW same object? {cw is ccw}")

gaps = [math.hypot(ccw[i]['x']-ccw[i-1]['x'], ccw[i]['y']-ccw[i-1]['y']) for i in range(1,len(ccw))]
print(f"ring_ccw max_gap: {max(gaps)*100:.1f}cm")
print(f"ring_ccw length: {sum(gaps):.3f}m")

max_dir = 0
for i in range(2, len(ccw)-2):
    a1 = math.atan2(ccw[i]['y']-ccw[i-1]['y'], ccw[i]['x']-ccw[i-1]['x'])
    a2 = math.atan2(ccw[i+1]['y']-ccw[i]['y'], ccw[i+1]['x']-ccw[i]['x'])
    d = abs(math.degrees(a2-a1))
    if d > 180: d = 360-d
    max_dir = max(max_dir, d)
print(f"ring_ccw max_dir: {max_dir:.1f}deg")

# Check join constraint
b = r['p_to_qr_backup_b_anchors'][3]
print(f"\nJoin: B anchor=({b['x']:.6f},{b['y']:.6f})")
print(f"  ring_ccw[0]=({ccw[0]['x']:.6f},{ccw[0]['y']:.6f}) d={math.hypot(b['x']-ccw[0]['x'],b['y']-ccw[0]['y']):.6f}")
print(f"  ring_cw[0]=({cw[0]['x']:.6f},{cw[0]['y']:.6f}) d={math.hypot(b['x']-cw[0]['x'],b['y']-cw[0]['y']):.6f}")

# Check cw gaps
cw_gaps = [math.hypot(cw[i]['x']-cw[i-1]['x'], cw[i]['y']-cw[i-1]['y']) for i in range(1,len(cw))]
print(f"\nring_cw max_gap: {max(cw_gaps)*100:.1f}cm (transformed from original)")

# Verify all keys present
print(f"\nRoutes: {list(r.keys())}")
print(f"Waypoints: {list(loaded['waypoints'].keys())}")
print(f"custom_rpp: {'present' if 'custom_rpp' in loaded else 'MISSING'}")

# Spot check: print first few ring_ccw points
print(f"\nFirst 3 ring_ccw points:")
for i in range(3):
    p = ccw[i]
    print(f"  [{i}] x={p['x']:.4f} y={p['y']:.4f} yaw={p['yaw']:.4f} motion={p['motion']}")

print(f"\nLast 3 ring_ccw points:")
for i in range(len(ccw)-3, len(ccw)):
    p = ccw[i]
    print(f"  [{i}] x={p['x']:.4f} y={p['y']:.4f} yaw={p['yaw']:.4f} motion={p['motion']}")

print("\n[DONE] Final flipped YAML ready!")
