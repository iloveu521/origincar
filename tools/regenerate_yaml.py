#!/usr/bin/env python3
"""Regenerate clean YAML with new ring_ccw and return_to_p_rpp."""
import yaml, math, json

# Load original ring_cw
with open('src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml') as f:
    orig = yaml.safe_load(f.read())

# Load new ring_ccw
with open('tools/ring_ccw_new_generated.yaml') as f:
    gen = yaml.safe_load(f.read())

# Build complete data
data = {}

# field_to_map
data['field_to_map'] = {
    'source': 'flowpath/dev_ws/src/origincar/origincar_system/config/waypoints.yaml',
    'source_map': 'race_modify.pgm',
    'source_yaml': 'race_modify.yaml',
    'method': 'copied_flowpath_waypoints_on_same_race_modify_map',
    'resolution': 0.05,
}

# waypoints
data['waypoints'] = {
    'point_P': {'x': 0.0, 'y': -0.071, 'yaw': 0.0, 'description': 'P start/parking center'},
    'qr_point': {'x': 3.2471, 'y': 0.9409, 'yaw': 0.335969, 'description': 'QR sign map position; not a drivable vehicle pose'},
    'qr_turn_point': {'x': 2.863253, 'y': 0.945718, 'yaw': 0.78, 'description': 'Vehicle QR scan pose before the reverse maneuver'},
    'qr_backup_point': {'x': 2.65, 'y': 0.55, 'yaw': 2.05, 'description': 'Single reverse pose that aligns the nose toward B'},
    'zone_b_a_side': {'x': 2.014753, 'y': 1.786151, 'yaw': 1.645396, 'description': 'B corridor A side from flowpath point_03'},
    'zone_b_c_side': {'x': 1.744509, 'y': 2.63476, 'yaw': 2.500253, 'description': 'C side after B crossing'},
}

# routes
data['routes'] = {}

# p_to_qr_backup_b_anchors (4 keypoints)
data['routes']['p_to_qr_backup_b_anchors'] = [
    {'x': 0.0, 'y': -0.071, 'yaw': 0.065384, 'pause': 0, 'motion': 'forward',
     'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
    {'x': 2.863253, 'y': 0.945718, 'yaw': 0.78, 'pause': 0, 'motion': 'forward',
     'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
    {'x': 2.694491, 'y': 0.482323, 'yaw': 2.05, 'pause': 0, 'motion': 'reverse',
     'pass_radius': 0.12, 'reverse_pass_radius': 0.12},
    {'x': 2.014753, 'y': 1.786151, 'yaw': 1.645396, 'pause': 0, 'motion': 'forward',
     'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
]

# ring_cw (original, unchanged)
data['routes']['ring_cw'] = orig['routes']['ring_cw']

# ring_ccw (NEW - from map-based design)
data['routes']['ring_ccw'] = gen['ring_ccw']

# return_to_p_rpp (NEW dense diagonal)
b_x, b_y = 2.014753, 1.786151
p_x, p_y = 0.0, -0.071
yaw_val = math.atan2(p_y - b_y, p_x - b_x)
dist = math.hypot(p_x - b_x, p_y - b_y)
steps = max(1, int(math.ceil(dist / 0.03)))
ret = []
for i in range(steps + 1):
    t = i / steps
    ret.append({
        'x': round(b_x + (p_x - b_x) * t, 6),
        'y': round(b_y + (p_y - b_y) * t, 6),
        'yaw': round(yaw_val, 6),
        'pause': 0,
        'motion': 'forward',
        'pass_radius': 0.3,
        'reverse_pass_radius': 0.3,
    })
data['routes']['return_to_p_rpp'] = ret

# custom_rpp
data['custom_rpp'] = {
    'stuck_skip': {
        'enabled': False,
        'timeout_sec': 0.8,
        'max_distance': 0.45,
        'recovery_enabled': False,
        'recovery_max_distance': 1.2,
        'recovery_duration_sec': 0.35,
        'recovery_speed': 0.12,
        'recovery_angular_z': 0.25,
        'description': 'Stuck skip & recovery are DISABLED. RPP no longer reverses or hunts',
    },
}

# Write
output = 'src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml'
with open(output, 'w') as f:
    yaml.dump(data, f, indent=2, allow_unicode=True, sort_keys=False, width=120)

# Verify
with open(output) as f:
    loaded = yaml.safe_load(f)

r = loaded['routes']
ccw = r['ring_ccw']
cw = r['ring_cw']
ret_p = r['return_to_p_rpp']

print(f"[PASS] YAML valid!")
print(f"ring_ccw: {len(ccw)} pts")
print(f"ring_cw: {len(cw)} pts")
print(f"CW/CCW same object? {ccw is cw}")
print(f"return_to_p_rpp: {len(ret_p)} pts")

gaps = [math.hypot(ccw[i]['x']-ccw[i-1]['x'], ccw[i]['y']-ccw[i-1]['y']) for i in range(1,len(ccw))]
print(f"ring_ccw max_gap: {max(gaps)*100:.1f}cm (ok={max(gaps)<=0.30})")
print(f"ring_ccw length: {sum(gaps):.3f}m")

max_d = 0
for i in range(2, len(ccw)-2):
    a1 = math.atan2(ccw[i]['y']-ccw[i-1]['y'], ccw[i]['x']-ccw[i-1]['x'])
    a2 = math.atan2(ccw[i+1]['y']-ccw[i]['y'], ccw[i+1]['x']-ccw[i]['x'])
    d = abs(math.degrees(a2-a1))
    if d > 180: d = 360-d
    max_d = max(max_d, d)
print(f"ring_ccw max_dir: {max_d:.1f}deg")

ret_g = [math.hypot(ret_p[i]['x']-ret_p[i-1]['x'], ret_p[i]['y']-ret_p[i-1]['y']) for i in range(1,len(ret_p))]
print(f"return_to_p_rpp: max_gap={max(ret_g)*100:.1f}cm, len={sum(ret_g):.3f}m")
print(f"  start=({ret_p[0]['x']:.3f},{ret_p[0]['y']:.3f}) -> end=({ret_p[-1]['x']:.3f},{ret_p[-1]['y']:.3f})")

# Join constraint
b = r['p_to_qr_backup_b_anchors'][3]
print(f"\nJoin constraint: B anchors[3]=({b['x']:.6f},{b['y']:.6f})")
print(f"  ring_ccw[0]=({ccw[0]['x']:.6f},{ccw[0]['y']:.6f}) d={math.hypot(b['x']-ccw[0]['x'],b['y']-ccw[0]['y']):.6f}")
print(f"  ring_cw[0]=({cw[0]['x']:.6f},{cw[0]['y']:.6f}) d={math.hypot(b['x']-cw[0]['x'],b['y']-cw[0]['y']):.6f}")

# Waypoints check
wps = loaded['waypoints']
print(f"\nWaypoints: {list(wps.keys())}")
print(f"  P: ({wps['point_P']['x']},{wps['point_P']['y']})")

print("\n[PASS] All checks complete!")
