#!/usr/bin/env python3
"""
Generate the final waypoint design:
- Original coordinates (origin at [-0.866, -0.648], top-left SVG origin)
- 5cm point spacing
- ring_ccw: smooth corridor-based C-zone path
- return_to_p_rpp: B -> P direct (single point)
- ring_cw preserved from original
- reference tool map overlay style

Coordinate reference (same as reference tool):
  SVG X -> world X = px*0.05 - 0.866
  SVG Y -> world Y = (114-py)*0.05 - 0.648
  Map is drawn at pixel coordinates: <image x="0" y="0" width="115" height="114"/>
"""
import math, json
import yaml as yaml_lib

ox, oy = -0.866, -0.648
res = 0.05
w_img, h_img = 115, 114

B_x, B_y = 2.014753, 1.786151
P_x, P_y = 0.0, -0.071

# ============================================================
# Catmull-Rom spline
# ============================================================
def catmull_rom_path(controls, spacing):
    if len(controls) < 4:
        result = []
        for i in range(len(controls)-1):
            d = math.hypot(controls[i+1][0]-controls[i][0],
                           controls[i+1][1]-controls[i][1])
            st = max(1, int(math.ceil(d/spacing)))
            for j in range(st):
                t = j/st
                result.append((controls[i][0]+(controls[i+1][0]-controls[i][0])*t,
                              controls[i][1]+(controls[i+1][1]-controls[i][1])*t))
        result.append(controls[-1])
        return result

    result = []
    for i in range(len(controls)-1):
        p0 = controls[max(0,i-1)]
        p1 = controls[i]
        p2 = controls[i+1]
        p3 = controls[min(len(controls)-1,i+2)]
        seg = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        st = max(1, int(math.ceil(seg/spacing)))
        for j in range(st):
            t = j/st; t2=t*t; t3=t2*t
            x = 0.5*((2*p1[0])+(-p0[0]+p2[0])*t+
                     (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2+
                     (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
            y = 0.5*((2*p1[1])+(-p0[1]+p2[1])*t+
                     (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2+
                     (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
            result.append((x, y))
    result.append(controls[-1])
    return result

# ============================================================
# ring_ccw design (CCW: B -> left corridor up -> U-turn -> right corridor down -> B)
# ============================================================
phase1 = [(B_x, B_y), (1.80, 1.82), (1.10, 1.88), (0.50, 1.97), (0.30, 2.08)]
phase2 = [(0.30, 2.08), (0.24, 2.35), (0.22, 2.65), (0.22, 2.95),
          (0.22, 3.25), (0.22, 3.48), (0.21, 3.68), (0.20, 3.83)]
phase3 = [(0.20, 3.83), (0.30, 4.03), (0.58, 4.22), (0.95, 4.37),
          (1.45, 4.46), (2.05, 4.49), (2.65, 4.46), (3.18, 4.37),
          (3.52, 4.20), (3.68, 3.98), (3.75, 3.78)]
phase4 = [(3.75, 3.78), (3.74, 3.50), (3.74, 3.20), (3.74, 2.90),
          (3.73, 2.60), (3.72, 2.30), (3.68, 2.05)]
phase5 = [(3.68, 2.05), (3.15, 1.92), (2.55, 1.84), (B_x, B_y)]

all_ctrl = phase1 + phase2[1:] + phase3[1:] + phase4[1:] + phase5[1:]
dense = catmull_rom_path(all_ctrl, 0.05)

# Compute yaw from path direction
pts_with_yaw = []
for i, (x, y) in enumerate(dense):
    if i < len(dense)-1:
        yaw = math.atan2(dense[i+1][1]-y, dense[i+1][0]-x)
    elif i > 0:
        yaw = math.atan2(y-dense[i-1][1], x-dense[i-1][0])
    else:
        yaw = 0.0
    pts_with_yaw.append((x, y, yaw))

# Validate
gaps = [math.hypot(pts_with_yaw[i][0]-pts_with_yaw[i-1][0],
                   pts_with_yaw[i][1]-pts_with_yaw[i-1][1])
        for i in range(1, len(pts_with_yaw))]

max_dir = 0
for i in range(2, len(pts_with_yaw)-2):
    a1 = math.atan2(pts_with_yaw[i][1]-pts_with_yaw[i-1][1],
                    pts_with_yaw[i][0]-pts_with_yaw[i-1][0])
    a2 = math.atan2(pts_with_yaw[i+1][1]-pts_with_yaw[i][1],
                    pts_with_yaw[i+1][0]-pts_with_yaw[i][0])
    d = a2 - a1
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    max_dir = max(max_dir, abs(math.degrees(d)))

print(f"ring_ccw: {len(pts_with_yaw)} pts, len={sum(gaps):.3f}m, "
      f"gap={max(gaps)*100:.1f}/{min(gaps)*100:.1f}cm, dir={max_dir:.1f}deg")
print(f"  first=({pts_with_yaw[0][0]:.6f},{pts_with_yaw[0][1]:.6f})")
print(f"  last =({pts_with_yaw[-1][0]:.6f},{pts_with_yaw[-1][1]:.6f})")

# ============================================================
# return_to_p_rpp: B -> P direct (single point)
# ============================================================
b_to_p_yaw = math.atan2(P_y - B_y, P_x - B_x)
ret = [
    {'x': B_x, 'y': B_y, 'yaw': b_to_p_yaw},
    {'x': P_x, 'y': P_y, 'yaw': b_to_p_yaw},
]
print(f"return_to_p_rpp: 2 pts, yaw={math.degrees(b_to_p_yaw):.1f}deg")

# ============================================================
# Build complete YAML
# ============================================================
with open('src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml') as f:
    orig = yaml_lib.safe_load(f.read())

data = {
    'field_to_map': orig.get('field_to_map', {}),
    'waypoints': {
        'point_P': {'x': 0.0, 'y': -0.071, 'yaw': 0.0,
                    'description': 'P start/parking center'},
        'qr_point': {'x': 3.2471, 'y': 0.9409, 'yaw': 0.335969,
                     'description': 'QR sign map position'},
        'qr_turn_point': {'x': 2.863253, 'y': 0.945718, 'yaw': 0.78,
                          'description': 'Vehicle QR scan pose'},
        'qr_backup_point': {'x': 2.65, 'y': 0.55, 'yaw': 2.05,
                            'description': 'Single reverse pose'},
        'zone_b_a_side': {'x': B_x, 'y': B_y, 'yaw': 1.645396,
                          'description': 'B corridor A side'},
        'zone_b_c_side': {'x': 1.744509, 'y': 2.63476, 'yaw': 2.500253,
                          'description': 'C side after B crossing'},
    },
    'routes': {},
    'custom_rpp': orig.get('custom_rpp', {}),
}

# p_to_qr_backup_b_anchors
data['routes']['p_to_qr_backup_b_anchors'] = [
    {'x': 0.0, 'y': -0.071, 'yaw': 0.065384, 'pause': 0, 'motion': 'forward',
     'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
    {'x': 2.863253, 'y': 0.945718, 'yaw': 0.78, 'pause': 0, 'motion': 'forward',
     'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
    {'x': 2.694491, 'y': 0.482323, 'yaw': 2.05, 'pause': 0, 'motion': 'reverse',
     'pass_radius': 0.12, 'reverse_pass_radius': 0.12},
    {'x': B_x, 'y': B_y, 'yaw': 1.645396, 'pause': 0, 'motion': 'forward',
     'pass_radius': 0.3, 'reverse_pass_radius': 0.3},
]

# ring_ccw (new)
data['routes']['ring_ccw'] = [{
    'x': round(x, 6), 'y': round(y, 6), 'yaw': round(yaw, 6),
    'pause': 0, 'motion': 'forward', 'pass_radius': 0.3, 'reverse_pass_radius': 0.3,
} for x, y, yaw in pts_with_yaw]

# ring_cw (preserved from original)
data['routes']['ring_cw'] = orig['routes']['ring_cw']

# p_to_qr_to_b_rpp_legacy (preserved)
if 'p_to_qr_to_b_rpp_legacy' in orig.get('routes', {}):
    data['routes']['p_to_qr_to_b_rpp_legacy'] = orig['routes']['p_to_qr_to_b_rpp_legacy']

# return_to_p_rpp (new: B->P direct)
data['routes']['return_to_p_rpp'] = [{
    'x': round(p['x'], 6), 'y': round(p['y'], 6), 'yaw': round(p['yaw'], 6),
    'pause': 0, 'motion': 'forward', 'pass_radius': 0.3, 'reverse_pass_radius': 0.3,
} for p in ret]

# Write
output = 'src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml'
with open(output, 'w') as f:
    yaml_lib.dump(data, f, indent=2, allow_unicode=True, sort_keys=False, width=120)

# Verify
with open(output) as f:
    loaded = yaml_lib.safe_load(f)
r = loaded['routes']
ccw = r['ring_ccw']
print(f"\n[Final] ring_ccw={len(ccw)}, ring_cw={len(r['ring_cw'])}, ret={len(r['return_to_p_rpp'])}")
print(f"B join: {math.hypot(r['p_to_qr_backup_b_anchors'][3]['x']-ccw[0]['x'], r['p_to_qr_backup_b_anchors'][3]['y']-ccw[0]['y']):.6f}")
print("[OK] YAML written")
