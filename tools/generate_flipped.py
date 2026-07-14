#!/usr/bin/env python3
"""
Complete regeneration: flip map 180 degrees, embed in HTML,
regenerate all waypoints in new coordinate system.
"""
import base64, math, json

# ============================================================
# 1. Parse and flip the PGM maps
# ============================================================
def parse_pgm(path):
    with open(path, 'rb') as f:
        f.readline()  # P5
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b'#'): continue
            tokens.extend(line.strip().split())
        w, h, mv = int(tokens[0]), int(tokens[1]), int(tokens[2])
        return w, h, bytes(f.read())

ox, oy = -0.866, -0.648
res = 0.05
w, h = 115, 114

# Flip race_keepout
ww, hh, raw_keepout = parse_pgm('src/origincar_base/map/race_keepout.pgm')
flipped_keepout = bytearray(w * h)
for row in range(h):
    for col in range(w):
        flipped_keepout[(h-1-row)*w + (w-1-col)] = raw_keepout[row * w + col]

# Flip race_modify
ww2, hh2, raw_modify = parse_pgm('src/origincar_base/map/race_modify.pgm')
flipped_modify = bytearray(w * h)
for row in range(h):
    for col in range(w):
        flipped_modify[(h-1-row)*w + (w-1-col)] = raw_modify[row * w + col]

# Write flipped PGMs
for name, data in [('race_keepout_flipped.pgm', flipped_keepout),
                    ('race_modify_flipped.pgm', flipped_modify)]:
    with open(f'tools/{name}', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(bytes(data))
    print(f"Written: tools/{name}")

# Base64 encode both maps for embedding in HTML
b64_keepout = base64.b64encode(bytes(flipped_keepout)).decode('ascii')
b64_modify = base64.b64encode(bytes(flipped_modify)).decode('ascii')
print(f"Base64: keepout={len(b64_keepout)} chars, modify={len(b64_modify)} chars")

# ============================================================
# 2. Coordinate transform function
# ============================================================
def to_new(x, y):
    """Convert original world coords to flipped (bottom-left origin)."""
    col = (x - ox) / res
    row = h - 1 - (y - oy) / res
    new_col = w - 1 - col
    new_row = h - 1 - row
    return new_col * res, new_row * res

# New map params
NEW_RES = 0.05
NEW_W = 115
NEW_H = 114
# Origin is (0, 0) at bottom-left

# ============================================================
# 3. Key waypoints in new coordinate system
# ============================================================
P_X, P_Y = to_new(0.0, -0.071)
QR_POINT_X, QR_POINT_Y = to_new(3.2471, 0.9409)
QR_TURN_X, QR_TURN_Y = to_new(2.863253, 0.945718)
QR_BACKUP_X, QR_BACKUP_Y = to_new(2.65, 0.55)
B_ENTRANCE_X, B_ENTRANCE_Y = to_new(2.014753, 1.786151)
B_C_SIDE_X, B_C_SIDE_Y = to_new(1.744509, 2.63476)

print(f"\nKey points (new coords, bottom-left origin):")
print(f"  P:           ({P_X:.6f}, {P_Y:.6f})")
print(f"  QR sign:     ({QR_POINT_X:.6f}, {QR_POINT_Y:.6f})")
print(f"  QR turn:     ({QR_TURN_X:.6f}, {QR_TURN_Y:.6f})")
print(f"  QR backup:   ({QR_BACKUP_X:.6f}, {QR_BACKUP_Y:.6f})")
print(f"  B entrance:  ({B_ENTRANCE_X:.6f}, {B_ENTRANCE_Y:.6f})")
print(f"  B C-side:    ({B_C_SIDE_X:.6f}, {B_C_SIDE_Y:.6f})")

# ============================================================
# 4. Analyze flipped map corridors for C-zone path
# ============================================================
# Track layout (flipped, bottom-left origin):
# - Y=0 to 2.3: Open entry/exit area
# - Y=2.4 to 2.8: Narrow passage (cols~20-65)
# - Y=2.8 to 4.8: C-zone with central obstacle
#   - Left corridor: cols~2-18 (X≈0.1-0.9)
#   - Right corridor: cols~90-112 (X≈4.5-5.6)
#   - Obstacle: cols~23-85, rows~60-95 (in flipped image)
# - Y=4.8 to 5.7: Top open area where P sits

# CCW ring in flipped map:
# B entrance -> left corridor UP -> U-turn at top -> right corridor DOWN -> B entrance
#
# In flipped coords, "UP" means increasing Y, "DOWN" means decreasing Y
#
# Left corridor: X ≈ 0.5 (center-inside of cols 2-18)
# Right corridor: X ≈ 5.1 (center-inside of cols 90-112)

# Free-space analysis for corridor centerlines
def is_free_flipped(col, row):
    if col < 0 or col >= w or row < 0 or row >= h: return False
    return flipped_keepout[row * w + col] >= 128

print("\n=== C-zone corridor centerlines (flipped) ===")
print("\nLeft corridor (cols ~2-18):")
for row in range(50, 100, 2):
    if row >= h: break
    y = row * res
    # Find left free segment cols 0-40
    fs = fe = 0
    found = False
    for col in range(0, 40):
        if is_free_flipped(col, row):
            if not found:
                found = True
                fs = col
            fe = col
        elif found:
            break
    if found and fe - fs >= 4:
        center = fs + (fe - fs) * 0.55
        x = center * res
        print(f"  Y={y:.2f}: cols[{fs}-{fe}] center={center:.1f} X={x:.3f}")

print("\nRight corridor (cols ~90-112):")
for row in range(50, 100, 2):
    if row >= h: break
    y = row * res
    fs = fe = 0
    found = False
    for col in range(75, 115):
        if is_free_flipped(col, row):
            if not found:
                found = True
                fs = col
            fe = col
        elif found:
            break
    if found and fe - fs >= 4:
        center = fs + (fe - fs) * 0.45
        x = center * res
        print(f"  Y={y:.2f}: cols[{fs}-{fe}] center={center:.1f} X={x:.3f}")

# ============================================================
# 5. Generate C-zone ring_ccw in new coordinates
# ============================================================
print("\n=== Generating ring_ccw in new coordinates ===")

def catmull_rom_path(control_pts, spacing):
    if len(control_pts) < 4:
        result = []
        for i in range(len(control_pts)-1):
            seg = math.hypot(control_pts[i+1][0]-control_pts[i][0],
                             control_pts[i+1][1]-control_pts[i][1])
            steps = max(1, int(math.ceil(seg/spacing)))
            for j in range(steps):
                t = j/steps
                result.append((control_pts[i][0]+(control_pts[i+1][0]-control_pts[i][0])*t,
                               control_pts[i][1]+(control_pts[i+1][1]-control_pts[i][1])*t))
        result.append(control_pts[-1])
        return result

    result = []
    for i in range(len(control_pts)-1):
        p0 = control_pts[max(0,i-1)]
        p1 = control_pts[i]
        p2 = control_pts[i+1]
        p3 = control_pts[min(len(control_pts)-1,i+2)]
        seg_len = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        steps = max(1, int(math.ceil(seg_len/spacing)))
        for j in range(steps):
            t = j/steps
            t2, t3 = t*t, t*t*t
            x = 0.5*((2*p1[0])+(-p0[0]+p2[0])*t+
                     (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2+
                     (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
            y = 0.5*((2*p1[1])+(-p0[1]+p2[1])*t+
                     (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2+
                     (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
            result.append((x, y))
    result.append(control_pts[-1])
    return result

# CCW ring on flipped map:
# Start at B entrance -> go toward left corridor
# B entrance = (2.819, 2.434)
# Left corridor entry at Y~2.5, X~0.5
# Right corridor at Y~2.5, X~5.1

# Phase 1: B -> left corridor entry
phase1 = [
    (B_ENTRANCE_X, B_ENTRANCE_Y),   # B entrance
    (2.3, 2.5),   # transition toward left
    (1.2, 2.6),   # entering left side
    (0.55, 2.7),  # left corridor entry
]

# Phase 2: Left corridor straight UP
phase2 = [
    (0.55, 2.7),
    (0.52, 3.0),
    (0.50, 3.3),
    (0.48, 3.6),
    (0.47, 3.9),
    (0.46, 4.1),
]

# Phase 3: Top U-turn (left -> right)
phase3 = [
    (0.46, 4.1),
    (0.7, 4.35),
    (1.1, 4.55),
    (1.6, 4.70),
    (2.2, 4.78),
    (2.8, 4.80),
    (3.4, 4.76),
    (3.9, 4.65),
    (4.3, 4.48),
    (4.6, 4.25),
    (4.85, 4.0),
    (5.05, 3.8),
]

# Phase 4: Right corridor straight DOWN
phase4 = [
    (5.05, 3.8),
    (5.08, 3.5),
    (5.10, 3.2),
    (5.12, 2.9),
    (5.10, 2.6),
]

# Phase 5: Bottom return to B
phase5 = [
    (5.10, 2.6),
    (4.6, 2.45),
    (4.0, 2.38),
    (3.4, 2.40),
    (B_ENTRANCE_X, B_ENTRANCE_Y),
]

all_controls = phase1 + phase2[1:] + phase3[1:] + phase4[1:] + phase5[1:]
dense = catmull_rom_path(all_controls, 0.03)

print(f"Control points: {len(all_controls)}")
print(f"Dense path: {len(dense)} points")

# Compute yaw
pts_with_yaw = []
for i, (x, y) in enumerate(dense):
    if i < len(dense) - 1:
        yaw = math.atan2(dense[i+1][1]-y, dense[i+1][0]-x)
    elif i > 0:
        yaw = math.atan2(y-dense[i-1][1], x-dense[i-1][0])
    else:
        yaw = 0.0
    pts_with_yaw.append((x, y, yaw))

# Validate
max_dir = 0
for i in range(2, len(pts_with_yaw)-2):
    a1 = math.atan2(pts_with_yaw[i][1]-pts_with_yaw[i-1][1],
                    pts_with_yaw[i][0]-pts_with_yaw[i-1][0])
    a2 = math.atan2(pts_with_yaw[i+1][1]-pts_with_yaw[i][1],
                    pts_with_yaw[i+1][0]-pts_with_yaw[i][0])
    d = abs(math.degrees(a2-a1))
    if d > 180: d = 360-d
    max_dir = max(max_dir, d)

gaps = [math.hypot(pts_with_yaw[i][0]-pts_with_yaw[i-1][0],
                   pts_with_yaw[i][1]-pts_with_yaw[i-1][1])
        for i in range(1, len(pts_with_yaw))]
tot_len = sum(gaps)

print(f"Max direction change: {max_dir:.1f}deg")
print(f"Max gap: {max(gaps)*100:.1f}cm")
print(f"Min gap: {min(gaps)*100:.1f}cm")
print(f"Path length: {tot_len:.3f}m")
print(f"Points: {len(pts_with_yaw)}")

# Generate return_to_p_rpp in new coordinates
b_to_p_yaw = math.atan2(P_Y - B_ENTRANCE_Y, P_X - B_ENTRANCE_X)
b_to_p_dist = math.hypot(P_X - B_ENTRANCE_X, P_Y - B_ENTRANCE_Y)
ret_steps = max(1, int(math.ceil(b_to_p_dist / 0.03)))
ret_pts = []
for i in range(ret_steps + 1):
    t = i / ret_steps
    ret_pts.append((
        B_ENTRANCE_X + (P_X - B_ENTRANCE_X) * t,
        B_ENTRANCE_Y + (P_Y - B_ENTRANCE_Y) * t,
        b_to_p_yaw
    ))
print(f"\nreturn_to_p_rpp: {len(ret_pts)} points, {b_to_p_dist:.3f}m, yaw={math.degrees(b_to_p_yaw):.1f}deg")

# ============================================================
# 6. Save all data for the web tool
# ============================================================
map_data = {
    'w': w, 'h': h, 'res': NEW_RES,
    'keepout_b64': b64_keepout,
    'modify_b64': b64_modify,
    'waypoints': {
        'point_P': {'x': round(P_X,6), 'y': round(P_Y,6), 'yaw': 0.0},
        'qr_point': {'x': round(QR_POINT_X,6), 'y': round(QR_POINT_Y,6), 'yaw': 0.335969},
        'qr_turn_point': {'x': round(QR_TURN_X,6), 'y': round(QR_TURN_Y,6), 'yaw': 0.78},
        'qr_backup_point': {'x': round(QR_BACKUP_X,6), 'y': round(QR_BACKUP_Y,6), 'yaw': 2.05},
        'zone_b_a_side': {'x': round(B_ENTRANCE_X,6), 'y': round(B_ENTRANCE_Y,6), 'yaw': 1.645396},
        'zone_b_c_side': {'x': round(B_C_SIDE_X,6), 'y': round(B_C_SIDE_Y,6), 'yaw': 2.500253},
    },
    'ring_ccw': [{'x': round(x,6), 'y': round(y,6), 'yaw': round(yaw,6),
                  'pause':0,'motion':'forward','pass_radius':0.3,'reverse_pass_radius':0.3}
                 for x,y,yaw in pts_with_yaw],
    'return_to_p_rpp': [{'x': round(x,6), 'y': round(y,6), 'yaw': round(yaw,6),
                          'pause':0,'motion':'forward','pass_radius':0.3,'reverse_pass_radius':0.3}
                         for x,y,yaw in ret_pts],
}

with open('tools/map_data.json', 'w') as f:
    json.dump(map_data, f)
print(f"\nSaved map_data.json ({len(json.dumps(map_data))} bytes)")

# Also save as JS for embedding
with open('tools/map_data.js', 'w') as f:
    f.write(f'const MAP_DATA = {json.dumps(map_data)};\n')
print(f"Saved map_data.js")

print("\n[DONE] All data generated for flipped coordinate system")
PYEOF
