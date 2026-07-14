#!/usr/bin/env python3
"""
Smooth ring_ccw waypoints using Catmull-Rom spline interpolation.
Modifies the original YAML file in place (with backup).

The core algorithm:
1. Parse the original YAML
2. Identify curvature problem areas in ring_ccw
3. Smooth those areas using Catmull-Rom spline with dense interpolation
4. Write the modified YAML
"""

import math
import yaml
import shutil
import os


def direction_change_deg(p1, p2, p3):
    a1 = math.atan2(p2['y'] - p1['y'], p2['x'] - p1['x'])
    a2 = math.atan2(p3['y'] - p2['y'], p3['x'] - p2['x'])
    d = a2 - a1
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return abs(math.degrees(d))


def menger_radius(p1, p2, p3):
    a = math.hypot(p2['x']-p1['x'], p2['y']-p1['y'])
    b = math.hypot(p3['x']-p2['x'], p3['y']-p2['y'])
    c = math.hypot(p3['x']-p1['x'], p3['y']-p1['y'])
    area = 0.5*abs(p1['x']*(p2['y']-p3['y'])+p2['x']*(p3['y']-p1['y'])+p3['x']*(p1['y']-p2['y']))
    if area < 1e-12: return float('inf')
    return (a*b*c)/(4*area)


def catmull_rom(p0, p1, p2, p3, t):
    t2, t3 = t*t, t*t*t
    x = 0.5*((2*p1['x'])+(-p0['x']+p2['x'])*t+(2*p0['x']-5*p1['x']+4*p2['x']-p3['x'])*t2+(-p0['x']+3*p1['x']-3*p2['x']+p3['x'])*t3)
    y = 0.5*((2*p1['y'])+(-p0['y']+p2['y'])*t+(2*p0['y']-5*p1['y']+4*p2['y']-p3['y'])*t2+(-p0['y']+3*p1['y']-3*p2['y']+p3['y'])*t3)
    return x, y


def path_yaw(p1, p2):
    return math.atan2(p2['y']-p1['y'], p2['x']-p1['x'])


def smooth_segment(pts, spacing):
    """Smooth a segment using Catmull-Rom spline."""
    if len(pts) < 4:
        return [dict(p) for p in pts]

    result = []
    for i in range(len(pts)-1):
        p0 = pts[max(0,i-1)]
        p1 = pts[i]
        p2 = pts[i+1]
        p3 = pts[min(len(pts)-1,i+2)]
        seg_len = math.hypot(p2['x']-p1['x'], p2['y']-p1['y'])
        steps = max(1, int(math.ceil(seg_len/spacing)))
        for j in range(steps):
            t = j/steps
            x, y = catmull_rom(p0, p1, p2, p3, t)
            result.append({'x': round(x,6), 'y': round(y,6)})
    last = pts[-1]
    result.append({'x': round(last['x'],6), 'y': round(last['y'],6)})
    return result


def smooth_ring_ccw(ring_ccw, spacing=0.03):
    """Main smoothing function."""
    n = len(ring_ccw)
    print(f"Input: {n} points")

    # 1. Find all problem points
    angle_th = 15.0
    radius_th = 0.35
    problems = set()
    for i in range(2, n-2):
        d = direction_change_deg(ring_ccw[i-1], ring_ccw[i], ring_ccw[i+1])
        r = menger_radius(ring_ccw[i-1], ring_ccw[i], ring_ccw[i+1])
        if d > angle_th or r < radius_th:
            problems.add(i)

    print(f"Problem indices: {sorted(problems)}")

    # 2. Group into regions
    si = sorted(problems)
    regions = []
    if si:
        rs = re = si[0]
        for idx in si[1:]:
            if idx - re <= 5:
                re = idx
            else:
                regions.append((rs, re))
                rs = re = idx
        regions.append((rs, re))

    expand = 3
    print(f"Regions to smooth (expand={expand} each side):")
    for rs, re in regions:
        s = max(0, rs-expand)
        e = min(n-1, re+expand)
        print(f"  [{rs},{re}] -> smooth [{s},{e}]  ({re-rs+1}->{e-s+1} core pts)")

    # Merge overlapping expanded regions
    expanded = [(max(0, rs-expand), min(n-1, re+expand)) for rs, re in regions]
    merged = []
    for s, e in expanded:
        if merged and s <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], e))
        else:
            merged.append((s, e))

    print(f"Merged regions: {merged}")

    # 3. Build new list
    new_pts = []
    last_e = 0

    for s, e in merged:
        mid = (s + e) // 2

        # Keep good points
        new_pts.extend([dict(p) for p in ring_ccw[last_e:s]])

        # Smooth problem region
        template = ring_ccw[mid]
        seg_pts = ring_ccw[s:e+1]
        seg = smooth_segment(seg_pts, spacing)

        # Compute yaw from path direction
        for i in range(len(seg)):
            if i < len(seg)-1:
                seg[i]['yaw'] = round(path_yaw(seg[i], seg[i+1]), 6)
            elif i > 0:
                seg[i]['yaw'] = round(path_yaw(seg[i-1], seg[i]), 6)

        # Apply metadata
        for pt in seg:
            pt['pause'] = template.get('pause', 0)
            pt['motion'] = template.get('motion', 'forward')
            pt['pass_radius'] = template.get('pass_radius', 0.3)
            pt['reverse_pass_radius'] = template.get('reverse_pass_radius', 0.3)
        new_pts.extend(seg)

        last_e = e + 1

    # Keep trailing good points
    new_pts.extend([dict(p) for p in ring_ccw[last_e:]])

    print(f"Output: {len(new_pts)} points")

    # 4. Verify
    issues = 0
    for i in range(2, len(new_pts)-2):
        d = direction_change_deg(new_pts[i-1], new_pts[i], new_pts[i+1])
        if d > 20:
            if issues < 10:
                print(f"  Direction issue at {i}: {d:.1f}deg")
            issues += 1

    # Check radius with wider spacing to avoid measurement noise
    radius_issues = 0
    step = max(1, int(0.05 / spacing))  # ~5cm step for radius check
    for i in range(step*2, len(new_pts)-step*2, step):
        r = menger_radius(new_pts[i-step], new_pts[i], new_pts[i+step])
        if r < 0.25:
            radius_issues += 1

    if issues == 0:
        print("Direction check: [OK] All direction changes < 20deg")
    else:
        print(f"Direction check: [WARN] {issues} issues")

    if radius_issues == 0:
        print("Radius check: [OK] All turning radii > 0.25m")
    else:
        print(f"Radius check: [WARN] {radius_issues} tight spots")

    return new_pts


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('input', nargs='?', default='src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml')
    p.add_argument('-o','--output', default=None)
    p.add_argument('-s','--spacing', type=float, default=0.03)
    args = p.parse_args()

    output = args.output or args.input.replace('.yaml', '_fixed.yaml')

    # Backup
    if args.output is None:
        shutil.copy2(args.input, args.input + '.bak')
        print(f"Backup: {args.input}.bak")

    with open(args.input) as f:
        data = yaml.safe_load(f)

    ring_ccw = data['routes']['ring_ccw']
    new_ccw = smooth_ring_ccw(ring_ccw, args.spacing)

    # Update data - note: ring_cw and ring_ccw use YAML anchors
    # In the parsed data, routes dict keys might appear multiple times due to anchors
    # We need to update all references
    data['routes']['ring_ccw'] = new_ccw

    # Write output
    with open(output, 'w') as f:
        yaml.dump(data, f, indent=2, allow_unicode=True, sort_keys=False, width=120)

    print(f"\nSaved: {output}")

    # Print total path length comparison
    old_len = sum(math.hypot(
        ring_ccw[i]['x']-ring_ccw[i-1]['x'],
        ring_ccw[i]['y']-ring_ccw[i-1]['y']
    ) for i in range(1, len(ring_ccw)))
    new_len = sum(math.hypot(
        new_ccw[i]['x']-new_ccw[i-1]['x'],
        new_ccw[i]['y']-new_ccw[i-1]['y']
    ) for i in range(1, len(new_ccw)))
    print(f"Path length: {old_len:.3f}m -> {new_len:.3f}m (delta: {new_len-old_len:+.3f}m)")


if __name__ == '__main__':
    main()
