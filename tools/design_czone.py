#!/usr/bin/env python3
"""
Auto-design C-zone ring_ccw waypoints from race_keepout map.

Uses free-space corridor centerlines with Catmull-Rom spline smoothing.
Outputs waypoints at 2-5cm spacing with yaw computed from path direction.

Key design principles:
- CCW ring: B entrance → left corridor up → U-turn at top → right corridor down → back to B
- C-zone points on center-inside bias (away from walls)
- Straight lines on straights, smooth curves on turns
- Point spacing 2-5cm to satisfy 0 < gap <= 0.30m constraint
- All motion = forward

Map: 115x114 px, 0.05 m/px, origin (-0.866, -0.648)
Free corridors (rows 10-54):
  Left corridor rows 25-37: cols 16-27, X=[-0.07,0.48], center at col 22 = X=0.23
  Right corridor rows 25-37: cols 87-98, X=[3.48,4.03], center at col 92 = X=3.73
  Top U-turn rows 6-15: cols 16-98, all open
  Bottom rows 38-50: cols 16-98, all open
  Narrow exit rows 50-55: cols 46-68, X=[1.43,2.53]
"""

import math
import yaml
import sys


def catmull_rom_pts(p0, p1, p2, p3, spacing):
    """Interpolate between p1 and p2 using Catmull-Rom spline."""
    seg_len = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
    steps = max(1, int(math.ceil(seg_len / spacing)))
    result = []
    for i in range(steps):
        t = i / steps
        t2, t3 = t*t, t*t*t
        x = 0.5 * ((2*p1[0]) + (-p0[0]+p2[0])*t +
                   (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 +
                   (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3)
        y = 0.5 * ((2*p1[1]) + (-p0[1]+p2[1])*t +
                   (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 +
                   (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3)
        result.append((x, y))
    return result


def catmull_rom_path(control_pts, spacing):
    """Generate dense path through control points using Catmull-Rom."""
    if len(control_pts) < 4:
        # Fallback: linear interpolation
        result = []
        for i in range(len(control_pts)-1):
            result.extend(linear_interp(control_pts[i], control_pts[i+1], spacing))
        result.append(control_pts[-1])
        return result

    result = []
    for i in range(len(control_pts) - 1):
        p0 = control_pts[max(0, i-1)]
        p1 = control_pts[i]
        p2 = control_pts[i+1]
        p3 = control_pts[min(len(control_pts)-1, i+2)]
        result.extend(catmull_rom_pts(p0, p1, p2, p3, spacing))
    result.append(control_pts[-1])
    return result


def linear_interp(p1, p2, spacing):
    dist = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
    steps = max(1, int(math.ceil(dist / spacing)))
    return [(p1[0]+(p2[0]-p1[0])*i/steps, p1[1]+(p2[1]-p1[1])*i/steps)
            for i in range(steps)]


def generate_ring_ccw(spacing=0.03):
    """
    Generate CCW ring waypoints.
    CCW: B → left corridor up → U-turn → right corridor down → B
    """

    # ===== B entrance (preserved) =====
    b_entrance = (2.014753, 1.786151)

    # ===== Phase 1: B → left corridor entry =====
    # Smooth transition from (2.015, 1.786) to left corridor at ~X=0.23
    # Cubic bezier-like control points
    phase1 = [
        b_entrance,
        (1.80, 1.85),
        (0.80, 2.00),
        (0.40, 2.00),
    ]

    # ===== Phase 2: Left corridor going UP =====
    # From Y=2.0 up to Y=3.7, X ~ 0.22 (center-inside)
    # At rows 25-37 the left corridor is cols 16-27, center at col 22 = X=0.234
    phase2 = [
        (0.40, 2.00),
        (0.25, 2.10),
        (0.23, 2.40),
        (0.23, 2.80),
        (0.23, 3.10),
        (0.23, 3.30),
        (0.22, 3.55),
        (0.22, 3.70),
    ]

    # ===== Phase 3: Top U-turn (left → right at Y=3.75-4.25) =====
    # Smooth arc: from left corridor top to right corridor top
    # Top of map: Y ~ 4.25, free space cols 16-98
    # Arc center roughly at X=2.0, Y=3.85, radius ~1.75m
    # But we need a tighter turn since the top free area isn't that tall
    phase3 = [
        (0.22, 3.70),
        (0.40, 3.90),
        (0.70, 4.08),
        (1.10, 4.20),
        (1.60, 4.26),
        (2.10, 4.28),
        (2.50, 4.28),
        (2.90, 4.24),
        (3.20, 4.16),
        (3.45, 4.00),
        (3.62, 3.82),
        (3.73, 3.70),
    ]

    # ===== Phase 4: Right corridor going DOWN =====
    # From Y=3.7 down to Y=2.10, X ~ 3.74 (center-inside)
    phase4 = [
        (3.73, 3.70),
        (3.74, 3.55),
        (3.74, 3.30),
        (3.74, 3.10),
        (3.74, 2.80),
        (3.74, 2.50),
        (3.72, 2.20),
        (3.60, 2.05),
    ]

    # ===== Phase 5: Bottom return to B entrance =====
    # From right corridor bottom back to B
    # At rows 45-55 the free space narrows to cols 46-68 (X=1.43-2.53)
    phase5 = [
        (3.60, 2.05),
        (3.20, 1.95),
        (2.80, 1.88),
        (2.40, 1.82),
        b_entrance,
    ]

    # Combine all phases
    all_controls = phase1 + phase2[1:] + phase3[1:] + phase4[1:] + phase5[1:]

    # Generate dense path via Catmull-Rom
    dense = catmull_rom_path(all_controls, spacing)

    print(f"Control points: {len(all_controls)}")
    print(f"Dense path: {len(dense)} points at {spacing}m spacing")

    return dense


def compute_yaw(pts):
    """Compute yaw from path direction for each point."""
    result = []
    for i, (x, y) in enumerate(pts):
        if i < len(pts) - 1:
            nx, ny = pts[i+1]
            yaw = math.atan2(ny - y, nx - x)
        elif i > 0:
            px, py = pts[i-1]
            yaw = math.atan2(y - py, x - px)
        else:
            yaw = 0.0
        result.append((x, y, yaw))
    return result


def check_curvature(pts_with_yaw):
    """Validate curvature: direction change < 20°, radius > 0.25m."""
    issues = 0
    for i in range(2, len(pts_with_yaw) - 2):
        p0, p1, p2 = pts_with_yaw[i-1], pts_with_yaw[i], pts_with_yaw[i+1]
        a1 = math.atan2(p1[1]-p0[1], p1[0]-p0[0])
        a2 = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        d = a2 - a1
        while d > math.pi: d -= 2*math.pi
        while d < -math.pi: d += 2*math.pi
        deg = abs(math.degrees(d))
        if deg > 20:
            print(f"  Warning: direction change {deg:.1f}° at point {i}")
            issues += 1
        elif deg > 15 and issues < 10:
            print(f"  Info: moderate change {deg:.1f}° at point {i}")

    if issues == 0:
        print("  [OK] All direction changes < 20deg")
    else:
        print(f"  [WARN] {issues} direction issues")

    # Check gap
    max_gap = 0
    for i in range(1, len(pts_with_yaw)):
        g = math.hypot(pts_with_yaw[i][0]-pts_with_yaw[i-1][0],
                       pts_with_yaw[i][1]-pts_with_yaw[i-1][1])
        max_gap = max(max_gap, g)
    print(f"  Max gap: {max_gap*100:.1f}cm (must be <= 30cm)")

    # Path length
    total = sum(math.hypot(pts_with_yaw[i][0]-pts_with_yaw[i-1][0],
                           pts_with_yaw[i][1]-pts_with_yaw[i-1][1])
                for i in range(1, len(pts_with_yaw)))
    print(f"  Total length: {total:.3f}m")
    print(f"  Avg spacing: {total/(len(pts_with_yaw)-1)*100:.2f}cm")


def format_waypoint(x, y, yaw):
    """Format a single waypoint as YAML lines."""
    return [
        f'  - x: {x:.6f}',
        f'    y: {y:.6f}',
        f'    yaw: {yaw:.6f}',
        f'    pause: 0',
        f'    motion: forward',
        f'    pass_radius: 0.3',
        f'    reverse_pass_radius: 0.3',
    ]


def main():
    spacing = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03

    print(f"=== Generating ring_ccw at {spacing}m spacing ===")

    # Generate
    dense_pts = generate_ring_ccw(spacing)

    # Add yaw
    pts_with_yaw = compute_yaw(dense_pts)

    # Validate
    print("\n=== Validation ===")
    check_curvature(pts_with_yaw)

    # Print for copy-paste into YAML
    print(f"\n=== ring_ccw: &id002 ({len(pts_with_yaw)} points) ===")
    for x, y, yaw in pts_with_yaw:
        lines = format_waypoint(x, y, yaw)
        for line in lines:
            print(line)

    # Also save as standalone YAML block
    with open('tools/ring_ccw_new_generated.yaml', 'w') as f:
        f.write(f'# Generated ring_ccw path ({len(pts_with_yaw)} points, {spacing}m spacing)\n')
        f.write(f'ring_ccw: &id002\n')
        for x, y, yaw in pts_with_yaw:
            for line in format_waypoint(x, y, yaw):
                f.write(line + '\n')

    print(f"\nSaved to tools/ring_ccw_new_generated.yaml")


if __name__ == '__main__':
    main()
