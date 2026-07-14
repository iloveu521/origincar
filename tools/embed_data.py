#!/usr/bin/env python3
"""Embed map data and YAML into the HTML editor."""
import base64

# Read maps
def read_pgm(path):
    with open(path, 'rb') as f:
        f.readline()  # P5
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b'#'): continue
            tokens.extend(line.strip().split())
        w, h, mv = int(tokens[0]), int(tokens[1]), int(tokens[2])
        return w, h, f.read()

w, h, mraw = read_pgm('src/origincar_base/map/race_modify.pgm')
_, _, kraw = read_pgm('src/origincar_base/map/race_keepout.pgm')

b64m = base64.b64encode(mraw).decode('ascii')
b64k = base64.b64encode(kraw).decode('ascii')
print(f"Maps: {w}x{h}, modify_b64={len(b64m)}, keepout_b64={len(b64k)}")

# Read YAML
with open('src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml', 'r', encoding='utf-8') as f:
    yaml_text = f.read()

# Escape: backslash, backtick, dollar for JS template literal
yaml_escaped = yaml_text.replace('\\', '\\\\').replace('`', '\\`').replace('$', '\\$')

# Read HTML
with open('tools/map_waypoint_designer.html', 'r', encoding='utf-8') as f:
    html = f.read()

# Replace placeholders
old_rpd = 'const RPD = {"modify_b64":"","keepout_b64":"","w":115,"h":114};'
new_rpd = f'const RPD = {{"modify_b64":"{b64m}","keepout_b64":"{b64k}","w":{w},"h":{h}}};'
html = html.replace(old_rpd, new_rpd)

old_yaml = 'const DEFAULT_YAML_TEXT = ``;'
new_yaml = f'const DEFAULT_YAML_TEXT = `{yaml_escaped}`;'
html = html.replace(old_yaml, new_yaml)

# Write
with open('tools/map_waypoint_designer.html', 'w', encoding='utf-8') as f:
    f.write(html)

print(f"HTML written: {len(html)} chars")
print("[OK] Embedded map data + YAML into HTML!")
