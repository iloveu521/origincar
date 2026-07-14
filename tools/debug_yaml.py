#!/usr/bin/env python3
"""Debug the HTML YAML embedding."""
with open('tools/map_waypoint_designer.html', 'r', encoding='utf-8') as f:
    h = f.read()

idx = h.find('DEFAULT_YAML_TEXT = ')
print(f'DEFAULT_YAML_TEXT at char {idx}')

# Find closing backtick-semicolon
end = h.find('`;', idx + 30)
print(f'Closing at char {end}')
if end > 0:
    print(f'End context: {repr(h[end-30:end+10])}')

# Count backticks
bt = h.count('`')
print(f'Total backticks: {bt}')

# Find all `; occurrences
import re
matches = [(m.start(), m.group()) for m in re.finditer(r'`;', h)]
print(f'Backtick-semicolon count: {len(matches)}')
for pos, _ in matches[:5]:
    print(f'  {pos}: {repr(h[max(0,pos-10):pos+10])}')

# Check the YAML section specifically
yaml_start = idx  # finds the start of definition
# Find the opening backtick
obt = h.find('`', idx)
print(f'Opening backtick at {obt}: {repr(h[obt:obt+30])}')
# Find closing
cbt = h.find('`;', obt + 10)
print(f'Closing backtick-semicolon at {cbt}')
if cbt > 0:
    yaml_inner = h[obt+1:cbt]
    print(f'YAML content length: {len(yaml_inner)}')
    print(f'YAML starts: {repr(yaml_inner[:80])}')
    print(f'YAML ends: {repr(yaml_inner[-80:])}')
    # Check for unescaped backticks inside
    inner_bt = yaml_inner.count('`')
    print(f'Backticks inside YAML: {inner_bt}')
    inner_ds = yaml_inner.count('${')
    print(f'Dollar-brace inside YAML: {inner_ds}')
