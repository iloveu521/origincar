#!/usr/bin/env python3
"""Generate the final self-contained HTML waypoint editor with ALL data pre-parsed as JSON."""
import base64, json, math, yaml

# ============================================================
# 1. Read maps
# ============================================================
def read_pgm(path):
    with open(path, 'rb') as f:
        f.readline()
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b'#'): continue
            tokens.extend(line.strip().split())
        return int(tokens[0]), int(tokens[1]), f.read()

w, h, mraw = read_pgm('src/origincar_base/map/race_modify.pgm')
_, _, kraw = read_pgm('src/origincar_base/map/race_keepout.pgm')
b64m = base64.b64encode(mraw).decode('ascii')
b64k = base64.b64encode(kraw).decode('ascii')

# ============================================================
# 2. Read and validate YAML
# ============================================================
with open('src/origincar_task/config/waypoints_flowpath_custom_rpp713.yaml', 'r') as f:
    yaml_data = yaml.safe_load(f)

# Extract routes and waypoints as JSON
editor_data = {
    'waypoints': yaml_data.get('waypoints', {}),
    'routes': yaml_data.get('routes', {}),
    'field_to_map': yaml_data.get('field_to_map', {}),
    'custom_rpp': yaml_data.get('custom_rpp', {}),
    'maps': {
        'modify_b64': b64m,
        'keepout_b64': b64k,
        'w': w, 'h': h,
    },
}

# Validate
ccw = editor_data['routes'].get('ring_ccw', [])
if ccw:
    gaps = [math.hypot(ccw[i]['x']-ccw[i-1]['x'], ccw[i]['y']-ccw[i-1]['y'])
            for i in range(1, len(ccw))]
    print(f"ring_ccw: {len(ccw)} pts, gap={max(gaps)*100:.1f}cm, len={sum(gaps):.3f}m")

print(f"Routes: {list(editor_data['routes'].keys())}")
print(f"Waypoints: {list(editor_data['waypoints'].keys())}")

# ============================================================
# 3. Generate HTML
# ============================================================
json_str = json.dumps(editor_data, ensure_ascii=False)
# Escape for embedding in JS
json_escaped = json_str.replace('\\', '\\\\').replace("'", "\\'")

html = f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OriginCar Waypoint Editor</title>
<style>
*{{box-sizing:border-box}}body{{margin:0;background:#f4f6f2;color:#172026;font-family:system-ui,-apple-system,"Segoe UI",sans-serif}}
button,input,select,textarea{{font:inherit}}button,select,input{{min-height:34px}}
button{{border:1px solid #bac5c8;border-radius:6px;background:#fff;padding:4px 10px;cursor:pointer}}
button:hover{{background:#e9eef0}}button.primary{{background:#172026;color:#fff}}button.danger{{color:#b52d28}}
input,select,textarea{{border:1px solid #bac5c8;border-radius:6px;background:#fff}}
textarea{{width:100%;min-height:280px;padding:10px;font:12px ui-monospace,Consolas,monospace}}
.app{{display:grid;grid-template-columns:minmax(0,1fr)440px;gap:12px;padding:12px}}
.toolbar,.status,.actions{{display:flex;flex-wrap:wrap;gap:8px;align-items:end}}
.toolbar{{margin-bottom:10px}}label.field{{display:grid;gap:3px;font-size:13px}}
label.check{{display:flex;gap:5px;align-items:center;min-height:34px;font-size:13px}}
.panel{{background:#fff;border:1px solid #bac5c8;border-radius:8px;overflow:hidden;margin-bottom:10px}}
.head{{display:flex;justify-content:space-between;align-items:center;padding:10px;border-bottom:1px solid #d9e0e2}}
.head h2{{font-size:16px;margin:0}}.body{{padding:10px}}
.svg-wrap{{position:relative;overflow:auto;max-height:74vh;border:1px solid #bac5c8;border-radius:8px;background:#e6e8e7}}
svg{{display:block;touch-action:none}}
.map-outline{{fill:none;stroke:#344148;stroke-width:.8}}
.grid-line{{stroke:#aeb8bc;stroke-width:.25}}
.route-shadow{{fill:none;stroke:rgba(255,255,255,.9);stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round}}
.route{{fill:none;stroke:#1769aa;stroke-width:.95;stroke-linecap:round;stroke-linejoin:round}}
.point-hit{{fill:transparent;cursor:grab}}
.point-dot{{fill:#1769aa;stroke:#fff;stroke-width:.55;pointer-events:none}}
.point-dot.active{{fill:#c0362c}}
.point-dot.waypoint-dot{{fill:#247a4d;stroke:#fff;stroke-width:.55;pointer-events:none}}
.yaw{{stroke:#172026;stroke-width:.45;pointer-events:none}}
.yaw-handle{{fill:#a57416;stroke:#fff;stroke-width:.55;cursor:grab}}
.point-label{{fill:#172026;stroke:rgba(255,255,255,.9);stroke-width:1.5;paint-order:stroke fill;font-size:2.6px;font-weight:700;pointer-events:none}}
.waypoint-label{{fill:#247a4d;stroke:rgba(255,255,255,.95);stroke-width:1.8;paint-order:stroke fill;font-size:2.7px;font-weight:700;pointer-events:none}}
.ghost{{fill:none;stroke:#a57416;stroke-width:.75;stroke-dasharray:2 1.5;pointer-events:none}}
.metric{{background:#fff;border:1px solid #bac5c8;border-radius:7px;padding:8px;min-width:120px}}
.metric span{{display:block;color:#66747c;font-size:12px}}.metric strong{{font-size:14px}}
.zoom-badge{{position:absolute;right:8px;bottom:8px;background:rgba(23,32,38,.78);color:#fff;font-size:11px;padding:2px 6px;border-radius:4px;pointer-events:none}}
.muted,.toast{{color:#66747c;font-size:13px}}
.table-wrap{{max-height:330px;overflow:auto}}table{{width:100%;border-collapse:collapse;font-size:13px}}
th,td{{border-bottom:1px solid #d8dee1;padding:5px;text-align:right;white-space:nowrap}}
th:first-child,td:first-child{{text-align:left}}tr.active{{background:#e4f0f8}}
.hint{{font-size:12px;color:#66747c;line-height:1.4}}
.waypoint-card{{display:flex;gap:6px;align-items:center;padding:4px 0;border-bottom:1px solid #e9eef0;font-size:13px}}
.waypoint-card:last-child{{border-bottom:none}}.waypoint-card .name{{font-weight:600;min-width:100px;color:#247a4d}}
.waypoint-card .coords{{color:#66747c;font-size:12px}}
@media(max-width:950px){{.app{{grid-template-columns:1fr}}}}
</style>
</head>
<body>
<main class="app">
<section>
<div class="toolbar">
<label class="field">Map<select id="ms"><option value="race_modify">race_modify</option><option value="race_keepout">race_keepout</option></select></label>
<label class="field">Route<select id="rs"></select></label>
<label class="check"><input id="sy" type="checkbox" checked>Yaw</label>
<label class="check"><input id="sl" type="checkbox" checked>Labels</label>
<label class="check"><input id="sw" type="checkbox" checked>Waypoints</label>
<label class="check"><input id="sc" type="checkbox" checked>Curvature</label>
<button id="ba">Add</button><button id="bd" class="danger">Delete</button>
<button id="bu">Undo</button><button id="br">Redo</button>
<button id="bs" class="primary" style="background:#247a4d">Smooth CR</button>
</div>
<div class="svg-wrap">
<svg id="svg" viewBox="-4 -4 128 122" preserveAspectRatio="xMidYMid meet">
<image id="mi" x="0" y="0" width="115" height="114" preserveAspectRatio="none"></image>
<g id="gr"></g><g id="wl"></g><g id="cl"></g>
<path id="rs2" class="route-shadow"></path><path id="rp" class="route"></path>
<g id="pl"></g><circle id="gh" class="ghost" cx="-100" cy="-100" r="1.8"></circle>
</svg>
<div class="zoom-badge" id="zb">100%</div>
</div>
<div class="status" style="margin-top:10px">
<div class="metric"><span>Cursor</span><strong id="cu">-</strong></div>
<div class="metric"><span>Points</span><strong id="cn">0</strong></div>
<div class="metric"><span>Length</span><strong id="ln">0m</strong></div>
<div class="metric"><span>Max Dir</span><strong id="md">-</strong></div>
</div>
</section>
<aside>
<div class="panel"><div class="head"><h2>Waypoints</h2></div><div class="body"><div id="wpl"></div></div></div>
<div class="panel"><div class="head"><h2>Points</h2><div class="actions"><button id="rv">Reverse</button><button id="rl">Reload</button></div></div><div class="body">
<div class="hint">Drag points/yaw handles. DblClick=add. Shift+click=insert. Right-click=delete. Ctrl+wheel=zoom. Arrows=select.</div>
<div id="em" class="muted" style="margin-top:8px">No points</div>
<div class="table-wrap"><table id="tb" hidden><thead><tr><th>#</th><th>x</th><th>y</th><th>yaw</th><th>motion</th><th>dir</th></tr></thead><tbody></tbody></table></div>
</div></div>
<div class="panel"><div class="head"><h2>Edit</h2><button id="ay">Auto yaw</button></div><div class="body">
<div id="ac" class="muted">No selection</div>
<div class="actions" style="margin-top:8px">
<label class="field">x<input id="xv" type="number" step="0.001"></label>
<label class="field">y<input id="yv" type="number" step="0.001"></label>
<label class="field">yaw(rad)<input id="yr" type="number" step="0.001"></label>
<label class="field">motion<select id="mv"><option value="forward">forward</option><option value="reverse">reverse</option></select></label>
</div></div></div>
<div class="panel"><div class="head"><h2>Export</h2><button id="cp" class="primary">Copy</button></div><div class="body">
<textarea id="yo" readonly spellcheck="false"></textarea>
<div class="actions" style="margin-top:8px"><button id="dl">Download</button></div>
<div id="to" class="toast"></div></div></div>
</aside>
</main>
<script>
// ===== PRE-PARSED DATA =====
var ED = {json_escaped};

// ===== MAP RENDERING =====
var MC={{}};
function rm(){{
 for(var mi=0;mi<2;mi++){{
  var mode=mi===0?'race_modify':'race_keepout';
  var b64=mode==='race_modify'?ED.maps.modify_b64:ED.maps.keepout_b64;
  if(!b64)continue;
  var bin=atob(b64),w=ED.maps.w,h=ED.maps.h;
  var c=document.createElement('canvas');c.width=w;c.height=h;
  var x=c.getContext('2d'),d=x.createImageData(w,h);
  for(var i=0;i<bin.length;i++){{var v=bin.charCodeAt(i),j=i*4;
   if(v<=80){{d.data[j]=35;d.data[j+1]=8;d.data[j+2]=8;d.data[j+3]=220}}
   else if(v>=200){{d.data[j]=220;d.data[j+1]=225;d.data[j+2]=210;d.data[j+3]=255}}
   else{{var t=(v-81)/119;d.data[j]=180*t;d.data[j+1]=190*t;d.data[j+2]=178*t;d.data[j+3]=255}}
  }}
  x.putImageData(d,0,0);
  MC[mode]=c.toDataURL('image/png');
 }}
 mi.src=MC[ms.value]||'';
}}
// ===== GLOBALS =====
var mo={{width:115,height:114,resolution:0.05,origin:[-0.866,-0.648,0.0]}};
function mx(x){{return (x-mo.origin[0])/mo.resolution}}
function my(y){{return mo.height-(y-mo.origin[1])/mo.resolution}}
function wx(px){{return px*mo.resolution+mo.origin[0]}}
function wy(py){{return (mo.height-py)*mo.resolution+mo.origin[1]}}
function ny(a){{while(a>Math.PI)a-=Math.PI*2;while(a<-Math.PI)a+=Math.PI*2;return a}}
function fm(n,d){{d=d||3;return Number(n||0).toFixed(d)}}
function pt(evt){{var p=svg.createSVGPoint();p.x=evt.clientX;p.y=evt.clientY;return p.matrixTransform(svg.getScreenCTM().inverse())}}
function cur(){{return ST.routes[ST.rn]||[]}}
function len(pts){{var d=0;for(var i=1;i<pts.length;i++)d+=Math.hypot(pts[i].x-pts[i-1].x,pts[i].y-pts[i-1].y);return d}}
function dc(p1,p2,p3){{var a1=Math.atan2(p2.y-p1.y,p2.x-p1.x),a2=Math.atan2(p3.y-p2.y,p3.x-p2.x);var d=a2-a1;while(d>Math.PI)d-=Math.PI*2;while(d<-Math.PI)d+=Math.PI*2;return Math.abs(d)}}
function psg(px,py,x1,y1,x2,y2){{var dx=x2-x1,dy=y2-y1,l2=dx*dx+dy*dy;if(l2<1e-12)return Math.hypot(px-x1,py-y1);var t=Math.max(0,Math.min(1,((px-x1)*dx+(py-y1)*dy)/l2));return Math.hypot(px-(x1+t*dx),py-(y1+t*dy))}}
function aw(i){{var pts=cur();if(i<0||i>=pts.length)return;var a=pts[Math.max(0,i-1)],b=pts[Math.min(pts.length-1,i+1)];pts[i].yaw=ny(Math.atan2(b.y-a.y,b.x-a.x))}}
function toast(m){{to.textContent=m;setTimeout(function(){{if(to.textContent===m)to.textContent=''}},1800)}}

// ===== ELLS =====
function $(id){{return document.getElementById(id)}}
var ms=$('ms'),rs=$('rs'),sy=$('sy'),sl=$('sl'),sw=$('sw'),sc=$('sc'),ba=$('ba'),bd=$('bd'),bu=$('bu'),br=$('br'),bs=$('bs'),svg=$('svg'),mi=$('mi'),gr=$('gr'),wl=$('wl'),cl=$('cl'),rs2=$('rs2'),rp=$('rp'),pl=$('pl'),gh=$('gh'),zb=$('zb'),cu=$('cu'),cn=$('cn'),ln=$('ln'),md=$('md'),wpl=$('wpl'),rv=$('rv'),rl=$('rl'),em=$('em'),tb=$('tb'),ac=$('ac'),ay=$('ay'),xv=$('xv'),yv=$('yv'),yr=$('yr'),mv=$('mv'),cp=$('cp'),yo=$('yo'),dl=$('dl'),to=$('to');

// ===== STATE =====
var ST={{field_to_map:ED.field_to_map||{{}},waypoints:ED.waypoints||{{}},custom_rpp:ED.custom_rpp||{{}},routes:ED.routes||{{}},rn:'',active:-1,drag:null,locked:false,undo:[],redo:[],view:{{x:-4,y:-4,w:128,h:122,baseW:128}}}};
ST.rn=Object.keys(ST.routes)[0]||'';
function pu(){{ST.undo.push(JSON.parse(JSON.stringify({{routes:ST.routes,waypoints:ST.waypoints}})));if(ST.undo.length>50)ST.undo.shift();ST.redo=[]}}
function re(s){{ST.routes=JSON.parse(JSON.stringify(s.routes));ST.waypoints=JSON.parse(JSON.stringify(s.waypoints));ST.active=-1;dr()}}

// ===== ROUTE LIST =====
function pr(){{rs.innerHTML='';for(var n in ST.routes){{if(!ST.routes.hasOwnProperty(n))continue;var o=document.createElement('option');o.value=n;o.textContent=n+' ('+(ST.routes[n]||[]).length+')';rs.appendChild(o)}}if(ST.routes[ST.rn])rs.value=ST.rn}}

// ===== DRAW =====
function dg(){{
 gr.innerHTML='';
 for(var x=Math.ceil(mo.origin[0]*2)/2;x<=mo.origin[0]+mo.width*mo.resolution;x+=.5){{
  var l=document.createElementNS('http://www.w3.org/2000/svg','line');
  l.setAttribute('x1',mx(x));l.setAttribute('x2',mx(x));l.setAttribute('y1',0);l.setAttribute('y2',mo.height);
  l.setAttribute('class','grid-line');gr.appendChild(l);
 }}
 for(var y=Math.ceil(mo.origin[1]*2)/2;y<=mo.origin[1]+mo.height*mo.resolution;y+=.5){{
  var l2=document.createElementNS('http://www.w3.org/2000/svg','line');
  l2.setAttribute('x1',0);l2.setAttribute('x2',mo.width);l2.setAttribute('y1',my(y));l2.setAttribute('y2',my(y));
  l2.setAttribute('class','grid-line');gr.appendChild(l2);
 }}
 var r=document.createElementNS('http://www.w3.org/2000/svg','rect');
 r.setAttribute('x',0);r.setAttribute('y',0);r.setAttribute('width',mo.width);r.setAttribute('height',mo.height);
 r.setAttribute('class','map-outline');gr.appendChild(r);
}}

function dwp(){{
 wl.innerHTML='';
 if(!sw.checked)return;
 for(var n in ST.waypoints){{if(!ST.waypoints.hasOwnProperty(n))continue;var wp=ST.waypoints[n];if(!wp||typeof wp.x!=='number')continue;
  var x=mx(wp.x),y=my(wp.y);
  var d=document.createElementNS('http://www.w3.org/2000/svg','circle');
  d.setAttribute('cx',x);d.setAttribute('cy',y);d.setAttribute('r',1.6);
  d.setAttribute('class','point-dot waypoint-dot');wl.appendChild(d);
  if(sl.checked){{
   var t=document.createElementNS('http://www.w3.org/2000/svg','text');
   t.setAttribute('x',x+2.2);t.setAttribute('y',y-2.2);t.setAttribute('class','waypoint-label');
   t.textContent=n;wl.appendChild(t);
  }}
 }}
 wpl.innerHTML='';
 for(var n2 in ST.waypoints){{if(!ST.waypoints.hasOwnProperty(n2))continue;var wp2=ST.waypoints[n2];if(!wp2||typeof wp2.x!=='number')continue;
  wpl.innerHTML+='<div class="waypoint-card"><div class="name">'+n2+'</div><div class="coords">x='+fm(wp2.x)+' y='+fm(wp2.y)+'</div></div>';
 }}
 if(!wpl.innerHTML){{wpl.className='muted';wpl.textContent='No waypoints'}}
}}

function dcv(){{
 cl.innerHTML='';
 if(!sc.checked)return;
 var pts=cur();if(pts.length<3)return;
 for(var i=1;i<pts.length-1;i++){{
  var d=dc(pts[i-1],pts[i],pts[i+1])*180/Math.PI;
  var clr='#40c060';if(d>20)clr='#ff4040';else if(d>12)clr='#ffa040';
  var x0=mx(pts[i-1].x),y0=my(pts[i-1].y),x1=mx(pts[i].x),y1=my(pts[i].y);
  var ln=document.createElementNS('http://www.w3.org/2000/svg','line');
  ln.setAttribute('x1',x0);ln.setAttribute('y1',y0);ln.setAttribute('x2',x1);ln.setAttribute('y2',y1);
  ln.setAttribute('stroke',clr);ln.setAttribute('stroke-width','2.5');ln.setAttribute('stroke-linecap','round');
  cl.appendChild(ln);
 }}
}}

function rpth(pts){{return pts.map(function(p,i){{return (i?'L':'M')+mx(p.x).toFixed(2)+' '+my(p.y).toFixed(2)}}).join(' ')}}

function dpt(){{
 pl.innerHTML='';var pts=cur();
 pts.forEach(function(p,i){{
  var x=mx(p.x),y=my(p.y);
  if(sy.checked){{
   var l2=5,x2=x+Math.cos(p.yaw||0)*l2,y2=y-Math.sin(p.yaw||0)*l2;
   var l=document.createElementNS('http://www.w3.org/2000/svg','line');
   l.setAttribute('x1',x);l.setAttribute('y1',y);l.setAttribute('x2',x2);l.setAttribute('y2',y2);
   l.setAttribute('class','yaw');pl.appendChild(l);
   var yh=document.createElementNS('http://www.w3.org/2000/svg','circle');
   yh.setAttribute('cx',x2);yh.setAttribute('cy',y2);yh.setAttribute('r',1.25);
   yh.setAttribute('class','yaw-handle');yh.dataset.yaw=i;pl.appendChild(yh);
  }}
  var hit=document.createElementNS('http://www.w3.org/2000/svg','circle');
  hit.setAttribute('cx',x);hit.setAttribute('cy',y);hit.setAttribute('r',2.8);
  hit.setAttribute('class','point-hit');hit.dataset.point=i;pl.appendChild(hit);
  var dot=document.createElementNS('http://www.w3.org/2000/svg','circle');
  dot.setAttribute('cx',x);dot.setAttribute('cy',y);dot.setAttribute('r',1.35);
  dot.setAttribute('class','point-dot'+(i===ST.active?' active':''));pl.appendChild(dot);
  if(sl.checked){{
   var tt=document.createElementNS('http://www.w3.org/2000/svg','text');
   tt.setAttribute('x',x+2.0);tt.setAttribute('y',y-2.0);tt.setAttribute('class','point-label');
   tt.textContent=i+1;pl.appendChild(tt);
  }}
 }});
}}

function dtb(){{
 var pts=cur();em.hidden=pts.length>0;tb.hidden=!pts.length;
 var body=tb.querySelector('tbody');body.innerHTML='';var maxD=0;
 pts.forEach(function(p,i){{
  var tr=document.createElement('tr');if(i===ST.active)tr.className='active';
  var dd='-';
  if(i>=2){{var d=dc(pts[i-2],pts[i-1],pts[i])*180/Math.PI;dd=d.toFixed(1);maxD=Math.max(maxD,d)}}
  tr.innerHTML='<td>'+(i+1)+'</td><td>'+fm(p.x)+'</td><td>'+fm(p.y)+'</td><td>'+fm(p.yaw,4)+'</td><td>'+(p.motion||'forward')+'</td><td style="color:'+(maxD>20?'#c0362c':maxD>12?'#a57416':'#247a4d')+'">'+dd+'</td>';
  tr.onclick=function(){{if(ST.locked)return;pu();ST.active=i;ST.locked=true;dr()}};body.appendChild(tr);
 }});
 md.textContent=maxD>0?maxD.toFixed(1):'-';
}}

function sef(){{
 var p=cur()[ST.active];
 if(!p){{ac.textContent='No selection';xv.value='';yv.value='';yr.value='';mv.value='forward';return}}
 ac.textContent='#'+(ST.active+1)+' / '+cur().length;
 xv.value=fm(p.x);yv.value=fm(p.y);yr.value=fm(p.yaw);mv.value=p.motion||'forward';
}}

// ===== YAML =====
function ys(v){{if(typeof v==='number')return Number(v.toFixed(6));if(typeof v==='boolean')return v?'true':'false';return JSON.stringify(v||'')}}
function fty(){{var f=ST.field_to_map||{{}};var o='field_to_map:\\n';if(f.source)o+='  source: '+ys(f.source)+'\\n';if(f.source_map)o+='  source_map: '+ys(f.source_map)+'\\n';if(f.source_yaml)o+='  source_yaml: '+ys(f.source_yaml)+'\\n';if(f.method)o+='  method: '+ys(f.method)+'\\n';if(f.resolution!==undefined)o+='  resolution: '+ys(f.resolution)+'\\n';return o}}
function wyy(){{var o='\\nwaypoints:\\n';for(var n in ST.waypoints){{if(!ST.waypoints.hasOwnProperty(n))continue;var wp=ST.waypoints[n];if(!wp||typeof wp.x!=='number')continue;o+='  '+n+':\\n    x: '+ys(wp.x)+'\\n    y: '+ys(wp.y)+'\\n    yaw: '+ys(wp.yaw||0)+'\\n';if(wp.description)o+='    description: '+ys(wp.description)+'\\n'}}return o}}
function rpy(p){{return'  - x: '+ys(p.x)+'\\n    y: '+ys(p.y)+'\\n    yaw: '+ys(p.yaw||0)+'\\n    pause: '+ys(p.pause||0)+'\\n    motion: '+(p.motion||'forward')+'\\n    pass_radius: '+ys(p.pass_radius||0.3)+'\\n    reverse_pass_radius: '+ys(p.reverse_pass_radius||0.3)}}
function rty(){{var o='routes:\\n';for(var n in ST.routes){{if(!ST.routes.hasOwnProperty(n))continue;o+='  '+n+':\\n';var pts=ST.routes[n];for(var i=0;i<pts.length;i++)o+=rpy(pts[i])+'\\n'}}return o}}
function cpy(){{var c=ST.custom_rpp||{{}},s=c.stuck_skip||{{}};return'\\ncustom_rpp:\\n  stuck_skip:\\n    enabled: '+ys(s.enabled!==undefined?s.enabled:false)+'\\n    timeout_sec: '+ys(s.timeout_sec||0.8)+'\\n    max_distance: '+ys(s.max_distance||0.45)+'\\n    recovery_enabled: '+ys(s.recovery_enabled!==undefined?s.recovery_enabled:false)+'\\n    recovery_max_distance: '+ys(s.recovery_max_distance||1.2)+'\\n    recovery_duration_sec: '+ys(s.recovery_duration_sec||0.35)+'\\n    recovery_speed: '+ys(s.recovery_speed||0.12)+'\\n    recovery_angular_z: '+ys(s.recovery_angular_z||0.25)+'\\n    description: '+ys(s.description||'')+'\\n'}}
function uy(){{yo.value=fty()+wyy()+rty()+cpy()}}

function dr(){{
 dg();dwp();dcv();
 var pts=cur();var d=rpth(pts);
 rp.setAttribute('d',d);rs2.setAttribute('d',d);
 dpt();dtb();sef();
 cn.textContent=String(pts.length);ln.textContent=fm(len(pts))+' m';
 uy();zb.textContent=Math.round(ST.view.baseW/ST.view.w*100)+'%';
}}

// ===== EVENTS =====
ms.onchange=function(){{mi.src=MC[ms.value]||''}};
for(var ei=0;ei<[ms,sy,sl,sw,sc].length;ei++)[ms,sy,sl,sw,sc][ei].onchange=dr;

svg.addEventListener('pointermove',function(evt){{
 var p=pt(evt);cu.textContent=fm(wx(p.x))+', '+fm(wy(p.y));
 gh.setAttribute('cx',p.x);gh.setAttribute('cy',p.y);
 if(!ST.drag)return;evt.preventDefault();
 if(ST.drag.type==='point'){{var q=cur()[ST.drag.index];q.x=wx(p.x);q.y=wy(p.y);dr()}}
 else if(ST.drag.type==='yaw'){{var q2=cur()[ST.drag.index];q2.yaw=ny(Math.atan2(my(q2.y)-p.y,p.x-mx(q2.x)));dr()}}
}});

svg.addEventListener('pointerdown',function(evt){{
 var t=evt.target;
 if(t.dataset.point!==undefined){{pu();ST.active=Number(t.dataset.point);ST.locked=true;ST.drag={{type:'point',index:ST.active}};svg.setPointerCapture(evt.pointerId);dr()}}
 else if(t.dataset.yaw!==undefined){{pu();ST.active=Number(t.dataset.yaw);ST.locked=true;ST.drag={{type:'yaw',index:ST.active}};svg.setPointerCapture(evt.pointerId);dr()}}
}});

svg.addEventListener('pointerup',function(evt){{ST.drag=null;try{{svg.releasePointerCapture(evt.pointerId)}}catch(e){{}}}});
svg.addEventListener('pointerleave',function(){{if(!ST.drag){{gh.setAttribute('cx',-100);gh.setAttribute('cy',-100)}}}});

svg.addEventListener('dblclick',function(evt){{
 var p=pt(evt);ST.locked=false;
 var pts=cur();var i=ST.active>=0?ST.active+1:pts.length;
 pu();pts.splice(i,0,{{x:wx(p.x),y:wy(p.y),yaw:0,pause:0,motion:'forward',pass_radius:0.3,reverse_pass_radius:0.3}});
 ST.active=i;aw(i);dr();
}});

svg.addEventListener('click',function(evt){{
 if(evt.shiftKey&&ST.active<0){{var p=pt(evt);var pts=cur();
  var best=pts.length-1,bd=Infinity;
  for(var i=0;i<pts.length-1;i++){{var d=psg(wx(p.x),wy(p.y),pts[i].x,pts[i].y,pts[i+1].x,pts[i+1].y);if(d<bd){{bd=d;best=i}}}}
  pu();pts.splice(best+1,0,{{x:wx(p.x),y:wy(p.y),yaw:0,pause:0,motion:'forward',pass_radius:0.3,reverse_pass_radius:0.3}});
  ST.active=best+1;aw(ST.active);ST.locked=true;dr();
 }}
}});

svg.addEventListener('contextmenu',function(evt){{
 evt.preventDefault();if(ST.active<0)return;
 var pts=cur();pu();pts.splice(ST.active,1);ST.active=Math.min(ST.active,pts.length-1);dr();
}});

svg.addEventListener('wheel',function(evt){{
 if(!evt.ctrlKey)return;evt.preventDefault();
 var scale=evt.deltaY<0?0.9:1.1;var p=pt(evt);
 ST.view.w=Math.max(25,Math.min(200,ST.view.w*scale));ST.view.h=ST.view.w*122/128;
 ST.view.x=p.x-(p.x-ST.view.x)*scale;ST.view.y=p.y-(p.y-ST.view.y)*scale;
 svg.setAttribute('viewBox',ST.view.x+' '+ST.view.y+' '+ST.view.w+' '+ST.view.h);
 dr();
}},{{passive:false}});

// Keyboard
document.addEventListener('keydown',function(evt){{
 if(evt.target.tagName==='INPUT'||evt.target.tagName==='TEXTAREA'||evt.target.tagName==='SELECT')return;
 var pts=cur();if(!pts.length)return;
 if(evt.key==='ArrowLeft'){{evt.preventDefault();ST.locked=true;ST.active=Math.max(0,ST.active-1);dr()}}
 else if(evt.key==='ArrowRight'){{evt.preventDefault();ST.locked=true;ST.active=Math.min(pts.length-1,ST.active+1);dr()}}
 else if(evt.key==='Delete'||evt.key==='Backspace'){{evt.preventDefault();if(ST.active<0)return;pu();pts.splice(ST.active,1);ST.active=Math.min(ST.active,pts.length-1);dr()}}
 else if(evt.ctrlKey&&evt.key==='z'){{evt.preventDefault();var s=ST.undo.pop();if(s){{ST.redo.push(JSON.parse(JSON.stringify({{routes:ST.routes,waypoints:ST.waypoints}})));re(s)}}}}
 else if(evt.ctrlKey&&evt.key==='y'){{evt.preventDefault();var s2=ST.redo.pop();if(s2){{ST.undo.push(JSON.parse(JSON.stringify({{routes:ST.routes,waypoints:ST.waypoints}})));re(s2)}}}}
}});

// Buttons
ba.onclick=function(){{var svgR=svg.getBoundingClientRect();var pt2=svg.createSVGPoint();pt2.x=svgR.left+svgR.width/2;pt2.y=svgR.top+svgR.height/2;var p=pt2.matrixTransform(svg.getScreenCTM().inverse());var pts=cur();ST.locked=false;var i=ST.active>=0?ST.active+1:pts.length;pu();pts.splice(i,0,{{x:wx(p.x),y:wy(p.y),yaw:0,pause:0,motion:'forward',pass_radius:0.3,reverse_pass_radius:0.3}});ST.active=i;aw(i);dr()}};
bd.onclick=function(){{var pts=cur();if(ST.active<0)return;pu();pts.splice(ST.active,1);ST.active=Math.min(ST.active,pts.length-1);dr()}};
bu.onclick=function(){{var s=ST.undo.pop();if(s){{ST.redo.push(JSON.parse(JSON.stringify({{routes:ST.routes,waypoints:ST.waypoints}})));re(s)}}}};
br.onclick=function(){{var s=ST.redo.pop();if(s){{ST.undo.push(JSON.parse(JSON.stringify({{routes:ST.routes,waypoints:ST.waypoints}})));re(s)}}}};
rv.onclick=function(){{pu();cur().reverse();ST.active=-1;dr()}};
rl.onclick=function(){{pu();ST.routes[ST.rn]=JSON.parse(JSON.stringify(ED.routes[ST.rn]||[]));ST.active=-1;dr()}};

// Smooth
bs.onclick=function(){{
 var pts=cur();if(pts.length<4)return;
 var sp=0.05;var res=[];pu();
 for(var i=0;i<pts.length-1;i++){{
  var p0=pts[Math.max(0,i-1)],p1=pts[i],p2=pts[i+1],p3=pts[Math.min(pts.length-1,i+2)];
  var seg=Math.hypot(p2.x-p1.x,p2.y-p1.y),st=Math.max(1,Math.ceil(seg/sp));
  for(var j=0;j<st;j++){{var t=j/st,t2=t*t,t3=t2*t;
   res.push({{x:0.5*((2*p1.x)+(-p0.x+p2.x)*t+(2*p0.x-5*p1.x+4*p2.x-p3.x)*t2+(-p0.x+3*p1.x-3*p2.x+p3.x)*t3),
              y:0.5*((2*p1.y)+(-p0.y+p2.y)*t+(2*p0.y-5*p1.y+4*p2.y-p3.y)*t2+(-p0.y+3*p1.y-3*p2.y+p3.y)*t3)}});}}
 }}
 res.push({{x:pts[pts.length-1].x,y:pts[pts.length-1].y}});
 for(var k=0;k<res.length;k++){{if(k<res.length-1)res[k].yaw=ny(Math.atan2(res[k+1].y-res[k].y,res[k+1].x-res[k].x));else if(k>0)res[k].yaw=ny(Math.atan2(res[k].y-res[k-1].y,res[k].x-res[k-1].x));res[k].pause=0;res[k].motion=pts[0].motion||'forward';res[k].pass_radius=0.3;res[k].reverse_pass_radius=0.3;}}
 ST.routes[ST.rn]=res;ST.active=-1;dr();toast('Smoothed: '+res.length+' pts');
}};

ay.onclick=function(){{if(ST.active>=0){{pu();aw(ST.active);dr()}}}};
xv.onchange=function(){{if(ST.active>=0){{pu();cur()[ST.active].x=Number(xv.value);dr()}}}};
yv.onchange=function(){{if(ST.active>=0){{pu();cur()[ST.active].y=Number(yv.value);dr()}}}};
yr.onchange=function(){{if(ST.active>=0){{pu();cur()[ST.active].yaw=ny(Number(yr.value));dr()}}}};
mv.onchange=function(){{if(ST.active>=0){{pu();cur()[ST.active].motion=mv.value;dr()}}}};
cp.onclick=function(){{navigator.clipboard.writeText(yo.value).then(function(){{toast('Copied!')}})}};
dl.onclick=function(){{var blob=new Blob([yo.value],{{type:'text/yaml'}});var a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='waypoints.yaml';a.click();URL.revokeObjectURL(a.href)}};
rs.onchange=function(){{ST.rn=rs.value;ST.active=-1;dr()}};

// ===== INIT =====
rm();pr();dr();mi.src=MC[ms.value]||'';
</script>
</body>
</html>'''

# Write
output = 'tools/map_waypoint_designer.html'
with open(output, 'w', encoding='utf-8') as f:
    f.write(html)

print(f"Written: {output} ({len(html)} chars)")
print("[OK] Complete!")
