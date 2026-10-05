# Shared building blocks for mockups (icons, chrome, iso 3D scene)
import math
ICONS = {
 'cube':'<path d="M12 2 3 7v10l9 5 9-5V7z"/><path d="M3 7l9 5 9-5M12 12v10"/>',
 'joint':'<circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M4.9 4.9l2.8 2.8M16.3 16.3l2.8 2.8"/>',
 'logic':'<rect x="3" y="4" width="7" height="6" rx="1.5"/><rect x="14" y="14" width="7" height="6" rx="1.5"/><path d="M10 7h3a2 2 0 0 1 2 2v5"/>',
 'robot':'<path d="M5 21h10M7 21v-4l4-5 5 2 3-4"/><circle cx="11" cy="12" r="1.6"/><circle cx="16" cy="14" r="1.4"/><path d="M19 10l2-1"/>',
 'bug':'<path d="M8 8a4 4 0 0 1 8 0v6a4 4 0 0 1-8 0z"/><path d="M4 12h4M16 12h4M5 6l3 2M19 6l-3 2M5 18l3-2M19 18l-3-2"/>',
 'plc':'<rect x="3" y="5" width="18" height="14" rx="2"/><path d="M7 9h2M7 12h2M7 15h2M12 9h5M12 12h5M12 15h3"/>',
 'search':'<circle cx="11" cy="11" r="6"/><path d="m20 20-4.2-4.2"/>',
 'spark':'<path d="M12 3l1.8 5.2L19 10l-5.2 1.8L12 17l-1.8-5.2L5 10l5.2-1.8z"/><path d="M19 15l.8 2.2L22 18l-2.2.8L19 21l-.8-2.2L16 18l2.2-.8z"/>',
 'tree':'<path d="M4 5h6M8 12h6M8 19h6M6 5v14h2M6 12h2"/>',
 'lib':'<rect x="4" y="4" width="6" height="6" rx="1"/><rect x="14" y="4" width="6" height="6" rx="1"/><rect x="4" y="14" width="6" height="6" rx="1"/><rect x="14" y="14" width="6" height="6" rx="1"/>',
 'signal':'<path d="M3 12h3l2-6 4 12 2-6h7"/>',
 'code':'<path d="m8 8-4 4 4 4M16 8l4 4-4 4M14 5l-4 14"/>',
 'check':'<path d="m5 12 4 4 10-10"/>',
 'shield':'<path d="M12 3 4 6v6c0 5 3.5 8 8 9 4.5-1 8-4 8-9V6z"/>',
 'chev':'<path d="m9 6 6 6-6 6"/>','chevd':'<path d="m6 9 6 6 6-6"/>',
 'eye':'<path d="M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
 'lock':'<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V8a4 4 0 0 1 8 0v3"/>',
 'move':'<path d="M12 3v18M3 12h18M12 3l-3 3M12 3l3 3M12 21l-3-3M12 21l3-3M3 12l3-3M3 12l3 3M21 12l-3-3M21 12l-3 3"/>',
 'rotate':'<path d="M20 12a8 8 0 1 1-2.3-5.7"/><path d="M20 4v5h-5"/>',
 'ruler':'<path d="M3 17 17 3l4 4L7 21z"/><path d="M7 13l2 2M10 10l2 2M13 7l2 2"/>',
 'section':'<path d="M4 20 20 4"/><rect x="3" y="3" width="18" height="18" rx="2"/>',
 'play':'<path d="M7 5v14l12-7z"/>','pause':'<path d="M8 5v14M16 5v14"/>',
 'stepb':'<path d="M18 6v12L9 12zM6 6v12"/>','stepf':'<path d="M6 6v12l9-6zM18 6v12"/>',
 'stop':'<rect x="6" y="6" width="12" height="12" rx="1.5"/>',
 'undo':'<path d="M9 14 4 9l5-5"/><path d="M4 9h10a6 6 0 0 1 0 12h-3"/>',
 'redo':'<path d="m15 14 5-5-5-5"/><path d="M20 9H10a6 6 0 0 0 0 12h3"/>',
 'plus':'<path d="M12 5v14M5 12h14"/>','x':'<path d="M6 6l12 12M18 6 6 18"/>',
 'filter':'<path d="M4 5h16l-6 8v6l-4-2v-4z"/>',
 'layers':'<path d="m12 3 9 5-9 5-9-5z"/><path d="m3 13 9 5 9-5"/>',
 'target':'<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>',
 'warn':'<path d="M12 3 2 20h20z"/><path d="M12 10v4M12 17h.01"/>',
 'alert':'<circle cx="12" cy="12" r="9"/><path d="M12 7v6M12 16h.01"/>',
 'info':'<circle cx="12" cy="12" r="9"/><path d="M12 11v5M12 8h.01"/>',
 'link':'<path d="M10 14a4 4 0 0 0 5.7 0l3-3a4 4 0 0 0-5.7-5.7l-1 1"/><path d="M14 10a4 4 0 0 0-5.7 0l-3 3a4 4 0 0 0 5.7 5.7l1-1"/>',
 'gear':'<circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.9 4.9 7 7M17 17l2.1 2.1M2 12h3M19 12h3M4.9 19.1 7 17M17 7l2.1-2.1"/>',
 'clock':'<circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/>',
 'cpu':'<rect x="6" y="6" width="12" height="12" rx="2"/><path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4"/>',
 'pin':'<path d="M12 17v5M5 17h14l-2-4V5H7v8z"/>',
 'dots':'<circle cx="5" cy="12" r="1"/><circle cx="12" cy="12" r="1"/><circle cx="19" cy="12" r="1"/>',
 'track':'<path d="M2 16h20M2 20h20M5 16v4M10 16v4M15 16v4M20 16v4"/><rect x="8" y="8" width="8" height="6" rx="1"/>',
 'gripper':'<path d="M12 3v6M8 9h8M8 9v6l2 3M16 9v6l-2 3"/>',
 'frame':'<path d="M5 19V5M5 19h14M5 19l9-9"/>',
 'path':'<circle cx="5" cy="18" r="2"/><circle cx="19" cy="6" r="2"/><path d="M7 18c6 0 4-12 10-12"/>',
 'wave':'<path d="M2 12c2.5-6 5-6 7.5 0s5 6 7.5 0 3.5-4 5-2"/>',
 'table':'<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M3 10h18M3 15h18M9 4v16"/>',
 'gantt':'<path d="M4 6h8M8 12h10M6 18h6"/>',
 'bell':'<path d="M6 16V11a6 6 0 0 1 12 0v5l2 2H4z"/><path d="M10 21h4"/>',
 'cmd':'<path d="M9 6a3 3 0 1 0-3 3h12a3 3 0 1 0-3-3v12a3 3 0 1 0 3-3H6a3 3 0 1 0 3 3z"/>',
 'wand':'<path d="M4 20 16 8M14 4v3M18 6h3M19 10v2M10 3h2"/><path d="m15 9 1 1"/>',
 'flow':'<rect x="3" y="3" width="6" height="6" rx="1"/><rect x="15" y="15" width="6" height="6" rx="1"/><path d="M9 6h4a3 3 0 0 1 3 3v6"/>',
 'expand':'<path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/>',
 'mouse':'<rect x="6" y="3" width="12" height="18" rx="6"/><path d="M12 7v4"/>',
 'zap':'<path d="M13 2 4 14h7l-1 8 9-12h-7z"/>',
 'globe':'<circle cx="12" cy="12" r="9"/><path d="M3 12h18M12 3c3 3 3 15 0 18M12 3c-3 3-3 15 0 18"/>',
 'user':'<circle cx="12" cy="8" r="4"/><path d="M4 21c1-4 4-6 8-6s7 2 8 6"/>',
 'min':'<path d="M5 12h14"/>','max':'<rect x="5" y="5" width="14" height="14" rx="1"/>',
 'file':'<path d="M6 3h8l4 4v14H6z"/><path d="M14 3v4h4"/>',
 'mag':'<path d="M4 4h6v6H4zM14 14h6v6h-6z"/><path d="M10 7h4v7"/>',
 'scissors':'<circle cx="6" cy="6" r="3"/><circle cx="6" cy="18" r="3"/><path d="M8.5 8.5 20 20M8.5 15.5 20 4"/>',
}
def ic(name, cls='', color=None):
    st = f' style="color:{color}"' if color else ''
    return f'<svg class="i {cls}" viewBox="0 0 24 24"{st}>{ICONS[name]}</svg>'

MODES=[('cube','建模','1'),('joint','运动学','2'),('logic','逻辑','3'),('robot','机器人','4'),('bug','调试','5'),('plc','虚拟调试','6')]

def titlebar(active, ai_on=False, crumb=('车身焊装线 A','侧围焊接站 OP30')):
    ms=''.join(f'<div class="m{" on" if n==active else ""}">{ic(i,"s")}{n}<kbd>{k}</kbd></div>' for i,n,k in MODES)
    ai = ('<div class="pill ai-on"><span class="dot"></span>AI 助手 · 本地</div>' if ai_on
          else '<div class="pill ai-off"><span class="dot"></span>AI 已关闭</div>')
    return f'''<div class="titlebar">
 <div class="brand"><img src="assets/snippin_logo_small_dark.svg"><span class="prod">应点仿真</span></div>
 <div class="crumb">{ic('chev','s')}<span>{crumb[0]}</span>{ic('chev','s')}<b>{crumb[1]}</b><span class="dirty"></span></div>
 <div class="modes">{ms}</div>
 <div class="tb-right">
  <div class="cmdk">{ic('search','s')}<span>搜索命令 / 对象，或描述意图…</span><span class="k"><kbd class="k">Ctrl</kbd> <kbd class="k">K</kbd></span></div>
  {ai}<span class="lang">中 / EN</span><div class="ib">{ic('bell','s')}</div>
  <div class="winctl"><span>{ic('min','s')}</span><span>{ic('max','s')}</span><span>{ic('x','s')}</span></div>
 </div></div>'''

def rail(items, on):
    out=[]
    for it in items:
        if it=='|': out.append('<div class="sep"></div>'); continue
        name,badge=(it if isinstance(it,tuple) else (it,None))
        b=f'<span class="badge">{badge}</span>' if badge else ''
        out.append(f'<div class="ri{" on" if name==on else ""}">{ic(name,"l")}{b}</div>')
    return '<div class="rail">'+''.join(out)+'<div style="flex:1"></div><div class="ri">'+ic('gear','l')+'</div><div class="ri" style="margin-bottom:8px">'+ic('user','l')+'</div></div>'

def statusbar(left, right):
    return '<div class="status">'+''.join(f'<span class="s">{x}</span>' for x in left)+'<span class="r">'+''.join(f'<span class="s">{x}</span>' for x in right)+'</span></div>'

def page(title, body, extra_css=''):
    return f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><title>{title}</title>
<link rel="stylesheet" href="common.css"><style>{extra_css}</style></head><body>{body}</body></html>'''

# ---------------- iso scene -----------------
class Iso:
    def __init__(s, ox, oy, sc):
        s.ox, s.oy, s.sc = ox, oy, sc
    def p(s, x, y, z=0):
        c, sn = math.cos(math.radians(30)), math.sin(math.radians(30))
        return (s.ox + (x - y) * c * s.sc, s.oy + (x + y) * sn * s.sc - z * s.sc)
    def pts(s, arr):
        return ' '.join(f'{a:.1f},{b:.1f}' for a, b in (s.p(*q) for q in arr))

def shade(hexc, f):
    hexc=hexc.lstrip('#'); r,g,b=[int(hexc[i:i+2],16) for i in (0,2,4)]
    if f>=1: r,g,b=[int(v+(255-v)*(f-1)) for v in (r,g,b)]
    else: r,g,b=[int(v*f) for v in (r,g,b)]
    return f'#{r:02x}{g:02x}{b:02x}'

def box(iso, x, y, z, w, d, h, col, stroke='rgba(0,0,0,.25)', op=1, extra=''):
    t=[(x,y,z+h),(x+w,y,z+h),(x+w,y+d,z+h),(x,y+d,z+h)]
    fx=[(x+w,y,z),(x+w,y+d,z),(x+w,y+d,z+h),(x+w,y,z+h)]
    fy=[(x,y+d,z),(x+w,y+d,z),(x+w,y+d,z+h),(x,y+d,z+h)]
    o=f' opacity="{op}"' if op!=1 else ''
    return (f'<g{o} {extra}><polygon points="{iso.pts(fy)}" fill="{shade(col,.62)}" stroke="{stroke}" stroke-width=".6"/>'
            f'<polygon points="{iso.pts(fx)}" fill="{shade(col,.8)}" stroke="{stroke}" stroke-width=".6"/>'
            f'<polygon points="{iso.pts(t)}" fill="{shade(col,1.08)}" stroke="{stroke}" stroke-width=".6"/></g>')

def cyl(iso, x, y, z, r, h, col, n=28):
    # vertical cylinder: side band + top ellipse
    top=[(x+r*math.cos(a), y+r*math.sin(a), z+h) for a in [i*2*math.pi/n for i in range(n)]]
    bot=[(x+r*math.cos(a), y+r*math.sin(a), z) for a in [i*2*math.pi/n for i in range(n)]]
    # visible half: angles where (cos+sin)>0  -> from -45 to 135 deg
    vis=[i for i in range(n) if math.cos(i*2*math.pi/n)+math.sin(i*2*math.pi/n)>-0.05]
    vis=sorted(vis,key=lambda i:((i*360/n+45)%360))
    side=[bot[i] for i in vis]+[top[i] for i in reversed(vis)]
    gid=f'g{abs(hash((x,y,z,r,h)))%99999}'
    return (f'<defs><linearGradient id="{gid}" x1="0" x2="1"><stop offset="0" stop-color="{shade(col,.55)}"/><stop offset=".55" stop-color="{shade(col,.95)}"/><stop offset="1" stop-color="{shade(col,.7)}"/></linearGradient></defs>'
            f'<polygon points="{iso.pts(side)}" fill="url(#{gid})"/><polygon points="{iso.pts(top)}" fill="{shade(col,1.1)}" stroke="rgba(0,0,0,.2)" stroke-width=".6"/>')

def grid(iso, x0, x1, y0, y1, step, col='rgba(142,146,236,.07)', major=5, mcol='rgba(142,146,236,.14)'):
    out=[]
    k=0
    v=x0
    while v<=x1+1e-6:
        a,b=iso.p(v,y0),iso.p(v,y1); c=mcol if round((v-x0)/step)%major==0 else col
        out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{c}" stroke-width="1"/>'); v+=step
    v=y0
    while v<=y1+1e-6:
        a,b=iso.p(x0,v),iso.p(x1,v); c=mcol if round((v-y0)/step)%major==0 else col
        out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" stroke="{c}" stroke-width="1"/>'); v+=step
    return ''.join(out)

def seg(iso, a, b, w, col, cap='round', op=1):
    p,q=iso.p(*a),iso.p(*b)
    return (f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{shade(col,.6)}" stroke-width="{w+3}" stroke-linecap="{cap}" opacity="{op}"/>'
            f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{col}" stroke-width="{w}" stroke-linecap="{cap}" opacity="{op}"/>'
            f'<line x1="{p[0]-w*.18:.1f}" y1="{p[1]-w*.22:.1f}" x2="{q[0]-w*.18:.1f}" y2="{q[1]-w*.22:.1f}" stroke="{shade(col,1.35)}" stroke-width="{max(1.5,w*.22)}" stroke-linecap="{cap}" opacity="{.55*op}"/>')

def jointdot(iso, a, r, col, op=1):
    p=iso.p(*a)
    return (f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="{r}" fill="{shade(col,.55)}" opacity="{op}"/>'
            f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="{r*.62}" fill="{shade(col,1.15)}" opacity="{op}"/>')

def robot(iso, bx, by, bz, J, col='#E98A3A', op=1, ghost=False, sel=False):
    """J: list of world points shoulder, elbow, wrist, flange, tcp"""
    s=[]
    if not ghost:
        s.append(cyl(iso,bx,by,bz,1.7,1.0,'#3A4366'))
        s.append(cyl(iso,bx,by,bz+1.0,1.4,1.3,col))
    sh,el,wr,fl,tcp=J
    base_top=(bx,by,bz+2.3)
    if ghost:
        gc='#8E92EC'
        for a,b,w in [(base_top,sh,30),(sh,el,24),(el,wr,18),(wr,fl,12),(fl,tcp,6)]:
            p,q=iso.p(*a),iso.p(*b)
            s.append(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{gc}" stroke-width="{w}" stroke-linecap="round" opacity=".32"/>')
            s.append(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{gc}" stroke-width="1.8" stroke-dasharray="5 3" opacity="1"/>')
        return '<g>'+''.join(s)+'</g>'
    if sel:
        for a,b,w in [(base_top,sh,30),(sh,el,24),(el,wr,18),(wr,fl,12)]:
            p,q=iso.p(*a),iso.p(*b)
            s.append(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="#A9ACF5" stroke-width="{w+9}" stroke-linecap="round" opacity=".55" filter="url(#selglow)"/>')
    s.append(seg(iso,base_top,sh,28,col))
    s.append(jointdot(iso,sh,15,'#4A5276'))
    s.append(seg(iso,sh,el,22,col))
    s.append(jointdot(iso,el,12,'#4A5276'))
    s.append(seg(iso,el,wr,16,col))
    s.append(jointdot(iso,wr,8,'#4A5276'))
    s.append(seg(iso,wr,fl,10,'#5A6288'))
    s.append(seg(iso,fl,tcp,5,'#9AA3C7','butt'))
    return f'<g opacity="{op}">'+''.join(s)+'</g>'

def extrude_profile(iso, prof, y0, y1, col, op=1, stroke='rgba(10,14,30,.45)'):
    """profile: list of (x,z) closed polygon; extrude along y from y0 to y1. Painter: back cap, edges, front cap"""
    s=[]
    n=len(prof)
    back=[(x,y0,z) for x,z in prof]; front=[(x,y1,z) for x,z in prof]
    s.append(f'<polygon points="{iso.pts(back)}" fill="{shade(col,.5)}" stroke="{stroke}" stroke-width=".6"/>')
    faces=[]
    for i in range(n):
        (x1,z1),(x2,z2)=prof[i],prof[(i+1)%n]
        # outward normal in xz for CCW profile
        nx,nz=(z2-z1),-(x2-x1)
        L=math.hypot(nx,nz) or 1; nx/=L; nz/=L
        light= .7+ .38*max(0,nz) + .12*max(0,nx)
        depth=(x1+x2)/2+ (z1+z2)/2*0.3
        q=[(x1,y0,z1),(x2,y0,z2),(x2,y1,z2),(x1,y1,z1)]
        faces.append((depth,f'<polygon points="{iso.pts(q)}" fill="{shade(col,min(1.3,light))}" stroke="{stroke}" stroke-width=".6"/>'))
    for _,f in sorted(faces): s.append(f)
    s.append(f'<polygon points="{iso.pts(front)}" fill="{shade(col,.74)}" stroke="{stroke}" stroke-width=".6"/>')
    return f'<g opacity="{op}">'+''.join(s)+'</g>'

def scene_defs():
    return '''<defs>
 <filter id="selglow" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="5"/></filter>
 <filter id="soft" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="10"/></filter>
 <radialGradient id="floorfade" cx="50%" cy="50%" r="60%"><stop offset="0" stop-color="#1E2750" stop-opacity=".9"/><stop offset="1" stop-color="#0A0E1C" stop-opacity="0"/></radialGradient>
 <linearGradient id="airail" x1="0" x2="1"><stop offset="0" stop-color="#8E92EC"/><stop offset="1" stop-color="#62D2F2"/></linearGradient>
</defs>'''

def cell_scene(iso, carriage_x=24, J=None, ghost=None, sel=True, extras='', pre=''):
    s=[scene_defs()]
    s.append(f'<ellipse cx="{iso.p(28,20)[0]:.0f}" cy="{iso.p(28,20)[1]:.0f}" rx="900" ry="420" fill="url(#floorfade)"/>')
    s.append(grid(iso,-6,62,-4,46,2))
    s.append(f'<polygon points="{iso.pts([(3,3,0),(53,3,0),(53,40,0),(3,40,0)])}" fill="none" stroke="rgba(255,178,36,.32)" stroke-width="1.5" stroke-dasharray="10 6"/>')
    for xx in range(2,56,6): s.append(box(iso,xx,2.2,0,.3,.3,5,'#59628A'))
    for yy in range(2,44,6): s.append(box(iso,1.6,yy,0,.3,.3,5,'#59628A'))
    s.append(f'<polygon points="{iso.pts([(1.7,2.35,.4),(56,2.35,.4),(56,2.35,4.8),(1.7,2.35,4.8)])}" fill="rgba(142,146,236,.05)" stroke="rgba(142,146,236,.18)" stroke-width="1"/>')
    s.append(f'<polygon points="{iso.pts([(1.75,2.35,.4),(1.75,44,.4),(1.75,44,4.8),(1.75,2.35,4.8)])}" fill="rgba(142,146,236,.04)" stroke="rgba(142,146,236,.18)" stroke-width="1"/>')
    s.append(pre)
    # back robot on pedestal
    s.append(box(iso,36.5,6.5,0,3,3,2.4,'#3A4366'))
    J2=[(38,8,6.2),(37,11.2,10.6),(35.2,14.4,9.8),(34.8,15.2,8.7),(34.6,15.6,7.6)]
    s.append(robot(iso,38,8,2.4,J2,col='#D9DCEA',sel=False))
    # fixture clamps far side
    for xx in (13,21,29,37):
        s.append(box(iso,xx,14.6,1.2,.9,.9,4.4,'#5560A0'))
        s.append(box(iso,xx-.2,14.6,5.6,1.3,1.9,.5,'#6B74A8'))
    # skid conveyor + car body
    s.append(box(iso,8,15.5,0,34,11,1.2,'#2F375C'))
    for xx in range(9,42,2): s.append(box(iso,xx,15.1,1.2,.6,11.8,.25,'#5B6590'))
    s.append(box(iso,10,16.6,1.45,30,1,.5,'#6E7799'))
    s.append(box(iso,10,24.2,1.45,30,1,.5,'#6E7799'))
    prof=[(11,2.2),(39.2,2.2),(39.8,4.3),(36.5,4.9),(31.5,5.4),(27.5,8.3),(18.5,8.5),(15.2,5.7),(11.6,5.3),(10.5,4.0)]
    s.append(extrude_profile(iso,prof,16.8,25.4,'#8C94BA'))
    yy=25.42
    s.append(f'<polygon points="{iso.pts([(19.2,yy,5.8),(26.6,yy,5.8),(26.6,yy,7.9),(18.9,yy,8.0)])}" fill="#121A36" opacity=".9"/>')
    s.append(f'<polygon points="{iso.pts([(27.6,yy,5.8),(30.8,yy,5.7),(27.2,yy,7.9)])}" fill="#121A36" opacity=".9"/>')
    s.append(f'<line x1="{iso.p(27.1,yy,5.8)[0]:.1f}" y1="{iso.p(27.1,yy,5.8)[1]:.1f}" x2="{iso.p(27.1,yy,2.4)[0]:.1f}" y2="{iso.p(27.1,yy,2.4)[1]:.1f}" stroke="rgba(10,14,30,.4)" stroke-width="1"/>')
    for wx in (15,35):
        pts=[(wx+2.1*math.cos(a),yy,2.2+2.1*math.sin(a)) for a in [i*math.pi/16 for i in range(17)]]
        s.append(f'<polygon points="{iso.pts(pts)}" fill="#0E1430"/>')
    # weld spots on roof edge (done=lavender, todo=white)
    for i,xx in enumerate(range(19,28,1)):
        c='#A9ACF5' if i<5 else 'rgba(231,233,247,.55)'
        p=iso.p(xx+.3,24.6,8.45); s.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="2.4" fill="{c}"/>')
    # track (7th axis) near side
    s.append(box(iso,3,33,0,46,4,.6,'#3B4570'))
    for xx in range(4,49,3): s.append(box(iso,xx,32.8,0,.5,4.4,.32,'#2E365A'))
    s.append(box(iso,3,33.6,.6,46,.35,.28,'#A3ABCC'))
    s.append(box(iso,3,36.0,.6,46,.35,.28,'#A3ABCC'))
    s.append(box(iso,3,37.6,0,26,.8,.8,'#262D4A'))
    s.append(box(iso,carriage_x-2.6,32.6,.88,5.2,4.8,.9,'#4A5683'))
    if ghost: s.append(ghost)
    if J is None:
        c=carriage_x
        J=[(c,34.9,6.6),(c-1.2,30.8,12.6),(c+.4,26.4,11.6),(c+.9,25.2,10.3),(c+1.1,24.7,9.0)]
    s.append(robot(iso,carriage_x,34.9,1.8,J,sel=sel))
    s.append(extras)
    return '\n'.join(s), J
