from kit import *
import math
VW,VH=1194,470
iso=Iso(640,40,19)
pts3=[(19+i*1.0+.3,24.6,8.45) for i in range(9)]
# waypoint path: approach + weld points
svg,J=cell_scene(iso,carriage_x=24,sel=True)
wp=[(17.0,27.5,12.0)]+[(x,y,z+0.9) for x,y,z in pts3]+[(29.5,27.0,12.5)]
P=[iso.p(*w) for w in wp]
spd=[.3,.5,.55,.6,.6,.62,.95,.6,.55,.5,.3]
def col(v):
    # lavender -> cyan -> amber
    if v<.6: return '#8E92EC'
    if v<.8: return '#62D2F2'
    return '#FFB224'
segs=''
for i in range(len(P)-1):
    a,b=P[i],P[i+1]
    if i==0 or i==len(P)-2:
        mx,my=(a[0]+b[0])/2,(a[1]+b[1])/2-40
        segs+=f'<path d="M{a[0]:.0f},{a[1]:.0f} Q{mx:.0f},{my:.0f} {b[0]:.0f},{b[1]:.0f}" fill="none" stroke="#A6ACCB" stroke-width="2" stroke-dasharray="6 4"/>'
    else:
        segs+=f'<line x1="{a[0]:.0f}" y1="{a[1]:.0f}" x2="{b[0]:.0f}" y2="{b[1]:.0f}" stroke="{col(spd[i])}" stroke-width="3.2"/>'
for i,p in enumerate(P):
    name='P0' if i==0 else ('P1' if i==len(P)-1 else f'W{i+3:02d}')
    sel=(name=='W08')
    tri=f'<line x1="{p[0]:.0f}" y1="{p[1]:.0f}" x2="{p[0]+16:.0f}" y2="{p[1]+8:.0f}" stroke="#FF5C6C" stroke-width="1.5"/><line x1="{p[0]:.0f}" y1="{p[1]:.0f}" x2="{p[0]-14:.0f}" y2="{p[1]+7:.0f}" stroke="#5FD38D" stroke-width="1.5"/><line x1="{p[0]:.0f}" y1="{p[1]:.0f}" x2="{p[0]:.0f}" y2="{p[1]+18:.0f}" stroke="#5B8CFF" stroke-width="1.5"/>'
    segs+=tri
    if sel:
        segs+=f'<circle cx="{p[0]:.0f}" cy="{p[1]:.0f}" r="13" fill="rgba(142,146,236,.25)" stroke="#A9ACF5" stroke-width="2"/>'
    segs+=f'<rect x="{p[0]-6:.0f}" y="{p[1]-6:.0f}" width="12" height="12" transform="rotate(45 {p[0]:.0f} {p[1]:.0f})" fill="{"#A9ACF5" if sel else "#0E1328"}" stroke="#E7E9F7" stroke-width="1.6"/>'
    if i in (0,3,5,7,len(P)-1) or sel:
        segs+=f'<text x="{p[0]+10:.0f}" y="{p[1]-14:.0f}" fill="{"#E0E1FC" if sel else "#A6ACCB"}" font-size="12" font-family="Inter" font-weight="{600 if sel else 400}">{name}</text>'
# singularity marker near W10
s=P[7]
segs+=f'<circle cx="{s[0]:.0f}" cy="{s[1]:.0f}" r="22" fill="none" stroke="#FFB224" stroke-width="1.5" stroke-dasharray="4 3"/>'
svg+=segs
selp=P[5]
vp=f'''<div class="vp" style="height:{VH}px;flex:none">
<svg class="scene" viewBox="0 0 {VW} {VH}" preserveAspectRatio="xMidYMid slice">{svg}</svg>
<div class="hud glass" style="left:14px;top:14px;padding:8px 12px;display:flex;gap:12px;align-items:center"><span style="color:var(--tx-1)">R01_焊接序列</span><span class="tag">11 点</span><span>路径着色：TCP 速度</span>
<span style="display:flex;align-items:center;gap:6px"><span style="width:90px;height:6px;border-radius:3px;background:linear-gradient(90deg,#8E92EC,#62D2F2,#FFB224)"></span><span class="mono" style="font-size:10.5px">0–1.5 m/s</span></span></div>
<div class="vtb glass" style="left:50%;top:14px;transform:translateX(-50%)"><div class="ib">{ic('mouse','s')}</div><div class="ib on">{ic('move','s')}</div><div class="ib">{ic('rotate','s')}</div><div class="sep"></div><div class="ib">{ic('target','s')}</div><div class="ib">{ic('path','s')}</div><div class="ib">{ic('frame','s')}</div><div class="sep"></div><span class="chip on" style="border:none">吸附：面法向</span></div>
<div class="callout glass" style="left:{s[0]+28:.0f}px;top:{s[1]+12:.0f}px;border-color:rgba(255,178,36,.45)"><div style="color:var(--warn);font-weight:600;display:flex;gap:6px;align-items:center">{ic('warn','s')}W10 接近腕部奇异</div><div style="font-size:11px;color:var(--tx-2)">J5 = 3.1° · 建议工具绕 Z 旋转 15°</div></div>
<div class="glass" style="position:absolute;left:{selp[0]-200:.0f}px;top:{selp[1]-96:.0f}px;display:flex;align-items:center;gap:2px;padding:4px">
<span style="padding:0 8px;font-size:11.5px;font-weight:600">W08</span><span class="tag" style="margin-right:4px">LIN</span><div style="width:1px;height:18px;background:var(--line-strong)"></div>
<span class="btn ghost" style="height:26px;padding:0 8px">移到此点</span><span class="btn ghost" style="height:26px;padding:0 8px">{ic('plus','s')}之后插入</span><span class="btn ghost" style="height:26px;padding:0 8px">翻转配置</span><span class="ib">{ic('dots','s')}</span></div>
</div>'''
# waypoint table
rows=[('P0','PTP','100%','Z50','2 120','ok'),('W04','LIN','1 200','FINE','2 260','ok'),('W05','LIN','1 200','FINE','2 290','ok'),('W06','LIN','1 200','FINE','2 340','warn'),('W07','LIN','1 200','FINE','2 380','ok'),
 ('W08','LIN','1 200','FINE','2 420','sel'),('W09','LIN','1 500','Z5','2 460','ok'),('W10','CIRC','800','FINE','2 500','warn'),('W11','LIN','1 200','FINE','2 540','ok'),('W12','LIN','1 200','FINE','2 580','ok'),('P1','PTP','100%','Z100','2 600','ok')]
tcol={'PTP':'#A6ACCB','LIN':'#8E92EC','CIRC':'#C792EA'}
tbl=''.join(f'''<div class="tr{' sel' if s=='sel' else ''}" style="display:grid;grid-template-columns:18px 44px 44px 1fr 44px 52px;gap:6px;padding:0 10px;height:30px">
<span style="color:{'var(--warn)' if s=='warn' else 'var(--ok)'}">{ic('warn' if s=='warn' else 'check','s')}</span><span class="nm mono">{n}</span><span class="mono" style="color:{tcol[t]};font-size:11px">{t}</span><span class="mono num" style="font-size:11.5px">{v}</span><span class="mono" style="font-size:11px;color:var(--tx-3)">{z}</span><span class="mono num" style="font-size:11px;color:var(--tx-3);text-align:right">{e}</span></div>''' for n,t,v,z,e,s in rows)
left=f'''<div class="panel" style="width:330px;border-right:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">路径点</div><div class="tab">程序</div><div class="tab">坐标系</div></div><div class="act"><div class="ib">{ic('plus','s')}</div><div class="ib">{ic('filter','s')}</div></div></div>
<div style="padding:10px;display:flex;gap:8px;align-items:center;border-bottom:1px solid var(--line-soft)"><span style="width:28px;height:28px;border-radius:8px;background:linear-gradient(135deg,#E98A3A,#B8621F);display:grid;place-items:center;color:#1a0d00">{ic('robot','s')}</span><div><div style="font-weight:600">R01_焊接序列</div><div style="font-size:11px;color:var(--tx-3)">R01 + E1 · TCP SG-02 · BASE_R01</div></div></div>
<div style="display:grid;grid-template-columns:18px 44px 44px 1fr 44px 52px;gap:6px;padding:6px 10px;font-size:10.5px;color:var(--tx-4)"><span></span><span>名称</span><span>类型</span><span>速度 mm/s</span><span>区域</span><span style="text-align:right">E1 mm</span></div>
<div class="tree">{tbl}</div>
<div style="padding:10px;border-top:1px solid var(--line-soft);display:flex;gap:6px;flex-wrap:wrap"><span class="btn" style="height:26px">{ic('target','s')}示教当前</span><span class="btn" style="height:26px">表面取点</span><span class="btn" style="height:26px">批量编辑</span></div>
</div>'''
# inspector
def jog(n,v,lo,hi,c='var(--accent)'):
    pct=(v-lo)/(hi-lo)*100
    return f'<div style="display:grid;grid-template-columns:22px 22px 1fr 22px 54px;gap:6px;align-items:center;height:26px"><span style="color:var(--tx-3);font-size:11.5px">{n}</span><span class="ib" style="width:22px;height:22px;background:var(--bg-raised)">−</span><div style="position:relative;height:6px;border-radius:3px;background:var(--bg-input);border:1px solid var(--line)"><div style="position:absolute;left:calc({pct:.0f}% - 5px);top:-5px;width:12px;height:14px;border-radius:3px;background:{c}"></div></div><span class="ib" style="width:22px;height:22px;background:var(--bg-raised)">+</span><span class="mono num" style="text-align:right;font-size:11.5px">{v:.1f}°</span></div>'
right=f'''<div class="panel" style="width:350px;border-left:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">W08 · 路径点</div><div class="tab">示教器</div></div><div class="act"><div class="ib">{ic('lock','s')}</div></div></div>
<div class="sec"><h4>位姿 <span class="r">参考 BASE_R01</span></h4>
<div class="row"><label>位置</label><div class="vec"><div class="fld num"><span class="ax" style="color:var(--x)">X</span>1 842.6</div><div class="fld num"><span class="ax" style="color:var(--y)">Y</span>-312.0</div><div class="fld num"><span class="ax" style="color:var(--z)">Z</span>1 655.1</div></div></div>
<div class="row"><label>姿态</label><div class="vec"><div class="fld num"><span class="ax" style="color:var(--x)">A</span>178.2</div><div class="fld num"><span class="ax" style="color:var(--y)">B</span>-2.5</div><div class="fld num"><span class="ax" style="color:var(--z)">C</span>-91.0</div></div></div>
<div class="row"><label>配置</label><div style="display:flex;gap:4px"><span class="chip on">前 · 上 · 不翻</span><span class="chip">前 · 上 · 翻</span><span class="chip">后…</span></div></div>
</div>
<div class="sec"><h4>运动</h4>
<div class="row"><label>类型</label><div style="display:flex;gap:2px;padding:2px;border-radius:7px;background:var(--bg-input);border:1px solid var(--line)"><span class="chip" style="border:none;background:transparent;flex:1;justify-content:center">PTP</span><span class="chip on" style="border:none;flex:1;justify-content:center">LIN</span><span class="chip" style="border:none;background:transparent;flex:1;justify-content:center">CIRC</span></div></div>
<div class="row"><label>速度</label><div class="fld num">1 200<span class="u">mm/s</span></div></div>
<div class="row"><label>加速度</label><div class="fld num">100<span class="u">%</span></div></div>
<div class="row"><label>逼近区</label><div class="fld">FINE<span class="u">{ic('chevd','s')}</span></div></div>
<div class="row"><label>E1</label><div class="fld num">2 420.0<span class="u">mm · 协调</span></div></div>
</div>
<div class="sec"><h4>点动 · 关节 <span class="r"><span class="chip on" style="height:18px">关节</span> <span class="chip" style="height:18px">笛卡尔</span> <span class="chip" style="height:18px">工具</span></span></h4>
{jog('J1',-28.6,-185,185)}{jog('J2',-68.2,-140,-5)}{jog('J3',92.4,-120,168)}{jog('J4',10.1,-350,350)}{jog('J5',-109.8,-125,125,'var(--warn)')}{jog('J6',2.4,-350,350)}
<div style="display:flex;gap:6px;margin-top:8px"><span class="chip">步长 1° </span><span class="chip">步长 0.1°</span><span class="chip on">连续</span><span style="margin-left:auto" class="btn" style="height:24px">{ic('target','s')}更新到此点</span></div></div>
<div class="sec"><h4>路径点事件</h4>
<div style="display:flex;flex-direction:column;gap:6px;font-size:11.5px">
<div style="display:flex;align-items:center;gap:8px;padding:6px 8px;border-radius:7px;background:var(--bg-input);border:1px solid var(--line)"><span style="color:#C792EA">{ic('signal','s')}</span>到达后 <span class="mono" style="color:#C792EA">R01_WeldReq := TRUE</span><span style="margin-left:auto;color:var(--tx-3)">脉冲 180 ms</span></div>
<div style="display:flex;align-items:center;gap:8px;padding:6px 8px;border-radius:7px;background:var(--bg-input);border:1px solid var(--line)"><span style="color:#62D2F2">{ic('clock','s')}</span>等待 <span class="mono" style="color:#C792EA">SG02_WeldDone</span><span style="margin-left:auto;color:var(--tx-3)">超时 2 s</span></div></div></div>
</div>'''
# timeline with plots
TW=1194-170  # plot width
T=10.0
def X(t): return t/T*TW
wtimes=[0,1.2,1.85,2.5,3.15,3.8,4.45,5.1,6.0,6.65,8.4]
names=['P0','W04','W05','W06','W07','W08','W09','W10','W11','W12','P1']
wp_track=''
for i,t in enumerate(wtimes):
    x=X(t); sel=names[i]=='W08'
    wp_track+=f'<div style="position:absolute;left:{x-6:.0f}px;top:9px;width:12px;height:12px;transform:rotate(45deg);background:{"#A9ACF5" if sel else "#151C37"};border:1.6px solid {"#fff" if sel else "#A6ACCB"}"></div><div class="mono" style="position:absolute;left:{x-14:.0f}px;top:24px;font-size:10px;color:{"#E0E1FC" if sel else "var(--tx-3)"};width:28px;text-align:center">{names[i]}</div>'
seg_track=''
for i in range(len(wtimes)-1):
    t=rows[i+1][1]; c=tcol[t]
    seg_track+=f'<div style="position:absolute;left:{X(wtimes[i])+2:.0f}px;width:{X(wtimes[i+1])-X(wtimes[i])-4:.0f}px;top:6px;bottom:6px;border-radius:4px;background:{c}26;border:1px solid {c}77;font-size:10px;color:{c};display:flex;align-items:center;justify-content:center" class="mono">{t}</div>'
# weld events
ev=''
for t in wtimes[1:10]:
    ev+=f'<div style="position:absolute;left:{X(t):.0f}px;width:{X(.18):.0f}px;top:8px;bottom:8px;background:#C792EA;border-radius:2px;opacity:.85"></div>'
def curve(f,h,col,w=1.6,dash=''):
    d='M'+' L'.join(f'{X(t):.1f},{h-f(t)*h:.1f}' for t in [i*T/400 for i in range(401)])
    return f'<path d="{d}" fill="none" stroke="{col}" stroke-width="{w}" {dash}/>'
H=150
jcols=['#FF8FA3','#7BD88F','#62D2F2','#F5D76E','#FFB224','#C792EA']
def jf(k):
    return lambda t: .5+.28*math.sin(t*.55+k)*math.cos(t*.21*k+.3)+.05*k-.12
plots=''.join(curve(jf(k),H,jcols[k],1.5) for k in range(6) if k!=4)
j5=lambda t: .2+ .62*math.exp(-((t-6.0)/.9)**2)+.05*math.sin(t)
plots+=f'<rect x="0" y="0" width="{TW}" height="{H*.08:.0f}" fill="rgba(255,178,36,.12)"/><line x1="0" x2="{TW}" y1="{H*.08:.0f}" y2="{H*.08:.0f}" stroke="rgba(255,178,36,.6)" stroke-dasharray="4 4"/>'
plots+=curve(j5,H,'#FFB224',2.4)
spdf=lambda t: (min(1,t/0.6)*.35 if t<1.2 else (.55 if t<5.8 else (.85 if t<6.0 else (.42 if t<6.65 else .3*max(0,(8.4-t)/1.75)+.1)))) if t<8.4 else .05
spd_plot=curve(lambda t:(spdf(t)),80,'#62D2F2',2)
spd_plot=f'<path d="M0,80 '+' L'.join(f'{X(t):.1f},{80-spdf(t)*80:.1f}' for t in [i*T/400 for i in range(401)])+f' L{TW},80 Z" fill="rgba(98,210,242,.12)"/>'+spd_plot+f'<line x1="0" x2="{TW}" y1="{80-.75*80}" y2="{80-.75*80}" stroke="rgba(255,79,100,.55)" stroke-dasharray="5 4"/><text x="{TW-120}" y="{80-.75*80-4}" fill="#FF8A98" font-size="10" font-family="Inter">工艺上限 1.2 m/s</text>'
e1f=lambda t: .15+.7*min(1,t/8.4)
e1_plot=curve(e1f,40,'#62D2F2',2)
ph=X(3.8)
ticks=''.join(f'<span class="mono" style="position:absolute;left:{X(i):.0f}px;top:4px;font-size:10px;color:var(--tx-4)">{i}.0s</span>' for i in range(11))
def lane(label,h,content,sub=''):
    return f'<div style="display:flex;height:{h}px;border-bottom:1px solid var(--line-soft)"><div style="width:170px;padding:6px 10px;border-right:1px solid var(--line-soft);color:var(--tx-2);font-size:12px">{label}<div style="font-size:10.5px;color:var(--tx-4)">{sub}</div></div><div style="position:relative;flex:1">{content}</div></div>'
legend=''.join(f'<div style="display:flex;align-items:center;gap:5px;font-size:10.5px;color:{"var(--warn)" if k==4 else "var(--tx-3)"}"><span style="width:10px;height:2px;background:{jcols[k]}"></span>J{k+1}</div>' for k in range(6))
bottom=f'''<div class="panel" style="flex:1;border-top:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">{ic('path','s')}路径时间线</div><div class="tab">{ic('gantt','s')}甘特</div><div class="tab">{ic('wave','s')}关节曲线</div></div>
<div style="display:flex;align-items:center;gap:4px;margin-left:20px"><div class="ib">{ic('stepb','s')}</div><div class="ib" style="background:var(--accent);color:#0C1030;width:30px">{ic('play','s')}</div><div class="ib">{ic('stepf','s')}</div><span class="mono num" style="margin-left:8px;font-size:13px">00:03.80</span><span class="mono" style="color:var(--tx-4)">/ 08.40 s</span><span class="pill" style="margin-left:8px">循环 W04–W12</span></div>
<div class="act"><span class="pill">缩放 {ic('search','s')} 1 s / 102 px</span></div></div>
<div style="display:flex;height:20px;border-bottom:1px solid var(--line-soft)"><div style="width:170px;border-right:1px solid var(--line-soft)"></div><div style="position:relative;flex:1">{ticks}</div></div>
<div style="position:relative;flex:1">
{lane('路径点',40,wp_track)}
{lane('运动段',30,seg_track)}
{lane('焊接事件',26,ev,'')}
{lane('E1 导轨',44,f'<svg width="{TW}" height="40" style="position:absolute;top:2px">{e1_plot}</svg>','2 120 → 2 600 mm')}
{lane('TCP 速度',84,f'<svg width="{TW}" height="80" style="position:absolute;top:2px">{spd_plot}</svg>','m/s')}
{lane('关节角 J1–J6',H+8,f'<svg width="{TW}" height="{H}" style="position:absolute;top:4px">{plots}</svg>','<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:3px;margin-top:6px">'+legend+'</div>')}
<div style="position:absolute;left:{170+ph:.0f}px;top:-20px;bottom:0;width:2px;background:var(--accent);box-shadow:0 0 8px rgba(142,146,236,.8)"></div>
<div class="glass" style="position:absolute;left:{170+ph+10:.0f}px;top:128px;padding:6px 10px;font-size:11px;line-height:1.6"><div class="mono">t = 3.80 s · W08</div><div class="mono" style="color:#62D2F2">v = 0.98 m/s</div><div class="mono" style="color:var(--warn)">J5 = −109.8°</div></div>
<div class="glass" style="position:absolute;left:{170+X(6.0)+14:.0f}px;top:236px;padding:4px 8px;font-size:11px;border-color:rgba(255,178,36,.5);color:var(--warn)">J5 余量 3.1° @ W10</div>
</div></div>'''
body=f'''<div class="app">{titlebar('机器人')}
<div class="body">{rail(['tree','lib','signal','logic','code','|',('shield','2'),'bell'],'tree')}{left}
<div style="flex:1;display:flex;flex-direction:column;min-width:0">{vp}{bottom}</div>{right}</div>
{statusbar([f'<span class="dot" style="background:var(--ok)"></span>就绪','路径 11 点 · 2 个警告','已选 W08'],['<span class="mono">Alt+拖动 = 沿路径滑动 · [ ] 上/下一点 · Ctrl+T 示教</span>','PDPS 键位',ic('globe','s')+'本地 · 离线'])}</div>'''
open('m4_path.html','w').write(page('机器人路径与时间线',body))
