from kit import *
VW,VH=1234,742
iso=Iso(600,128,12.2)
svg,J=cell_scene(iso)
tcp=iso.p(*J[4]); fl=iso.p(*J[3])
# gizmo at TCP
gx,gy=tcp
giz=f'''<g transform="translate({gx:.0f},{gy:.0f})">
<circle r="38" fill="none" stroke="rgba(91,140,255,.55)" stroke-width="2"/>
<ellipse rx="38" ry="14" fill="none" stroke="rgba(255,92,108,.5)" stroke-width="2" transform="rotate(-30)"/>
<line x1="0" y1="0" x2="58" y2="30" stroke="#FF5C6C" stroke-width="3"/><polygon points="64,33 52,33 58,24" fill="#FF5C6C" transform="rotate(0)"/>
<line x1="0" y1="0" x2="-52" y2="28" stroke="#5FD38D" stroke-width="3"/><polygon points="-58,31 -46,32 -51,22" fill="#5FD38D"/>
<line x1="0" y1="0" x2="0" y2="-60" stroke="#5B8CFF" stroke-width="3"/><polygon points="-6,-58 6,-58 0,-70" fill="#5B8CFF"/>
<rect x="-6" y="-6" width="12" height="12" rx="2" fill="#E7E9F7" stroke="#0A0E1A"/>
</g>'''
# near-miss
nm=iso.p(*J[2])
near=f'''<g><circle cx="{nm[0]:.0f}" cy="{nm[1]:.0f}" r="26" fill="rgba(255,178,36,.10)" stroke="#FFB224" stroke-width="1.5" stroke-dasharray="5 4"/>
<line x1="{nm[0]+18:.0f}" y1="{nm[1]-18:.0f}" x2="{nm[0]+70:.0f}" y2="{nm[1]-70:.0f}" stroke="#FFB224" stroke-width="1"/></g>'''
# measure on track
a=iso.p(3,38.6,.7); b=iso.p(24,38.6,.7)
meas=f'''<g><line x1="{a[0]:.0f}" y1="{a[1]:.0f}" x2="{b[0]:.0f}" y2="{b[1]:.0f}" stroke="#62D2F2" stroke-width="1.5"/>
<circle cx="{a[0]:.0f}" cy="{a[1]:.0f}" r="3.5" fill="#62D2F2"/><circle cx="{b[0]:.0f}" cy="{b[1]:.0f}" r="3.5" fill="#62D2F2"/></g>'''
# path preview of upcoming weld points
pts=[iso.p(x+.3,24.6,8.45+.9) for x in range(19,28)]
path='<polyline points="'+' '.join(f'{p[0]:.0f},{p[1]-10:.0f}' for p in pts)+'" fill="none" stroke="#A9ACF5" stroke-width="1.5" stroke-dasharray="3 3" opacity=".8"/>'
svg+=path+near+meas+giz
mx=(a[0]+b[0])/2; my=(a[1]+b[1])/2

tree_rows=[
 (0,'▾','layers','站点 OP30 · 侧围焊接','',''),
 (1,'▾','cube','资源','<span>5</span>',''),
 (2,'▾','robot','R01 · 6 轴机器人 KR-210','<span class="tag lav">7 轴</span>','sel'),
 (3,'▸','track','E1 · 线性导轨 7 m','',''),
 (3,'▸','gripper','SG-02 伺服焊枪 · TCP','',''),
 (3,' ','frame','BASE_R01','',''),
 (2,'▸','robot','R02 · 6 轴机器人 (固定)','',''),
 (2,'▸','cube','FX-30 焊装夹具 · 4 夹爪','<span class="tag warn">近碰</span>',''),
 (2,'▸','cube','SK-01 滑橇输送','',''),
 (1,'▾','file','工件','',''),
 (2,' ','cube','BIW_SIDE_L.step','<span>1.2M 面</span>',''),
 (1,'▾','path','操作','',''),
 (2,'▾','path','R01_焊接序列','<span>24 点</span>',''),
 (3,' ','target','接近 · PTP','',''),
 (3,' ','target','W01–W12 · LIN','<span class="tag ok">可达</span>',''),
 (3,' ','target','退出 · CIRC','',''),
 (2,'▸','path','R02_涂胶序列','',''),
 (1,'▸','logic','逻辑块','<span>6</span>',''),
 (1,'▸','signal','信号','<span>148</span>',''),
]
tree=''.join(f'<div class="tr {c}" style="padding-left:{10+d*14}px"><span class="tw">{t}</span><span class="ic">{ic(i,"s")}</span><span class="nm">{n}</span><span class="meta">{m}</span></div>' for d,t,i,n,m,c in tree_rows)

def jbar(name,val,lo,hi,warn=False):
    pct=(val-lo)/(hi-lo)*100
    col='var(--warn)' if warn else 'var(--accent)'
    return f'''<div style="display:grid;grid-template-columns:26px 1fr 62px;align-items:center;gap:8px;height:24px">
<span style="color:var(--tx-3);font-size:11.5px">{name}</span>
<div style="position:relative;height:6px;border-radius:3px;background:var(--bg-input);border:1px solid var(--line)">
<div style="position:absolute;left:0;top:0;bottom:0;width:{pct:.0f}%;border-radius:3px;background:{col};opacity:.85"></div>
<div style="position:absolute;left:calc({pct:.0f}% - 1px);top:-4px;width:2px;height:12px;background:#fff;border-radius:1px"></div>
<div style='position:absolute;right:0;top:-1px;bottom:-1px;width:6%;background:repeating-linear-gradient(45deg,rgba(255,178,36,.4) 0 2px,transparent 2px 4px)'></div><div style='position:absolute;left:0;top:-1px;bottom:-1px;width:6%;background:repeating-linear-gradient(45deg,rgba(255,178,36,.4) 0 2px,transparent 2px 4px)'></div>
</div><span class="mono num" style="text-align:right;font-size:11.5px;color:{'var(--warn)' if warn else 'var(--tx-1)'}">{val:.1f}°</span></div>'''

joints=''.join([jbar('J1',-32.4,-185,185),jbar('J2',-71.0,-140,-5),jbar('J3',96.2,-120,168),jbar('J4',12.5,-350,350),jbar('J5',-118.6,-125,125,True),jbar('J6',4.0,-350,350)])

insp=f'''
<div class="ph"><div class="tabs"><div class="tab on">属性</div><div class="tab">可达性</div><div class="tab">注释</div></div><div class="act"><div class="ib">{ic('pin','s')}</div><div class="ib">{ic('dots','s')}</div></div></div>
<div class="sec" style="display:flex;align-items:center;gap:10px">
 <div style="width:34px;height:34px;border-radius:9px;background:linear-gradient(135deg,#E98A3A,#B8621F);display:grid;place-items:center;color:#1a0d00">{ic('robot')}</div>
 <div><div style="font-weight:600;font-size:13px">R01 · KR-210 R2700</div><div style="color:var(--tx-3);font-size:11.5px">6 轴 + 外部轴 E1 · 控制器 KRC-sim</div></div>
 <span class="tag ok" style="margin-left:auto">运行中</span></div>
<div class="sec"><h4>变换 <span class="r">相对 · 世界</span></h4>
 <div class="row"><label>位置</label><div class="vec"><div class="fld num"><span class="ax" style="color:var(--x)">X</span>4 120.0</div><div class="fld num"><span class="ax" style="color:var(--y)">Y</span>-1 850.0</div><div class="fld num"><span class="ax" style="color:var(--z)">Z</span>0.0</div></div></div>
 <div class="row"><label>旋转</label><div class="vec"><div class="fld num"><span class="ax" style="color:var(--x)">Rx</span>0.0</div><div class="fld num"><span class="ax" style="color:var(--y)">Ry</span>0.0</div><div class="fld num"><span class="ax" style="color:var(--z)">Rz</span>90.0</div></div></div>
</div>
<div class="sec"><h4>外部轴 E1 · 线性导轨 <span class="r mono">0 – 7 000 mm</span></h4>
 <div style="position:relative;height:30px;margin:2px 0 6px">
  <div style="position:absolute;left:0;right:0;top:13px;height:4px;border-radius:2px;background:var(--bg-input);border:1px solid var(--line)"></div>
  <div style="position:absolute;left:0;width:33%;top:13px;height:4px;border-radius:2px;background:var(--accent)"></div>
  <div style="position:absolute;left:calc(33% - 8px);top:7px;width:16px;height:16px;border-radius:50%;background:#fff;box-shadow:0 0 0 4px rgba(142,146,236,.3)"></div>
  <div style="position:absolute;left:52%;top:9px;width:2px;height:12px;background:var(--tx-3)"></div><div style="position:absolute;left:49%;top:-4px;font-size:10px;color:var(--tx-3)">W06</div>
 </div>
 <div class="row"><label>位置</label><div class="fld num">2 340.0<span class="u">mm</span></div></div>
 <div class="row"><label>同步方式</label><div class="fld">与 TCP 协调运动<span class="u">{ic('chevd','s')}</span></div></div>
</div>
<div class="sec"><h4>关节 <span class="r">度 · 软限位</span></h4>{joints}</div>
<div class="sec"><h4>运动参数</h4>
 <div class="row"><label>速度</label><div class="fld num">1 200<span class="u">mm/s</span></div></div>
 <div class="row"><label>逼近区</label><div class="fld">FINE<span class="u">{ic('chevd','s')}</span></div></div>
</div>
<div style="margin:10px 12px;padding:10px 12px;border-radius:9px;background:rgba(255,178,36,.07);border:1px solid rgba(255,178,36,.3)">
 <div style="display:flex;gap:8px;align-items:center;color:var(--warn);font-weight:600;font-size:12px">{ic('warn','s')}规则检查 · J5 接近软限位</div>
 <div style="color:var(--tx-2);font-size:11.5px;margin:4px 0 8px">W06 处 J5 = −118.6°，余量 6.4°。将 E1 前移 120 mm 可将余量提升至 21°。</div>
 <div style="display:flex;gap:6px"><span class="btn" style="height:24px">{ic('eye','s')}预览</span><span class="btn ghost" style="height:24px">定位到 W06</span><span class="btn ghost" style="height:24px;margin-left:auto">忽略</span></div>
</div>
'''

# timeline
def blk(l,w,txt,col,sel=False,hatch=False):
    bg=f'repeating-linear-gradient(135deg,{col}33 0 4px,{col}18 4px 8px)' if hatch else f'{col}2b'
    b=f'1px solid {col}{"" if sel else "88"}'
    sh='box-shadow:0 0 0 2px rgba(142,146,236,.5);' if sel else ''
    return f'<div style="position:absolute;left:{l}px;width:{w}px;top:5px;bottom:5px;border-radius:5px;background:{bg};border:{b};{sh}font-size:11px;color:var(--tx-1);padding:0 7px;display:flex;align-items:center;overflow:hidden;white-space:nowrap">{txt}</div>'
tracks=[
 ('robot','R01 焊接',[blk(0,90,'接近 PTP','#8E92EC'),blk(92,300,'W01–W12 · LIN · 12 焊点','#8E92EC',True),blk(394,70,'退出','#8E92EC'),blk(500,250,'W13–W24','#8E92EC')]),
 ('track','E1 导轨',[blk(0,160,'0 → 2 340 mm','#62D2F2'),blk(300,180,'2 340 → 3 900','#62D2F2'),blk(560,120,'→ 4 800','#62D2F2')]),
 ('robot','R02 涂胶',[blk(40,260,'涂胶 CIRC ×4','#C792EA'),blk(330,30,'等待','#6E769C',hatch=True),blk(370,240,'涂胶 LIN','#C792EA')]),
 ('cube','FX-30 夹具',[blk(0,40,'夹紧','#7BD88F'),blk(720,40,'松开','#7BD88F')]),
]
def wave(points,col):
    # digital wave: list of (x,state)
    d=''; y1,y0=6,22
    prev=None
    for x,s in points:
        y=y1 if s else y0
        if prev is None: d+=f'M{x},{y}'
        else: d+=f' H{x} V{y}'
        prev=y
    d+=' H860'
    return f'<svg width="860" height="28" style="position:absolute;left:0;top:0"><path d="{d}" fill="none" stroke="{col}" stroke-width="1.6"/></svg>'
sig=[('signal','DI  FX30_Clamped',wave([(0,0),(40,1),(720,0)],'#C792EA')),('signal','DO  R01_WeldReq',wave([(0,0),(100,1),(118,0),(140,1),(158,0),(180,1),(198,0),(220,1),(238,0),(260,1),(278,0),(300,1),(318,0)],'#C792EA'))]
rows=''
for icn,name,bl in tracks:
    rows+=f'<div style="display:flex;height:34px;border-bottom:1px solid var(--line-soft)"><div style="width:170px;display:flex;align-items:center;gap:7px;padding:0 10px;color:var(--tx-2);border-right:1px solid var(--line-soft)">{ic(icn,"s")}{name}</div><div style="position:relative;flex:1">{"".join(bl)}</div></div>'
for icn,name,wv in sig:
    rows+=f'<div style="display:flex;height:28px;border-bottom:1px solid var(--line-soft)"><div class="mono" style="width:170px;display:flex;align-items:center;gap:7px;padding:0 10px;color:var(--tx-3);font-size:11px;border-right:1px solid var(--line-soft)">{name}</div><div style="position:relative;flex:1">{wv}</div></div>'
ticks=''.join(f'<span style="position:absolute;left:{i*86}px;top:6px;font-size:10.5px;color:var(--tx-4)" class="mono">{i*6}s</span><span style="position:absolute;left:{i*86}px;bottom:0;height:5px;width:1px;background:var(--line-strong)"></span>' for i in range(11))
bottom=f'''<div class="panel" style="height:262px;border-top:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">{ic('clock','s')}时间线</div><div class="tab">{ic('gantt','s')}甘特 / 节拍</div><div class="tab">{ic('table','s')}信号表</div><div class="tab">{ic('shield','s')}碰撞 <span class="tag warn">1</span></div><div class="tab">{ic('code','s')}Python</div></div>
<div style="display:flex;align-items:center;gap:4px;margin-left:24px">
 <div class="ib">{ic('stepb','s')}</div><div class="ib" style="background:var(--accent);color:#0C1030;width:30px">{ic('pause','s')}</div><div class="ib">{ic('stepf','s')}</div><div class="ib">{ic('stop','s')}</div>
 <span class="mono num" style="margin-left:10px;font-size:13px">00:12.48</span><span style="color:var(--tx-4)" class="mono">/ 54.00 s</span>
 <span class="pill" style="margin-left:10px">1.0×</span><span class="pill">{ic('zap','s')}确定性步长 4 ms</span></div>
<div class="act"><span class="pill" style="color:var(--ok);border-color:rgba(63,208,139,.35)">节拍 54.0 s · 目标 56.0 s</span></div></div>
<div style="display:flex;height:22px;border-bottom:1px solid var(--line-soft)"><div style="width:170px;border-right:1px solid var(--line-soft)"></div><div style="position:relative;flex:1">{ticks}</div></div>
<div style="position:relative;flex:1">{rows}
<div style="position:absolute;left:{170+178}px;top:-22px;bottom:0;width:2px;background:var(--accent);box-shadow:0 0 8px rgba(142,146,236,.8)"><div style="position:absolute;top:0;left:-6px;width:14px;height:10px;background:var(--accent);clip-path:polygon(0 0,100% 0,50% 100%)"></div></div>
<div style="position:absolute;left:{170+680}px;top:0;bottom:0;width:0;border-left:1px dashed rgba(255,178,36,.6)"></div>
</div></div>'''

vp=f'''<div class="vp">
<svg class="scene" viewBox="0 0 {VW} {VH}" preserveAspectRatio="xMidYMid slice">{svg}</svg>
<div class="hud glass" style="left:14px;top:14px;padding:8px 12px;display:flex;gap:14px;align-items:center">
 <span style="display:flex;gap:6px;align-items:center;color:var(--tx-1)">{ic('eye','s')}透视 · 着色+边线</span><span style="width:1px;height:14px;background:var(--line-strong)"></span>
 <span class="mono num">仿真 00:12.48</span><span class="mono num" style="color:var(--ok)">60 fps</span><span class="mono num">2.4M 三角面</span></div>
<div class="vtb glass" style="left:50%;top:14px;transform:translateX(-50%)">
 <div class="ib on">{ic('mouse','s')}</div><div class="ib">{ic('move','s')}</div><div class="ib">{ic('rotate','s')}</div><div class="sep"></div>
 <div class="ib">{ic('ruler','s')}</div><div class="ib">{ic('section','s')}</div><div class="ib">{ic('target','s')}</div><div class="sep"></div>
 <div class="ib">{ic('shield','s')}</div><div class="ib">{ic('layers','s')}</div><div class="ib">{ic('expand','s')}</div></div>
<svg style="position:absolute;right:20px;top:16px" width="96" height="104" viewBox="0 0 96 104">
 <polygon points="48,8 86,30 48,52 10,30" fill="#2A3360" stroke="#4A5A94"/><polygon points="10,30 48,52 48,96 10,74" fill="#1F2750" stroke="#4A5A94"/><polygon points="48,52 86,30 86,74 48,96" fill="#252E5A" stroke="#4A5A94"/>
 <text x="48" y="34" fill="#C5C7F8" font-size="11" text-anchor="middle" font-family="Noto Sans CJK SC">上</text><text x="29" y="68" fill="#A6ACCB" font-size="11" text-anchor="middle" font-family="Noto Sans CJK SC">前</text><text x="67" y="68" fill="#A6ACCB" font-size="11" text-anchor="middle" font-family="Noto Sans CJK SC">右</text>
 <circle cx="86" cy="30" r="3" fill="#8E92EC"/></svg>
<div class="callout glass" style="left:{nm[0]+70:.0f}px;top:{nm[1]-96:.0f}px;border-color:rgba(255,178,36,.45)"><div style="color:var(--warn);font-weight:600;display:flex;gap:6px;align-items:center">{ic('warn','s')}近碰 8.2 mm</div><div style="color:var(--tx-2);font-size:11px">R01 腕部 A5 ↔ 侧围 B 柱 · 阈值 10 mm</div></div>
<div class="callout" style="left:{mx-70:.0f}px;top:{my+8:.0f}px;background:rgba(98,210,242,.14);border:1px solid rgba(98,210,242,.5);color:#BDEBFA" ><span class="mono num">E1 = 2 340.0 mm</span></div>
<div class="glass" style="position:absolute;left:{iso.p(*J[1])[0]-330:.0f}px;top:{iso.p(*J[1])[1]-64:.0f}px;display:flex;align-items:center;gap:2px;padding:4px">
 <span style="padding:0 8px;font-size:11.5px;color:var(--tx-2)">R01 · TCP</span><div class="sep" style="width:1px;height:18px;background:var(--line-strong)"></div>
 <span class="btn ghost" style="height:26px;padding:0 8px">{ic('plus','s')}示教点</span><span class="btn ghost" style="height:26px;padding:0 8px">{ic('joint','s')}关节点动</span><span class="btn ghost" style="height:26px;padding:0 8px">{ic('target','s')}可达性</span><span class="ib">{ic('dots','s')}</span></div>
<div class="glass" style="position:absolute;left:14px;bottom:14px;display:flex;align-items:center;gap:2px;padding:4px">
 <span style="padding:0 8px;color:var(--tx-3);font-size:11px;display:flex;gap:5px;align-items:center">{ic('filter','s')}选择</span>
 {''.join(f'<span class="chip{" on" if n=="组件" else ""}" style="border:none;{"" if n=="组件" else "background:transparent"}">{n}</span>' for n in ['组件','连杆','面','边','点','坐标系','路径点'])}
</div>
<div class="glass" style="position:absolute;right:14px;bottom:14px;padding:6px 10px;display:flex;gap:10px;align-items:center;font-size:11px;color:var(--tx-3)"><span class="mono">捕捉 1 mm · 1°</span><span style="width:60px;height:4px;border:1px solid var(--tx-3);border-top:none"></span><span class="mono">1 m</span></div>
</div>'''

body=f'''<div class="app">{titlebar('机器人')}
<div class="body">{rail(['tree','lib','signal','logic','code','|',('shield','1'),'bell'],'tree')}
<div class="panel" style="width:300px;border-right:1px solid var(--line-soft)">
 <div class="ph"><div class="tabs"><div class="tab on">装配树</div><div class="tab">操作</div><div class="tab">资源库</div></div><div class="act"><div class="ib">{ic('plus','s')}</div><div class="ib">{ic('dots','s')}</div></div></div>
 <div class="search">{ic('search','s')}筛选对象…<span style="margin-left:auto" class="kb">/</span></div>
 <div class="chips"><span class="chip on">全部</span><span class="chip">机器人</span><span class="chip">夹具</span><span class="chip">工件</span><span class="chip">已修改</span></div>
 <div class="tree">{tree}</div>
</div>
<div style="flex:1;display:flex;flex-direction:column;min-width:0">{vp}{bottom}</div>
<div class="panel" style="width:340px;border-left:1px solid var(--line-soft)">{insp}</div>
</div>
{statusbar([f'<span class="dot" style="background:var(--ok)"></span>就绪',ic('cpu','s')+'物理 4 ms · 碰撞 0.8 ms','已选 1 · R01'],['<span class="mono">X 4 126.3  Y -1 412.0  Z 1 893.5</span>','单位 mm · deg','PDPS 键位',ic('globe','s')+'本地 · 离线'])}
</div>'''
open('m1_workspace.html','w').write(page('主工作区',body))
