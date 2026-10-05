from kit import *
LIGHT='''
:root{--bg-app:#EEF0F7;--bg-panel:#FFFFFF;--bg-raised:#F6F7FB;--bg-hover:#EEF0F8;--bg-active:#E6E7FA;--bg-input:#F7F8FC;
--line:#E1E4EE;--line-soft:#EAECF3;--line-strong:#CDD1E0;--tx-1:#171D38;--tx-2:#4C5373;--tx-3:#767C9C;--tx-4:#A1A6BE;
--accent:#5C60C2;--accent-hi:#474AA0;--accent-bg:rgba(92,96,194,.10);--accent-bg2:rgba(92,96,194,.18);
--ok:#178A57;--warn:#B86E00;--alarm:#D42A40;--info:#1F6FD1;--x:#D93848;--y:#2E9E5B;--z:#2F62D9;
--sh-2:0 6px 18px rgba(23,32,56,.10);--sh-3:0 18px 48px rgba(23,32,56,.16)}
html,body{background:var(--bg-app);color:var(--tx-1)}
.titlebar,.rail,.status{background:#FFFFFF;border-color:var(--line)}
.modes{background:#F1F2F8;border-color:var(--line)} .modes .m.on{background:#fff;box-shadow:0 1px 3px rgba(23,32,56,.12),inset 0 0 0 1px rgba(92,96,194,.35)}
.cmdk,.pill{background:#F4F5FA;border-color:var(--line)}
.vp{background:radial-gradient(120% 90% at 50% 35%,#FBFBFE 0%,#ECEEF6 60%,#DFE2EE 100%)}
.glass{background:rgba(255,255,255,.78);border:1px solid rgba(23,32,56,.08);box-shadow:0 6px 20px rgba(23,32,56,.10)}
.btn.pri{color:#fff} .chip.on{color:#3F43A0;background:rgba(92,96,194,.12);border-color:rgba(92,96,194,.45)} .tag.lav{color:#3F43A0;border-color:rgba(92,96,194,.45)} .chip{background:#F4F5FA} .tr.sel{background:var(--accent-bg)} kbd.k,.kb{background:#fff}
'''
VW,VH=1174,660
iso=Iso(560,90,15)
s=[]
s.append(grid(iso,-4,40,-4,34,2,col='rgba(23,32,56,.05)',mcol='rgba(23,32,56,.10)'))
# shadow
s.append(f'<ellipse cx="{iso.p(18,15)[0]:.0f}" cy="{iso.p(18,15)[1]:.0f}" rx="260" ry="120" fill="rgba(23,32,56,.08)" filter="url(#soft)"/>')
s.append('<defs><filter id="soft" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="14"/></filter></defs>')
L0='#9AA1BD'; L1='#8E92EC'; L2='#62B7E0'; L3='#E9A15A'
s.append(box(iso,6,8,0,26,14,1.4,L0,stroke='rgba(23,32,56,.25)'))       # base
s.append(box(iso,6,11,1.4,3,8,9.5,L0,stroke='rgba(23,32,56,.25)'))      # headstock
s.append(box(iso,29,11,1.4,3,8,9.5,L0,stroke='rgba(23,32,56,.25)'))     # tailstock
# J1 axis (along x at y=15,z=8.5)
a=iso.p(2,15,8.5); b=iso.p(36,15,8.5)
# tilting frame (link 1)
s.append(box(iso,9,11.5,7.4,20,7,1.2,L1,stroke='rgba(23,32,56,.3)'))
s.append(box(iso,9,10.5,7.4,1.2,9,3.0,L1,stroke='rgba(23,32,56,.3)'))
s.append(box(iso,27.8,10.5,7.4,1.2,9,3.0,L1,stroke='rgba(23,32,56,.3)'))
# turntable (link 2)
s.append(cyl(iso,19,15,8.6,4.6,1.0,L2))
# fixture on table (link 3 = tooling)
s.append(box(iso,16,12.5,9.6,6,5,0.6,L3,stroke='rgba(23,32,56,.3)'))
for (xx,yy) in [(16.3,12.8),(21.2,12.8),(16.3,16.6),(21.2,16.6)]:
    s.append(box(iso,xx,yy,10.2,.5,.6,2.2,'#C0773A',stroke='rgba(23,32,56,.3)'))
s.append(box(iso,17,13.5,10.2,4,3,1.2,'#B8BED6',stroke='rgba(23,32,56,.3)'))
# axes draw
s.append(f'<line x1="{a[0]:.0f}" y1="{a[1]:.0f}" x2="{b[0]:.0f}" y2="{b[1]:.0f}" stroke="#5C60C2" stroke-width="2" stroke-dasharray="10 5 2 5"/>')
c1=iso.p(19,15,4); c2=iso.p(19,15,17)
s.append(f'<line x1="{c1[0]:.0f}" y1="{c1[1]:.0f}" x2="{c2[0]:.0f}" y2="{c2[1]:.0f}" stroke="#1F8FC0" stroke-width="2" stroke-dasharray="10 5 2 5"/>')
# rotation arcs
pa=iso.p(34,15,8.5)
s.append(f'<path d="M{pa[0]-6:.0f},{pa[1]-36:.0f} A22 38 0 1 1 {pa[0]-18:.0f},{pa[1]+30:.0f}" fill="none" stroke="#5C60C2" stroke-width="2.4"/><polygon points="{pa[0]-24:.0f},{pa[1]+24:.0f} {pa[0]-10:.0f},{pa[1]+34:.0f} {pa[0]-24:.0f},{pa[1]+38:.0f}" fill="#5C60C2"/>')
pc=iso.p(19,15,15.5)
s.append(f'<ellipse cx="{pc[0]:.0f}" cy="{pc[1]:.0f}" rx="44" ry="18" fill="none" stroke="#1F8FC0" stroke-width="2.4" stroke-dasharray="200 40"/>')
# labels anchor
la=iso.p(36,15,8.5); lc=iso.p(19,15,17)
svg=''.join(s)
vp=f'''<div class="vp" style="height:{VH}px;flex:none">
<svg class="scene" viewBox="0 0 {VW} {VH}">{svg}</svg>
<div class="hud glass" style="left:14px;top:14px;padding:8px 12px;display:flex;gap:10px;align-items:center;color:var(--tx-2)"><span style="color:var(--tx-1);font-weight:600">PS-02 双轴变位机</span><span class="tag lav">用户自定义设备</span><span>2 自由度 · 4 连杆</span></div>
<div class="vtb glass" style="left:50%;top:14px;transform:translateX(-50%)"><div class="ib on">{ic('mouse','s')}</div><div class="ib">{ic('joint','s')}</div><div class="ib">{ic('link','s')}</div><div class="ib">{ic('frame','s')}</div><div class="sep"></div><span class="chip on" style="border:none">按连杆着色</span><span class="chip" style="border:none;background:transparent">X 光</span></div>
<div class="callout glass" style="left:{la[0]-30:.0f}px;top:{la[1]-110:.0f}px"><div style="font-weight:600;color:#5C60C2">J1 · 翻转 · 旋转</div><div class="mono" style="font-size:11px;color:var(--tx-2)">轴：两圆心 · −90° … +90°</div></div>
<div class="callout glass" style="left:{lc[0]+20:.0f}px;top:{lc[1]-30:.0f}px"><div style="font-weight:600;color:#1F8FC0">J2 · 回转 · 旋转</div><div class="mono" style="font-size:11px;color:var(--tx-2)">轴：面法向 · 连续 360°</div></div>
<div class="glass" style="position:absolute;left:14px;bottom:14px;padding:8px 12px;display:flex;gap:14px;font-size:11.5px;color:var(--tx-2)">
{''.join(f'<span style="display:flex;gap:6px;align-items:center"><span style="width:10px;height:10px;border-radius:3px;background:{c}"></span>{n}</span>' for c,n in [(L0,'L0 底座 (固定)'),(L1,'L1 翻转框'),(L2,'L2 回转台'),(L3,'L3 工装板')])}</div>
<div class="glass" style="position:absolute;right:14px;bottom:14px;padding:10px 12px;width:330px">
<div style="display:flex;align-items:center;gap:8px;margin-bottom:8px"><span style="font-weight:600;font-size:12px">实时测试</span><span class="tag" style="margin-left:auto">拖动滑块驱动 3D</span></div>
{''.join(f'<div style="display:grid;grid-template-columns:28px 1fr 60px;gap:8px;align-items:center;height:26px"><span class="mono" style="color:var(--tx-2)">{n}</span><div style="position:relative;height:6px;border-radius:3px;background:#E6E8F2"><div style="position:absolute;left:0;width:{p}%;height:100%;border-radius:3px;background:{c}"></div><div style="position:absolute;left:calc({p}% - 7px);top:-5px;width:14px;height:14px;border-radius:50%;background:#fff;border:2px solid {c}"></div></div><span class="mono num" style="text-align:right">{v}</span></div>' for n,p,v,c in [('J1',66,'+30.0°','#5C60C2'),('J2',25,'90.0°','#1F8FC0')])}</div>
</div>'''
# structure graph
def kn(x,y,t,sub,c,sel=False):
    return f'<div style="position:absolute;left:{x}px;top:{y}px;width:200px;padding:8px 10px;border-radius:10px;background:#fff;border:1px solid {"#5C60C2" if sel else "var(--line)"};box-shadow:{"0 0 0 3px rgba(92,96,194,.15)," if sel else ""}0 2px 6px rgba(23,32,56,.06);display:flex;gap:8px;align-items:center"><span style="width:10px;height:28px;border-radius:3px;background:{c}"></span><div><div style="font-weight:600;font-size:12px">{t}</div><div style="font-size:11px;color:var(--tx-3)">{sub}</div></div></div>'
def jn(x,y,t,sub,c,sel=False):
    return f'<div style="position:absolute;left:{x}px;top:{y}px;padding:4px 10px;border-radius:14px;background:{c}18;border:1.5px solid {c};font-size:11.5px;display:flex;gap:6px;align-items:center;{"box-shadow:0 0 0 3px "+c+"33;" if sel else ""}"><span style="color:{c}">{ic("joint","s")}</span><b>{t}</b><span style="color:var(--tx-3)">{sub}</span></div>'
graph=f'''<div style="position:relative;height:470px">
<svg width="300" height="470" style="position:absolute;left:0;top:0"><path d="M110,58 V96 M110,128 V166 M110,214 V252 M110,284 V322 M110,370 V400" stroke="#CDD1E0" stroke-width="2"/></svg>
{kn(12,12,'L0 · 底座','固定于世界 · 3 个几何体',L0)}
{jn(48,96,'J1','旋转 · 翻转','#5C60C2',True)}
{kn(12,168,'L1 · 翻转框','5 个几何体 · 质量 410 kg',L1)}
{jn(48,252,'J2','旋转 · 回转','#1F8FC0')}
{kn(12,324,'L2 · 回转台','2 个几何体',L2)}
{jn(48,400,'F1','固定 · 工装','#A1A6BE')}
</div>
<div style="margin:0 12px;padding:10px;border-radius:9px;background:var(--bg-raised);border:1px dashed var(--line-strong);color:var(--tx-3);font-size:11.5px;display:flex;gap:6px;align-items:center">{ic('plus','s')}从 3D 中选择几何体以创建连杆，或拖动到图上</div>'''
left=f'''<div class="panel" style="width:300px;border-right:1px solid var(--line)">
<div class="ph"><div class="tabs"><div class="tab on">运动链</div><div class="tab">几何体</div><div class="tab">TCP / 坐标系</div></div></div>
<div style="padding:10px 12px;display:flex;gap:6px;border-bottom:1px solid var(--line-soft)"><span class="chip on">树视图</span><span class="chip">图视图</span><span style="margin-left:auto" class="tag ok">{ic('check','s')} 链有效</span></div>
{graph}</div>'''
right=f'''<div class="panel" style="width:360px;border-left:1px solid var(--line)">
<div class="ph"><div class="tabs"><div class="tab on">关节 J1</div><div class="tab">驱动 / 信号</div></div></div>
<div class="sec"><h4>类型</h4><div style="display:grid;grid-template-columns:repeat(4,1fr);gap:4px">{''.join(f'<span class="chip{" on" if n=="旋转" else ""}" style="justify-content:center;height:28px">{n}</span>' for n in ['旋转','移动','固定','耦合'])}</div></div>
<div class="sec"><h4>轴定义</h4>
<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin-bottom:8px">{''.join(f'<span class="chip{" on" if n=="两圆心" else ""}" style="justify-content:center">{n}</span>' for n in ['两点','两圆心','面法向'])}</div>
<div class="row"><label>起点</label><div class="fld mono" style="font-size:11px">⊙ 圆边 · 头座 Ø320</div></div>
<div class="row"><label>终点</label><div class="fld mono" style="font-size:11px">⊙ 圆边 · 尾座 Ø320</div></div>
<div class="row"><label>方向</label><div class="vec"><div class="fld num"><span class="ax" style="color:var(--x)">X</span>1.000</div><div class="fld num"><span class="ax" style="color:var(--y)">Y</span>0.000</div><div class="fld num"><span class="ax" style="color:var(--z)">Z</span>0.000</div></div></div>
<div style="display:flex;gap:6px;margin-top:6px"><span class="btn" style="height:24px">反向</span><span class="btn" style="height:24px">在 3D 中重新拾取</span></div></div>
<div class="sec"><h4>限位与动力学</h4>
<div class="row"><label>下 / 上限</label><div style="display:grid;grid-template-columns:1fr 1fr;gap:4px"><div class="fld num">−90.0<span class="u">°</span></div><div class="fld num">+90.0<span class="u">°</span></div></div></div>
<div class="row"><label>Home</label><div class="fld num">0.0<span class="u">°</span></div></div>
<div class="row"><label>最大速度</label><div class="fld num">60<span class="u">°/s</span></div></div>
<div class="row"><label>加速度</label><div class="fld num">120<span class="u">°/s²</span></div></div></div>
<div class="sec"><h4>信号绑定</h4>
<div style="font-size:11.5px;display:flex;flex-direction:column;gap:6px">
<div style="display:flex;gap:8px;align-items:center;padding:6px 8px;border-radius:7px;background:var(--bg-input);border:1px solid var(--line)"><span class="tag">AI</span><span class="mono">PS02_J1_SetPos</span><span style="margin-left:auto;color:var(--tx-3)">→ 目标位置</span></div>
<div style="display:flex;gap:8px;align-items:center;padding:6px 8px;border-radius:7px;background:var(--bg-input);border:1px solid var(--line)"><span class="tag">DO</span><span class="mono">PS02_J1_InPos</span><span style="margin-left:auto;color:var(--tx-3)">← |误差| < 0.1°</span></div></div></div>
</div>'''
poses=[('HOME','0.0°','0.0°','原点',True),('LOAD','−90.0°','0.0°','上料',False),('WELD_A','+30.0°','90.0°','焊接 A 面',False),('WELD_B','+30.0°','270.0°','焊接 B 面',False)]
pt=''.join(f'<div style="display:grid;grid-template-columns:20px 110px 90px 90px 1fr 210px;gap:10px;align-items:center;height:34px;padding:0 14px;border-bottom:1px solid var(--line-soft);{"background:var(--accent-bg);" if n=="WELD_A" else ""}"><span style="color:var(--tx-3)">{ic("pin","s") if h else ic("target","s")}</span><b class="mono">{n}</b><span class="mono num">{a}</span><span class="mono num">{b}</span><span style="color:var(--tx-2)">{d}</span><span style="display:flex;gap:4px"><span class="btn" style="height:24px">跳转</span><span class="btn ghost" style="height:24px">过渡预览 ▸</span><span class="btn ghost" style="height:24px">更新</span></span></div>' for n,a,b,d,h in poses)
bottom=f'''<div class="panel" style="flex:1;border-top:1px solid var(--line)">
<div class="ph"><div class="tabs"><div class="tab on">{ic('target','s')}姿态</div><div class="tab">{ic('link','s')}耦合 / 公式</div><div class="tab">{ic('shield','s')}运动学检查</div></div><div class="act"><span class="btn" style="height:24px">{ic('plus','s')}由当前创建姿态</span></div></div>
<div style="display:grid;grid-template-columns:20px 110px 90px 90px 1fr 210px;gap:10px;padding:6px 14px;font-size:11px;color:var(--tx-3);border-bottom:1px solid var(--line)"><span></span><span>名称</span><span>J1 翻转</span><span>J2 回转</span><span>说明</span><span></span></div>
{pt}</div>'''
body=f'''<div class="app">{titlebar('运动学',crumb=('设备库 · 本项目','PS-02 双轴变位机'))}
<div class="body">{rail(['tree','lib','signal','logic','code','|','shield','bell'],'tree')}{left}
<div style="flex:1;display:flex;flex-direction:column;min-width:0">{vp}{bottom}</div>{right}</div>
{statusbar([f'<span class="dot" style="background:var(--ok)"></span>运动链有效 · 2 自由度','已选 J1'],['浅色主题','<span class="mono">J 新建关节 · L 新建连杆 · P 记录姿态</span>',ic('globe','s')+'本地 · 离线'])}</div>'''
html=page('运动学编辑器 · 浅色',body,LIGHT).replace('assets/snippin_logo_small_dark.svg','assets/snippin_logo_small.svg')
open('m6_kin.html','w').write(html)
