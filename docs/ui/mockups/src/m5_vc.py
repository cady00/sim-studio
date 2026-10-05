from kit import *
import math, random
random.seed(4)
# PIP scene
iso=Iso(600,128,12.2)
svg,J=cell_scene(iso,sel=False)
hl=iso.p(37.4,15.0,5.6)
svg+=f'<circle cx="{hl[0]:.0f}" cy="{hl[1]:.0f}" r="20" fill="rgba(142,146,236,.18)" stroke="#A9ACF5" stroke-width="1.5"/>'
pip=f'''<div style="position:relative;height:250px;border-bottom:1px solid var(--line-soft);background:radial-gradient(120% 90% at 50% 35%,#1B2346 0%,#11172E 55%,#0A0E1C 100%);overflow:hidden">
<svg viewBox="170 70 1000 625" width="400" height="250" style="position:absolute;inset:0">{svg}</svg>
<div class="glass" style="position:absolute;left:10px;top:10px;padding:5px 9px;font-size:11px;display:flex;gap:6px;align-items:center">{ic('eye','s')}3D 联动 · 跟随所选信号</div>
<div class="glass" style="position:absolute;left:{(hl[0]-170)*.4-190:.0f}px;top:{(hl[1]-70)*.4-44:.0f}px;padding:4px 8px;font-size:11px"><span class="mono" style="color:#C792EA">FX30_C4_Closed</span> → 夹爪 C4</div>
<div class="ib glass" style="position:absolute;right:10px;top:10px;width:28px;height:28px">{ic('expand','s')}</div></div>'''
def spark(vals,w=110,h=18,col='#A6ACCB',digital=False):
    n=len(vals)
    if digital:
        d=f'M0,{h-2 if not vals[0] else 3}'
        for i,v in enumerate(vals):
            x=i*w/(n-1); y=3 if v else h-2
            d+=f' H{x:.1f} V{y}'
        d+=f' H{w}'
    else:
        d='M'+' L'.join(f'{i*w/(n-1):.1f},{h-2-v*(h-5):.1f}' for i,v in enumerate(vals))
    return f'<svg width="{w}" height="{h}"><path d="{d}" fill="none" stroke="{col}" stroke-width="1.4"/></svg>'
def dig(pattern): return [int(c) for c in pattern]
rows=[
 ('in','PLC_StartClamp','%Q100.0','Bool','FX-30 · 逻辑块 夹紧序列','TRUE','', dig('000001111111111111'),'12.31 s',''),
 ('out','Skid_InPos','%I100.3','Bool','SK-01 · 到位传感器','TRUE','', dig('001111111111111111'),'11.02 s',''),
 ('out','FX30_C1_Closed','%I101.0','Bool','FX-30 · 夹爪 C1','TRUE','', dig('000000011111111111'),'12.42 s',''),
 ('out','FX30_C2_Closed','%I101.1','Bool','FX-30 · 夹爪 C2','TRUE','', dig('000000001111111111'),'12.45 s',''),
 ('out','FX30_C3_Closed','%I101.2','Bool','FX-30 · 夹爪 C3','TRUE','', dig('000000001111111111'),'12.44 s',''),
 ('out','FX30_C4_Closed','%I101.3','Bool','FX-30 · 夹爪 C4','FALSE','', dig('000000000000000000'),'—','warn'),
 ('out','FX30_Clamped','%I101.7','Bool','FX-30 · 逻辑输出','FALSE','', dig('000000000000000000'),'—',''),
 ('in','R01_Start','%Q110.0','Bool','R01 · 程序启动','FALSE','强制 FALSE', dig('000000000000000000'),'09.80 s','force'),
 ('in','R01_ProgNo','%QB111','Int','R01 · 程序号','12','', [ .5]*18,'08.12 s',''),
 ('out','R01_E1_ActPos','%ID120','Real','E1 · 实际位置 mm','2 340.0','', [min(1,i/12) for i in range(18)],'连续',''),
 ('out','R01_InHome','%I112.0','Bool','R01 · Home 区','FALSE','', dig('111110000000000000'),'05.40 s',''),
 ('out','Safety_DoorClosed','%I130.0','Bool','安全门 SD-01','TRUE','', dig('111111111111111111'),'00.00 s',''),
 ('out','Safety_LightCurtain','%I130.2','Bool','光栅 LC-02','FALSE','', dig('111111111111110000'),'12.47 s','alarm'),
 ('in','Conv_Run','%Q140.0','Bool','SK-01 · 输送启动','FALSE','', dig('011110000000000000'),'06.90 s',''),
]
def row(r,i):
    d,n,a,t,m,v,f,sp,lc,st=r
    bg={'warn':'background:rgba(255,178,36,.06);','alarm':'background:rgba(255,79,100,.08);','force':'background:rgba(255,178,36,.035);'}.get(st,'')
    sel = n=='FX30_C4_Closed'
    if sel: bg='background:var(--accent-bg);'
    vcol={'alarm':'var(--alarm)','warn':'var(--warn)'}.get(st,'var(--tx-1)')
    arrow=('<span class="tag" style="color:#7FD8F5;border-color:rgba(76,201,240,.35)">PLC→</span>' if d=='in' else '<span class="tag" style="color:var(--tx-2)">→PLC</span>')
    vb = f'<span class="mono" style="display:inline-flex;align-items:center;gap:6px;color:{vcol};font-weight:600"><span style="width:8px;height:8px;border-radius:2px;background:{"var(--tx-1)" if v=="TRUE" else "transparent"};border:1.5px solid {vcol}"></span>{v}</span>' if t=='Bool' else f'<span class="mono num" style="font-weight:600">{v}</span>'
    fz = f'<span class="tag warn">{ic("lock","s")} {f}</span>' if f else '<span class="tx4" style="color:var(--tx-4)">—</span>'
    icon={'alarm':f'<span style="color:var(--alarm)">{ic("alert","s")}</span>','warn':f'<span style="color:var(--warn)">{ic("warn","s")}</span>'}.get(st,'')
    return f'''<div style="display:grid;grid-template-columns:22px 62px 1.5fr 86px 50px 1.5fr 100px 108px 118px 70px;gap:10px;align-items:center;height:32px;padding:0 14px;border-bottom:1px solid var(--line-soft);{bg}{"box-shadow:inset 2px 0 0 var(--accent);" if sel else ""}font-size:12px">
<span>{icon}</span>{arrow}<span class="mono" style="color:var(--tx-1)">{n}</span><span class="mono" style="color:var(--tx-3)">{a}</span><span style="color:var(--tx-3)">{t}</span><span style="color:var(--tx-2);white-space:nowrap;overflow:hidden">{m}</span>{vb}{fz}{spark(sp,110,18,{'alarm':'#FF4F64','warn':'#FFB224'}.get(st,'#A6ACCB'),t=='Bool')}<span class="mono num" style="color:var(--tx-3);font-size:11px;text-align:right">{lc}</span></div>'''
table_rows=''.join(row(r,i) for i,r in enumerate(rows))
hdr=f'''<div style="display:grid;grid-template-columns:22px 62px 1.5fr 86px 50px 1.5fr 100px 108px 118px 70px;gap:10px;align-items:center;height:30px;padding:0 14px;border-bottom:1px solid var(--line);font-size:11px;color:var(--tx-3);background:var(--bg-panel)">
<span></span><span>方向</span><span>信号名 ▾</span><span>PLC 地址</span><span>类型</span><span>映射对象</span><span>当前值</span><span>强制</span><span>趋势 · 10 s</span><span style="text-align:right">上次变化</span></div>'''
def card(icon,title,sub,stat,stcol,kv):
    k=''.join(f'<div><div style="font-size:10.5px;color:var(--tx-3)">{a}</div><div class="mono num" style="font-size:13px;color:var(--tx-1)">{b}</div></div>' for a,b in kv)
    return f'''<div style="flex:1;display:flex;gap:14px;align-items:center;padding:12px 14px;border-radius:12px;background:var(--bg-raised);border:1px solid var(--line)">
<span style="width:36px;height:36px;border-radius:10px;background:var(--bg-input);border:1px solid var(--line-strong);display:grid;place-items:center;color:var(--tx-1)">{ic(icon)}</span>
<div style="min-width:150px;white-space:nowrap"><div style="font-weight:600;display:flex;gap:8px;align-items:center">{title}<span class="tag" style="color:{stcol};border-color:{stcol}66">{stat}</span></div><div class="mono" style="font-size:11px;color:var(--tx-3)">{sub}</div></div>
<div style="display:flex;gap:18px;margin-left:auto;white-space:nowrap">{k}</div></div>'''
conn=f'''<div style="display:flex;gap:12px;padding:12px 14px;border-bottom:1px solid var(--line-soft);background:var(--bg-panel)">
{card('cpu','PLC_OP30 · S7-1500','S7-PLCSIM Advanced · 实例 #1','RUN','var(--ok)',[('同步','虚拟时间'),('OB1','3.2 ms')])}
{card('link','OPC UA 客户端','opc.tcp://127.0.0.1:4840','已连接','var(--ok)',[('订阅','148'),('p95','1.8 ms')])}
<div style="width:300px;display:flex;flex-direction:column;justify-content:center;gap:8px;padding:10px 14px;border-radius:12px;background:var(--bg-raised);border:1px solid var(--line)">
<div style="display:flex;align-items:center;gap:8px"><span style="font-weight:600">联调</span><span class="mono num" style="margin-left:auto;font-size:13px">00:12.48</span><span class="pill">×1.0</span></div>
<div style="display:flex;gap:6px"><span class="btn pri" style="flex:1">{ic('pause','s')}暂停</span><span class="btn">{ic('stepf','s')}单周期</span><span class="btn">{ic('stop','s')}</span><span class="btn ghost">{ic('dots','s')}</span></div></div>
</div>'''
# logic analyzer
LW=1210; T=4.0
def X(t): return 150+ (t)/T*(LW-170)
la_sigs=[('PLC_StartClamp',[(0,0),(0.6,1)]),('FX30_C1_Closed',[(0,0),(1.15,1)]),('FX30_C2_Closed',[(0,0),(1.22,1)]),('FX30_C3_Closed',[(0,0),(1.20,1)]),('FX30_C4_Closed',[(0,0)]),('FX30_Clamped',[(0,0)]),('Safety_LightCurtain',[(0,1),(3.2,0)])]
la=''
for i,(n,ch) in enumerate(la_sigs):
    y=34+i*26
    col='#FF4F64' if 'Light' in n else ('#FFB224' if 'C4' in n else '#C792EA')
    d=''
    for j,(t,v) in enumerate(ch):
        yy=y+4 if v else y+18
        d+= (f'M{X(t):.1f},{yy}' if j==0 else f' H{X(t):.1f} V{yy}')
    d+=f' H{X(T):.1f}'
    la+=f'<text x="12" y="{y+15}" fill="#A6ACCB" font-size="11" font-family="JetBrains Mono">{n}</text><path d="{d}" fill="none" stroke="{col}" stroke-width="1.7"/>'
    la+=f'<line x1="150" x2="{X(T):.0f}" y1="{y+24}" y2="{y+24}" stroke="#1C2444"/>'
ca,cb=X(0.6),X(1.22)
la+=f'<line x1="{ca:.0f}" x2="{ca:.0f}" y1="22" y2="216" stroke="#62D2F2" stroke-dasharray="4 3"/><line x1="{cb:.0f}" x2="{cb:.0f}" y1="22" y2="216" stroke="#62D2F2" stroke-dasharray="4 3"/>'
la+=f'<rect x="{ca-8:.0f}" y="8" width="16" height="14" rx="3" fill="#62D2F2"/><text x="{ca:.0f}" y="19" fill="#0C1030" font-size="10" text-anchor="middle" font-weight="700" font-family="Inter">A</text>'
la+=f'<rect x="{cb-8:.0f}" y="8" width="16" height="14" rx="3" fill="#62D2F2"/><text x="{cb:.0f}" y="19" fill="#0C1030" font-size="10" text-anchor="middle" font-weight="700" font-family="Inter">B</text>'
la+=f'<rect x="{(ca+cb)/2-44:.0f}" y="8" width="88" height="14" rx="7" fill="rgba(98,210,242,.15)" stroke="rgba(98,210,242,.5)"/><text x="{(ca+cb)/2:.0f}" y="19" fill="#BDEBFA" font-size="10" text-anchor="middle" font-family="JetBrains Mono">Δt = 620 ms</text>'
tw=X(2.0)
la+=f'<rect x="{X(1.22):.0f}" y="{34+4*26}" width="{X(3.22)-X(1.22):.0f}" height="24" fill="rgba(255,178,36,.08)"/><text x="{X(1.6):.0f}" y="{34+4*26+15}" fill="#FFB224" font-size="11" font-family="Inter">等待 C4 · 已超出期望 0.42 s（PLC 超时 2.0 s）</text>'
la+=f'<line x1="{X(3.12):.0f}" x2="{X(3.12):.0f}" y1="22" y2="216" stroke="#8E92EC" stroke-width="2"/>'
for k in range(5):
    la+=f'<text x="{X(k):.0f}" y="232" fill="#4D5579" font-size="10" font-family="JetBrains Mono" text-anchor="middle">{9.36+k:.2f}s</text>'
analyzer=f'''<div class="panel" style="height:290px;border-top:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">{ic('wave','s')}逻辑分析仪</div><div class="tab">{ic('clock','s')}事件日志</div><div class="tab">{ic('gantt','s')}PLC ↔ 仿真时序</div></div>
<div class="act"><span class="chip on">触发：PLC_StartClamp ↑</span><span class="chip">光标 A/B</span><span class="btn ghost" style="height:24px">导出 VCD</span></div></div>
<svg width="{LW}" height="246">{la}</svg></div>'''
center=f'''<div style="flex:1;display:flex;flex-direction:column;min-width:0">{conn}
<div class="panel" style="flex:1">
<div class="ph"><div class="tabs"><div class="tab on">{ic('table','s')}信号表 <span class="tag">148</span></div><div class="tab">映射 <span class="tag warn">2 未映射</span></div><div class="tab">强制表 <span class="tag warn">1</span></div></div>
<div class="act" style="gap:6px"><div class="search" style="margin:0;width:240px;height:26px">{ic('search','s')}筛选信号 / 地址…</div><span class="chip on">异常优先</span><span class="chip">仅变化</span><span class="chip">FX-30</span></div></div>
{hdr}{table_rows}</div>{analyzer}</div>'''
tree_rows=[(0,'▾','plc','PLC_OP30','<span>148</span>',''),(1,'▾','cube','FX-30 焊装夹具','<span class="tag warn">1</span>','sel'),(2,' ','signal','输入 · 6','',''),(2,' ','signal','输出 · 8','',''),(1,'▸','robot','R01 + E1','<span>42</span>',''),(1,'▸','robot','R02','<span>38</span>',''),(1,'▸','cube','SK-01 滑橇输送','<span>16</span>',''),(1,'▸','shield','安全回路','<span class="tag alarm">1</span>',''),(1,'▸','gear','未映射','<span class="tag warn">2</span>','')]
tree=''.join(f'<div class="tr {c}" style="padding-left:{10+d*14}px"><span class="tw">{t}</span><span class="ic">{ic(i,"s")}</span><span class="nm">{n}</span><span class="meta">{m}</span></div>' for d,t,i,n,m,c in tree_rows)
left=f'''<div class="panel" style="width:250px;border-right:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">设备 / 信号</div><div class="tab">连接</div></div></div>
<div class="search">{ic('search','s')}搜索…</div><div class="tree">{tree}</div>
<div style="margin:10px;padding:10px;border-radius:9px;background:var(--bg-raised);border:1px solid var(--line);font-size:11.5px;color:var(--tx-2)">
<div style="font-weight:600;color:var(--tx-1);margin-bottom:6px">同步健康度</div>
<svg width="206" height="40"><path d="M0,30 {' '.join(f'L{i*206/59:.1f},{30-random.random()*6-(14 if i==41 else 0):.1f}' for i in range(60))}" fill="none" stroke="#62D2F2" stroke-width="1.4"/><line x1="0" x2="206" y1="12" y2="12" stroke="rgba(255,178,36,.5)" stroke-dasharray="3 3"/></svg>
<div style="display:flex;justify-content:space-between" class="mono"><span>抖动 0.21 ms</span><span>超周期 0</span></div></div>
</div>'''
alarms=[('alarm','12.47 s','光栅 LC-02 被遮挡','Safety_LightCurtain = FALSE · 来源：R02 工具进入光栅区','确认'),('warn','12.89 s','FX-30 夹爪 C4 未到位','超出期望 0.42 s · 可能原因：姿态终点 31.2° < 传感器阈值 32.0°','定位'),('info','09.80 s','R01_Start 被强制为 FALSE','由 用户 在强制表设置 · 联调结束时自动解除','解除')]
al=''.join(f'''<div style="display:grid;grid-template-columns:20px 1fr;gap:8px;padding:10px 12px;border-bottom:1px solid var(--line-soft);{"background:rgba(255,79,100,.06);" if k=="alarm" else ""}">
<span style="color:{dict(alarm='var(--alarm)',warn='var(--warn)',info='var(--info)')[k]}">{ic(dict(alarm='alert',warn='warn',info='info')[k],'s')}</span>
<div><div style="display:flex;gap:8px;align-items:center;font-weight:600;font-size:12px">{t}<span class="mono" style="margin-left:auto;font-weight:400;color:var(--tx-4);font-size:11px">{tm}</span></div><div style="font-size:11.5px;color:var(--tx-2);margin:2px 0 6px">{d}</div><span class="btn" style="height:22px;font-size:11px">{a}</span></div></div>''' for k,tm,t,d,a in alarms)
right=f'''<div class="panel" style="width:400px;border-left:1px solid var(--line-soft)">{pip}
<div class="ph"><div class="tabs"><div class="tab on">{ic('bell','s')}报警 <span class="tag alarm">1</span> <span class="tag warn">1</span></div><div class="tab">强制</div><div class="tab">监视表</div></div></div>
{al}
<div class="sec" style="border-top:1px solid var(--line-soft)"><h4>所选 · FX30_C4_Closed</h4>
<div class="row"><label>来源</label><div class="fld">FX-30 · 夹爪 C4 · 接近开关</div></div>
<div class="row"><label>触发条件</label><div class="fld mono">J_C4 ≥ 32.0°</div></div>
<div class="row"><label>当前</label><div class="fld mono" style="color:var(--warn)">J_C4 = 31.2°</div></div>
<div style="display:flex;gap:6px;margin-top:8px"><span class="btn">{ic('lock','s')}强制 TRUE</span><span class="btn">打开运动学</span><span class="btn ghost">查看逻辑</span></div></div>
</div>'''
body=f'''<div class="app">{titlebar('虚拟调试')}
<div class="body">{rail(['tree','lib','signal','logic','code','|',('shield','1'),'bell'],'signal')}{left}{center}{right}</div>
{statusbar([f'<span class="dot" style="background:var(--ok)"></span>联调运行中 · PLCSIM Advanced + OPC UA','虚拟时间同步 · 4 ms 步长',f'<span style="color:var(--alarm)">● 1 报警</span>'],['<span class="mono">F5 继续 · F10 单周期 · Ctrl+F 强制</span>',ic('globe','s')+'仅本机回环 127.0.0.1'])}</div>'''
open('m5_vc.html','w').write(page('虚拟调试信号监视',body))
