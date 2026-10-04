from kit import *
VW,VH=1474,1010
iso=Iso(700,30,12.4)
c=24; g=c+4.2
Jg=[(g,34.9,6.6),(g-0.6,31.0,12.9),(g+.2,26.6,12.2),(g+.5,25.3,10.8),(g+.6,24.7,9.4)]
gh=box(iso,g-2.6,32.6,.88,5.2,4.8,.9,'#8E92EC',op=.25)+robot(iso,g,34.9,1.8,Jg,ghost=True)
svg,J=cell_scene(iso,carriage_x=c,sel=False,extras=gh)
# new path preview W07..W12 (LIN straight) vs old (curvy)
old=[iso.p(x+.3,24.6,9.6+ (0.6 if i%2 else 0)) for i,x in enumerate(range(23,29))]
new=[iso.p(x+.3,24.6,9.4) for x in range(23,29)]
svg+='<polyline points="'+' '.join(f'{p[0]:.0f},{p[1]:.0f}' for p in old)+'" fill="none" stroke="#A6ACCB" stroke-width="1.5" stroke-dasharray="3 4" opacity=".6"/>'
svg+='<polyline points="'+' '.join(f'{p[0]:.0f},{p[1]:.0f}' for p in new)+'" fill="none" stroke="url(#airail)" stroke-width="3"/>'
for i,p in enumerate(new):
    svg+=f'<circle cx="{p[0]:.0f}" cy="{p[1]:.0f}" r="5" fill="#0E1328" stroke="#62D2F2" stroke-width="2"/>'
a=iso.p(c,38.8,.7); b=iso.p(g,38.8,.7)
svg+=f'<line x1="{a[0]:.0f}" y1="{a[1]:.0f}" x2="{b[0]:.0f}" y2="{b[1]:.0f}" stroke="#62D2F2" stroke-width="2" marker-end="url(#arr)"/>'
svg+='<defs><marker id="arr" markerWidth="8" markerHeight="8" refX="6" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8z" fill="#62D2F2"/></marker></defs>'
lbl=new[0]
steps=[
 ('select','选择对象','<span class="k1">op</span>=R01_焊接序列 <span class="k1">points</span>=W07..W12','6 个路径点','done'),
 ('set_motion','修改运动类型','<span class="k1">type</span>=LIN <span class="k1">speed</span>=200 mm/s <span class="k1">zone</span>=FINE','6 处修改 · 可撤销','done'),
 ('optimize_axis','外部轴寻优','<span class="k1">axis</span>=E1 <span class="k1">goal</span>=J5_margin≥15° <span class="k1">range</span>=[2100, 2700]','E1: 2 340 → <b>2 460 mm</b>','done'),
 ('check_collision','碰撞 / 近碰检查','<span class="k1">scope</span>=R01 ↔ {BIW_SIDE_L, FX-30} <span class="k1">near</span>=10 mm','0 碰撞 · 最近 12.6 mm','done'),
 ('simulate','分段预演','<span class="k1">range</span>=W06..W12 <span class="k1">mode</span>=确定性','节拍 +0.4 s (54.0 → 54.4)','run'),
]
def step(i,s):
    cmd,t,args,res,st=s
    stc={'done':('var(--ok)',ic('check','s')),'run':('var(--ai-2)','<span class="spin"></span>')}[st]
    return f'''<div style="display:grid;grid-template-columns:26px 1fr auto;gap:10px;align-items:start;padding:9px 12px;border-radius:9px;{"background:rgba(98,210,242,.06);border:1px solid rgba(98,210,242,.25)" if st=="run" else "border:1px solid transparent"}">
<span style="width:22px;height:22px;border-radius:50%;display:grid;place-items:center;background:var(--bg-raised);color:{stc[0]};border:1px solid var(--line-strong);font-size:11px">{stc[1]}</span>
<div><div style="display:flex;gap:8px;align-items:center"><span style="font-weight:600">{i+1}. {t}</span><span class="mono" style="font-size:10.5px;color:var(--lav-300);background:var(--accent-bg);border-radius:4px;padding:0 5px">{cmd}</span></div>
<div class="mono" style="font-size:11.5px;color:var(--tx-2);margin-top:3px">{args}</div></div>
<div style="font-size:11.5px;color:var(--tx-1);text-align:right;white-space:nowrap;padding-top:2px">{res}</div></div>'''
plan=''.join(step(i,s) for i,s in enumerate(steps))
intent=f'''<div style="position:absolute;left:50%;transform:translateX(-50%);bottom:22px;width:900px">
<div style="border-radius:16px;padding:1.5px;background:linear-gradient(120deg,rgba(142,146,236,.9),rgba(98,210,242,.7) 50%,rgba(142,146,236,.4));box-shadow:0 24px 70px rgba(3,6,18,.7),0 0 60px rgba(142,146,236,.18)">
<div style="border-radius:15px;background:rgba(16,21,42,.94);backdrop-filter:blur(18px)">
 <div style="display:flex;align-items:center;gap:10px;padding:12px 16px;border-bottom:1px solid var(--line)">
  <span style="width:26px;height:26px;border-radius:8px;background:var(--ai-grad);display:grid;place-items:center;color:#0C1030">{ic('spark','s')}</span>
  <span style="font-weight:600">命令计划</span><span class="tag lav">5 步 · 12 条结构化命令</span><span class="tag ok">已在沙盒预演</span>
  <span style="margin-left:auto;color:var(--tx-3);font-size:11.5px;display:flex;gap:6px;align-items:center">{ic('shield','s')}本地模型 · 数据不出本机</span></div>
 <div style="padding:8px 6px 4px">{plan}</div>
 <div style="margin:4px 16px 10px;padding:9px 12px;border-radius:9px;background:var(--bg-input);border:1px solid var(--line);font-size:11.5px;color:var(--tx-2);display:flex;gap:18px">
  <span><b style="color:var(--tx-1)">为什么是 2 460 mm：</b>J5 余量 6.4° → <b style="color:var(--ok)">18.2°</b>；近碰 8.2 mm → <b style="color:var(--ok)">12.6 mm</b>；E1 行程增量 120 mm 不影响 W13 起点。</span>
  <span style="margin-left:auto;color:var(--lav-300);white-space:nowrap">查看依据 {ic('chev','s')}</span></div>
 <div style="display:flex;align-items:center;gap:8px;padding:10px 16px;border-top:1px solid var(--line)">
  <span class="btn pri">{ic('check','s')}应用全部 <span class="mono" style="opacity:.7;font-size:10.5px">Ctrl ↵</span></span><span class="btn">逐步应用</span><span class="btn">编辑计划</span><span class="btn ghost">另存为 Python 脚本</span>
  <span style="margin-left:auto;font-size:11.5px;color:var(--tx-3)">影响 7 个对象 · 一次 <span class="kb">Ctrl Z</span> 即可整体撤销</span><span class="btn ghost">取消 <span class="kb">Esc</span></span></div>
</div></div>
<div style="margin-top:12px;display:flex;align-items:center;gap:12px;height:52px;padding:0 10px 0 16px;border-radius:14px;background:rgba(16,21,42,.92);border:1px solid rgba(142,146,236,.45);box-shadow:var(--sh-3)">
 <span style="color:var(--ai-1)">{ic('spark','l')}</span>
 <span style="font-size:13.5px;flex:1;white-space:nowrap;overflow:hidden">把 <span class="mention">@R01</span> 的 <span class="mention">W07–W12</span> 改成 LIN 200 mm/s，在 <span class="mention">@E1</span> 上找个让 J5 余量 ≥ 15° 的位置，并避开 B 柱近碰<span style="display:inline-block;width:2px;height:16px;background:var(--accent);vertical-align:-3px;margin-left:2px"></span></span>
 <span class="chip">R01 · 12.48 s</span><span class="kb">Ctrl</span><span class="kb">I</span></div>
</div>'''
vp=f'''<div class="vp" style="flex:1">
<svg class="scene" viewBox="0 0 {VW} {VH}" preserveAspectRatio="xMidYMid slice">{svg}</svg>
<div class="hud glass" style="left:14px;top:14px;padding:8px 12px;display:flex;gap:12px;align-items:center"><span class="tag lav">{ic('eye','s')} 预览模式</span><span>场景未修改 · 显示为幽灵叠加</span></div>
<div class="callout glass" style="left:{b[0]+10:.0f}px;top:{b[1]-4:.0f}px;border-color:rgba(98,210,242,.5)"><span class="mono" style="color:#BDEBFA">E1 +120 mm → 2 460</span></div>
<div class="callout glass" style="left:{lbl[0]-250:.0f}px;top:{lbl[1]-80:.0f}px;border-color:rgba(98,210,242,.45)"><div style="font-weight:600;background:var(--ai-grad);-webkit-background-clip:text;color:transparent">W07–W12 · LIN 200 mm/s</div><div style="font-size:11px;color:var(--tx-2)">虚线 = 当前 · 实线 = 提议</div></div>
<div class="glass" style="position:absolute;right:16px;top:14px;padding:10px 12px;width:250px;font-size:11.5px">
 <div style="color:var(--tx-3);margin-bottom:6px">预览对比</div>
 {''.join(f'<div style="display:flex;justify-content:space-between;height:22px;align-items:center"><span style="color:var(--tx-2)">{k}</span><span class="mono num"><span style="color:var(--tx-3)">{a}</span> → <b style="color:{c}">{b2}</b></span></div>' for k,a,b2,c in [('J5 余量','6.4°','18.2°','var(--ok)'),('最近距离','8.2','12.6 mm','var(--ok)'),('节拍','54.0','54.4 s','var(--warn)'),('E1 位置','2 340','2 460','var(--tx-1)')])}
</div>
{intent}
</div>'''
log_items=[('14:02','已应用','为 R02 生成 4 段涂胶 CIRC 路径','12 条命令','ok'),('13:47','已撤销','批量重命名 FX-30 信号为 IO 清单格式','148 信号','tx'),('13:31','已应用','将 BIW_SIDE_L.step 简化为碰撞代理 (8%)','1 对象','ok')]
right=f'''<div class="panel" style="width:400px;border-left:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">{ic('spark','s')}AI 会话</div><div class="tab">活动记录</div><div class="tab">权限</div></div><div class="act"><div class="ib">{ic('dots','s')}</div></div></div>
<div style="padding:12px;display:flex;flex-direction:column;gap:12px;flex:1;overflow:hidden">
 <div style="align-self:flex-end;max-width:310px;padding:9px 12px;border-radius:12px 12px 4px 12px;background:var(--bg-active);font-size:12.5px">把 @R01 的 W07–W12 改成 LIN 200 mm/s，在 @E1 上找个让 J5 余量 ≥ 15° 的位置，并避开 B 柱近碰</div>
 <div style="font-size:12.5px;color:var(--tx-2);line-height:1.65">
  <div style="display:flex;gap:6px;align-items:center;color:var(--tx-1);font-weight:600;margin-bottom:4px"><span style="color:var(--ai-1)">{ic('spark','s')}</span>理解的意图</div>
  <div style="padding:8px 10px;border-radius:8px;border:1px solid var(--line);background:var(--bg-panel)">
  <div>• 目标：<b style="color:var(--tx-1)">W07–W12</b>（R01_焊接序列，6 点）</div>
  <div>• 约束：J5 余量 ≥ 15°，近碰阈值 10 mm</div>
  <div>• 可调变量：E1 位置（允许 2 100–2 700 mm）</div>
  <div>• 不会修改：焊点位置、TCP、其他机器人</div></div>
  <div style="margin-top:10px;display:flex;gap:6px;align-items:center;color:var(--tx-1);font-weight:600">{ic('clock','s')}执行过程</div>
  {''.join(f'<div style="display:flex;gap:8px;align-items:center;height:24px;font-size:11.5px"><span style="color:var(--ok)">{ic("check","s")}</span>{t}<span class="mono" style="margin-left:auto;color:var(--tx-4)">{d}</span></div>' for t,d in [('读取场景快照 (只读)','12 ms'),('求解 E1 × 6 点逆解 · 61 个候选','0.8 s'),('沙盒碰撞扫描 · 3 对','0.3 s'),('分段仿真 W06–W12','运行中')])}
 </div>
 <div style="border-top:1px solid var(--line-soft);padding-top:10px">
  <div style="font-size:11px;color:var(--tx-3);font-weight:600;letter-spacing:.5px;margin-bottom:6px">最近的 AI 操作 · 均可回滚</div>
  {''.join(f'<div style="display:grid;grid-template-columns:40px 1fr auto;gap:8px;align-items:center;padding:7px 0;border-bottom:1px solid var(--line-soft);font-size:11.5px"><span class="mono" style="color:var(--tx-4)">{t}</span><span><span class="tag {"ok" if k=="ok" else ""}">{s}</span> {d}<br><span style="color:var(--tx-4)">{n}</span></span><span class="btn ghost" style="height:22px;padding:0 6px">{ic("undo","s")}</span></div>' for t,s,d,n,k in log_items)}
 </div>
 <div style="margin-top:auto;padding:10px;border-radius:10px;background:var(--bg-panel);border:1px solid var(--line);font-size:11.5px;color:var(--tx-2)">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px"><b style="color:var(--tx-1)">权限 · 本会话</b><span class="tag lav">需确认</span></div>
  <div style="display:grid;grid-template-columns:1fr auto;gap:5px 10px">
  <span>读取场景 / 仿真结果</span><span style="color:var(--ok)">允许</span><span>修改路径与参数</span><span style="color:var(--warn)">预览后确认</span><span>删除对象 / 改动信号映射</span><span style="color:var(--alarm)">禁止</span><span>联网 / 外部服务</span><span style="color:var(--alarm)">禁止</span></div></div>
</div></div>'''
css='''.mention{color:var(--lav-200);background:var(--accent-bg);border-radius:5px;padding:0 4px}
.k1{color:var(--ai-2)} .spin{width:12px;height:12px;border-radius:50%;border:2px solid rgba(98,210,242,.25);border-top-color:#62D2F2;display:inline-block}'''
body=f'''<div class="app">{titlebar('机器人',ai_on=True)}
<div class="body">{rail(['tree','lib','signal','logic','code','|',('shield','1'),'spark'],'spark')}
<div style="flex:1;display:flex;flex-direction:column;min-width:0">{vp}</div>{right}</div>
{statusbar([f'<span class="dot" style="background:var(--ai-2)"></span>AI 预览中 · 场景锁定','沙盒 #7 · 基于快照 12.48 s'],['推理：本地 GPU · 7.1 tok/s','<span class="mono">Ctrl+I 意图 · Ctrl+K 命令</span>',ic('globe','s')+'本地 · 离线'])}</div>'''
open('m3_ai.html','w').write(page('AI 意图栏',body,css))
