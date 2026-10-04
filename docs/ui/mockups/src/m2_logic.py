from kit import *
TC={'bool':'var(--t-bool)','real':'var(--t-real)','int':'var(--t-int)','pose':'var(--t-pose)','exec':'var(--t-exec)','str':'var(--t-str)'}
HEX={'bool':'#C792EA','real':'#7BD88F','int':'#4CC9F0','pose':'#F7A26C','exec':'#E7E9F7','str':'#F5D76E'}
CAT={'in':('#4CC9F0','信号输入','signal'),'out':('#C792EA','信号输出','signal'),'par':('#7BD88F','参数','gear'),
     'cond':('#FFB224','条件','filter'),'act':('#8E92EC','动作','zap'),'tim':('#62D2F2','定时器','clock'),'kin':('#F7A26C','运动学','joint')}
W=228; HDR=34; ROW=26
nodes={}
def N(id,x,y,cat,title,ins=(),outs=(),params=(),state='',w=W,sub=''):
    nodes[id]=dict(x=x,y=y,cat=cat,title=title,ins=list(ins),outs=list(outs),params=list(params),state=state,w=w,sub=sub)
def port_xy(id,side,i):
    n=nodes[id]; y=n['y']+HDR+6+i*ROW+ROW/2
    return (n['x'] if side=='in' else n['x']+n['w'], y)
N('n1',40,96,'in','PLC_StartClamp',outs=[('值','bool')],sub='DI · %I12.0',state='on')
N('n2',40,214,'in','Skid_InPos',outs=[('值','bool')],sub='DI · %I12.3',state='on')
N('n3',40,332,'par','t_clamp',outs=[('值','real')],params=[('数值','0.35 s')],sub='参数 · 块级')
N('n4',340,120,'cond','全部满足 (AND)',ins=[('A','bool'),('B','bool'),('+','bool')],outs=[('上升沿','exec'),('结果','bool')],state='on')
N('n5',640,96,'act','切换姿态',ins=[('触发','exec'),('目标姿态','pose'),('时长','real')],outs=[('完成','exec'),('进度','real')],params=[('设备','FX-30 夹具'),('插值','S 曲线')],state='run',w=244)
N('n6',340,316,'kin','姿态 · CLOSED',outs=[('姿态','pose')],params=[('C1–C4','0° → 32°')],sub='FX-30 · 4 关节')
N('n7',950,96,'cond','传感器全部到位',ins=[('触发','exec'),('C1..C4','bool')],outs=[('是','exec'),('否','exec'),('结果','bool')],state='')
N('n8',1000,330,'out','FX30_Clamped',ins=[('值','bool')],sub='DO · %Q20.0')
N('n9',1000,470,'out','FX30_Fault',ins=[('值','bool')],sub='DO · %Q20.1',state='err')
N('n10',640,420,'tim','超时 TON',ins=[('IN','bool'),('PT','real')],outs=[('Q','bool'),('ET','real')],params=[('PT','2.00 s')])
wires=[('n1',0,'n4',0,'bool','TRUE'),('n2',0,'n4',1,'bool','TRUE'),('n4',0,'n5',0,'exec',''),('n6',0,'n5',1,'pose',''),('n3',0,'n5',2,'real','0.35'),
       ('n5',0,'n7',0,'exec',''),('n7',2,'n8',0,'bool','FALSE'),('n4',1,'n10',0,'bool','TRUE')]
def node_html(id):
    n=nodes[id]; col,cn,ci=CAT[n['cat']]
    rows=max(len(n['ins']),len(n['outs']))
    st=n['state']
    border='rgba(255,79,100,.7)' if st=='err' else ('rgba(142,146,236,.9)' if id=='n7' else 'var(--line-strong)')
    glow='box-shadow:0 0 0 2px rgba(142,146,236,.35),var(--sh-3);' if id=='n7' else 'box-shadow:var(--sh-2);'
    h=f'<div style="position:absolute;left:{n["x"]}px;top:{n["y"]}px;width:{n["w"]}px;background:#151C37;border:1px solid {border};border-radius:10px;{glow}overflow:hidden">'
    h+=f'<div style="height:{HDR}px;display:flex;align-items:center;gap:7px;padding:0 10px;background:linear-gradient(90deg,{col}26,transparent);border-bottom:1px solid var(--line)"><span style="width:20px;height:20px;border-radius:6px;background:{col}33;color:{col};display:grid;place-items:center">{ic(ci,"s")}</span><div style="line-height:1.15"><div style="font-weight:600;font-size:12.5px">{n["title"]}</div><div style="font-size:10px;color:var(--tx-3)">{n["sub"] or cn}</div></div>'
    if st=='on': h+='<span style="margin-left:auto;width:8px;height:8px;border-radius:50%;background:var(--ok);box-shadow:0 0 8px var(--ok)"></span>'
    if st=='run': h+='<span class="tag lav" style="margin-left:auto">执行中 62%</span>'
    if st=='err': h+='<span class="tag alarm" style="margin-left:auto">未连接</span>'
    h+='</div><div style="padding:6px 0">'
    for i in range(rows):
        a=n['ins'][i] if i<len(n['ins']) else None; b=n['outs'][i] if i<len(n['outs']) else None
        h+=f'<div style="height:{ROW}px;display:flex;align-items:center;justify-content:space-between;padding:0 12px;font-size:11.5px;color:var(--tx-2)">'
        h+=(f'<span>{a[0]}</span>' if a else '<span></span>')+(f'<span>{b[0]}</span>' if b else '<span></span>')+'</div>'
    for k,v in n['params']:
        h+=f'<div style="margin:2px 10px 4px;display:flex;align-items:center;justify-content:space-between;height:24px;padding:0 8px;border-radius:6px;background:var(--bg-input);border:1px solid var(--line);font-size:11.5px"><span style="color:var(--tx-3)">{k}</span><span class="mono num">{v}</span></div>'
    if st=='run': h+='<div style="margin:2px 10px 8px;height:4px;border-radius:2px;background:var(--bg-input)"><div style="width:62%;height:100%;border-radius:2px;background:var(--accent)"></div></div>'
    h+='</div></div>'
    # port dots
    for i,(nm,t) in enumerate(n['ins']):
        x,y=port_xy(id,'in',i)
        shape='clip-path:polygon(0 0,100% 50%,0 100%);border-radius:0;' if t=='exec' else ''
        conn=any(w[2]==id and w[3]==i for w in wires)
        fill=HEX[t] if conn else '#151C37'
        h+=f'<span style="position:absolute;left:{x-6}px;top:{y-6}px;width:12px;height:12px;border-radius:50%;background:{fill};border:2px solid {HEX[t]};{shape}"></span>'
    for i,(nm,t) in enumerate(n['outs']):
        x,y=port_xy(id,'out',i)
        shape='clip-path:polygon(0 0,100% 50%,0 100%);border-radius:0;' if t=='exec' else ''
        conn=any(w[0]==id and w[1]==i for w in wires)
        fill=HEX[t] if conn else '#151C37'
        h+=f'<span style="position:absolute;left:{x-6}px;top:{y-6}px;width:12px;height:12px;border-radius:50%;background:{fill};border:2px solid {HEX[t]};{shape}"></span>'
    return h
svgw=''
labels=''
for a,ai,b,bi,t,val in wires:
    x1,y1=port_xy(a,'out',ai); x2,y2=port_xy(b,'in',bi)
    dx=max(60,(x2-x1)*.5)
    wdt=3 if t=='exec' else 2.2
    active = val!='' or t=='exec'
    svgw+=f'<path d="M{x1},{y1} C{x1+dx},{y1} {x2-dx},{y2} {x2},{y2}" fill="none" stroke="{HEX[t]}" stroke-width="{wdt}" opacity="{.95 if active else .6}"/>'
    if t=='exec' and a=='n4':
        svgw+=f'<path d="M{x1},{y1} C{x1+dx},{y1} {x2-dx},{y2} {x2},{y2}" fill="none" stroke="#fff" stroke-width="3" stroke-dasharray="6 14" opacity=".9"/>'
    if val:
        mx=(x1+x2)/2; my=(y1+y2)/2
        labels+=f'<span class="mono" style="position:absolute;left:{mx-22}px;top:{my-10}px;font-size:10.5px;padding:1px 6px;border-radius:9px;background:#0E1328;border:1px solid {HEX[t]}88;color:{HEX[t]}">{val}</span>'
# frames
frames=f'''
<div style="position:absolute;left:20px;top:44px;width:276px;height:410px;border-radius:12px;border:1px dashed rgba(76,201,240,.35);background:rgba(76,201,240,.03)"><div style="padding:8px 12px;font-size:11.5px;color:#7FD8F5;font-weight:600">触发与输入</div></div>
<div style="position:absolute;left:316px;top:44px;width:886px;height:340px;border-radius:12px;border:1px dashed rgba(142,146,236,.4);background:rgba(142,146,236,.035)"><div style="padding:8px 12px;font-size:11.5px;color:var(--lav-200);font-weight:600;display:flex;gap:8px">夹紧序列 <span style="color:var(--tx-3);font-weight:400">· 进入条件满足后切换夹具姿态并确认传感器</span></div></div>
<div style="position:absolute;left:616px;top:396px;width:330px;height:190px;border-radius:12px;border:1px dashed rgba(255,79,100,.35);background:rgba(255,79,100,.03)"><div style="padding:8px 12px;font-size:11.5px;color:#FF8A98;font-weight:600">安全互锁 · 超时</div></div>
<div style="position:absolute;left:330px;top:610px;width:250px;padding:10px 12px;border-radius:8px;background:#3A3420;border:1px solid #5C522C;color:#F3E2A6;font-size:11.5px;box-shadow:var(--sh-2)"><b>便签</b> · 与 PLC FB_Clamp 的握手时序对齐，见 IO 清单 v3.2 第 4 页。</div>
'''
canvas_nodes=''.join(node_html(k) for k in nodes)
canvas=f'''<div style="position:relative;flex:1;min-width:0;overflow:hidden;background-color:#0E1328;background-image:radial-gradient(rgba(142,146,236,.16) 1px,transparent 1px);background-size:22px 22px">
<div style="position:absolute;left:0;top:0;width:1300px;height:800px;transform:translate(10px,20px)">
{frames}<svg width="1300" height="800" style="position:absolute;left:0;top:0">{svgw}</svg>{labels}{canvas_nodes}</div>
<div class="glass" style="position:absolute;left:14px;top:12px;display:flex;align-items:center;gap:8px;padding:6px 12px;font-size:12px;color:var(--tx-2)">{ic('cube','s')}FX-30 焊装夹具 {ic('chev','s')} 逻辑块 {ic('chev','s')}<b style="color:var(--tx-1)">夹紧序列</b><span class="tag ok" style="margin-left:6px">实时调试 · 已连接仿真</span></div>
<div class="vtb glass" style="right:14px;top:12px"><div class="ib on">{ic('mouse','s')}</div><div class="ib">{ic('plus','s')}</div><div class="ib">{ic('link','s')}</div><div class="sep"></div><div class="ib">{ic('layers','s')}</div><div class="ib">{ic('expand','s')}</div><div class="sep"></div><span style="font-size:11px;color:var(--tx-3);padding:0 6px" class="mono">92%</span></div>
<div class="glass" style="position:absolute;left:50%;bottom:16px;transform:translateX(-50%);display:flex;align-items:center;gap:10px;padding:8px 10px 8px 14px;width:560px">
 {ic('search','s')}<span style="color:var(--tx-3);flex:1">添加块：输入类型或信号名…  例如 “TON”、“Skid_”</span><span class="kb">Tab</span><span class="kb">空格</span></div>
<div class="glass" style="position:absolute;right:14px;bottom:16px;width:220px;height:140px;padding:8px">
 <div style="font-size:10.5px;color:var(--tx-3);margin-bottom:4px">导航图</div>
 <div style="position:relative;height:108px">
 {''.join(f'<span style="position:absolute;left:{n["x"]/6.2:.0f}px;top:{n["y"]/6.2:.0f}px;width:{n["w"]/6.2:.0f}px;height:{(HDR+12+max(len(n["ins"]),len(n["outs"]))*ROW+len(n["params"])*28)/6.2:.0f}px;border-radius:2px;background:{CAT[n["cat"]][0]}66"></span>' for n in nodes.values())}
 <span style="position:absolute;left:0;top:0;width:200px;height:104px;border:1.5px solid var(--accent);border-radius:3px"></span></div></div>
<div class="glass" style="position:absolute;left:14px;bottom:16px;padding:8px 10px;font-size:11px;color:var(--tx-2);display:grid;grid-template-columns:auto auto;gap:4px 12px">
 {''.join(f'<span style="display:flex;align-items:center;gap:6px"><span style="width:9px;height:9px;border-radius:50%;background:{HEX[k]}"></span>{v}</span>' for k,v in [('exec','执行流'),('bool','布尔'),('real','实数'),('int','整数'),('pose','姿态'),('str','字符串')])}</div>
</div>'''

lib_groups=[('信号',[('signal','信号输入 DI/AI','#4CC9F0'),('signal','信号输出 DO/AO','#C792EA')]),
 ('流程',[('filter','条件 · 比较 / 逻辑','#FFB224'),('flow','分支 / 选择','#FFB224'),('clock','定时器 TON/TOF','#62D2F2'),('wave','边沿 R_TRIG','#62D2F2')]),
 ('动作',[('zap','切换姿态','#8E92EC'),('joint','驱动关节','#F7A26C'),('cube','创建/销毁零件','#8E92EC'),('link','挂接 / 解挂','#8E92EC'),('code','Python 脚本块','#F5D76E')]),
 ('结构',[('layers','子图 (宏)','#A6ACCB'),('gear','参数','#7BD88F')])]
lib=''
for g,items in lib_groups:
    lib+=f'<div style="padding:10px 12px 4px;font-size:11px;color:var(--tx-3);font-weight:600;letter-spacing:.5px">{g}</div>'
    for i,n,c in items:
        lib+=f'<div class="tr" style="padding-left:12px;height:30px"><span style="width:22px;height:22px;border-radius:6px;background:{c}22;color:{c};display:grid;place-items:center">{ic(i,"s")}</span><span class="nm">{n}</span><span class="meta">{ic("dots","s")}</span></div>'
blocks=[('夹紧序列','FX-30',True,'ok'),('松开序列','FX-30',False,'ok'),('滑橇到位检测','SK-01',False,'ok'),('焊枪握手','SG-02',False,'warn'),('安全门互锁','CELL',False,'ok')]
bl=''.join(f'<div class="tr{" sel" if s else ""}" style="padding-left:12px"><span class="ic">{ic("logic","s")}</span><span class="nm">{n}</span><span class="meta"><span>{d}</span><span style="width:7px;height:7px;border-radius:50%;background:{"var(--ok)" if st=="ok" else "var(--warn)"}"></span></span></div>' for n,d,s,st in blocks)
left=f'''<div class="panel" style="width:280px;border-right:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">块库</div><div class="tab">逻辑块</div><div class="tab">变量</div></div></div>
<div class="search">{ic('search','s')}搜索块…</div>{lib}
<div style="padding:12px 12px 4px;font-size:11px;color:var(--tx-3);font-weight:600;letter-spacing:.5px;border-top:1px solid var(--line-soft);margin-top:8px">本站逻辑块</div>{bl}
</div>'''
right=f'''<div class="panel" style="width:340px;border-left:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">块属性</div><div class="tab">真值 / 时序</div></div></div>
<div class="sec" style="display:flex;gap:10px;align-items:center"><span style="width:32px;height:32px;border-radius:8px;background:#FFB22422;color:#FFB224;display:grid;place-items:center">{ic('filter')}</span><div><div style="font-weight:600">传感器全部到位</div><div style="font-size:11.5px;color:var(--tx-3)">条件块 · 执行流分支</div></div></div>
<div class="sec"><h4>条件表达式</h4>
<div class="mono" style="padding:10px;border-radius:8px;background:var(--bg-input);border:1px solid var(--line);font-size:12px;line-height:1.7">
<span style="color:#C792EA">C1_Closed</span> <span style="color:var(--warn)">AND</span> <span style="color:#C792EA">C2_Closed</span><br><span style="color:var(--warn)">AND</span> <span style="color:#C792EA">C3_Closed</span> <span style="color:var(--warn)">AND</span> <span style="color:#C792EA">C4_Closed</span><br><span style="color:var(--tx-3)">// 稳定时间</span> <span style="color:#7BD88F">≥ 0.05 s</span></div>
<div style="display:flex;gap:6px;margin-top:8px"><span class="chip">可视化编辑</span><span class="chip on">表达式</span><span class="chip">表格</span></div></div>
<div class="sec"><h4>实时输入 <span class="r">t = 12.48 s</span></h4>
{''.join(f'<div style="display:flex;align-items:center;height:26px;gap:8px"><span class="mono" style="width:86px;font-size:11.5px;color:var(--tx-2)">{n}</span><svg width="150" height="18"><path d="{p}" stroke="#C792EA" stroke-width="1.5" fill="none"/></svg><span class="tag {"ok" if v else ""}" style="margin-left:auto">{"TRUE" if v else "FALSE"}</span></div>' for n,p,v in [('C1_Closed','M0,15 H40 V3 H150',1),('C2_Closed','M0,15 H52 V3 H150',1),('C3_Closed','M0,15 H48 V3 H150',1),('C4_Closed','M0,15 H150',0)])}
</div>
<div class="sec"><h4>诊断</h4>
<div style="display:flex;gap:8px;padding:8px 10px;border-radius:8px;background:rgba(255,178,36,.07);border:1px solid rgba(255,178,36,.28);font-size:11.5px;color:var(--tx-2)"><span style="color:var(--warn)">{ic('warn','s')}</span><div><b style="color:var(--warn)">C4 未到位已 0.42 s</b><br>夹爪 C4 运动学姿态终点 31.2°，传感器触发阈值 32.0°。<div style="margin-top:6px;display:flex;gap:6px"><span class="btn" style="height:24px">跳转到 C4 关节</span><span class="btn ghost" style="height:24px">在 3D 中显示</span></div></div></div>
</div>
<div class="sec"><h4>引用</h4><div style="font-size:11.5px;color:var(--tx-2);line-height:1.9">被 <b>夹紧序列</b> 使用 · 驱动信号 <span class="mono" style="color:#C792EA">FX30_Clamped</span><br>Python：<span class="mono" style="color:var(--tx-1)">cell.fx30.logic["夹紧序列"]</span></div></div>
</div>'''
bottom=f'''<div class="panel" style="height:200px;border-top:1px solid var(--line-soft)">
<div class="ph"><div class="tabs"><div class="tab on">{ic('alert','s')}问题 <span class="tag alarm">1</span> <span class="tag warn">1</span></div><div class="tab">{ic('wave','s')}监视</div><div class="tab">{ic('clock','s')}执行轨迹</div></div></div>
<div style="font-size:12px">
{''.join(f'<div style="display:grid;grid-template-columns:28px 120px 1fr 200px 120px;align-items:center;height:32px;padding:0 12px;border-bottom:1px solid var(--line-soft)"><span style="color:{c}">{ic(i,"s")}</span><span style="color:{c}">{t}</span><span>{m}</span><span style="color:var(--tx-3)">{w}</span><span class="btn ghost" style="height:24px;justify-content:flex-start">{a}</span></div>' for i,c,t,m,w,a in [
 ('alert','var(--alarm)','错误 LB-201','输出块 FX30_Fault 的输入“值”未连接；该信号在 PLC 映射中为必需','夹紧序列 › FX30_Fault','连接到超时 Q'),
 ('warn','var(--warn)','警告 LB-114','传感器 C4 在 0.42 s 内未达到阈值，序列卡在“传感器全部到位”','夹紧序列 › 传感器全部到位','查看时序'),
 ('info','var(--info)','提示','“超时 TON” 输出 Q 未被使用，可连接到 FX30_Fault','夹紧序列 › 超时 TON','自动连接')])}
</div></div>'''
body=f'''<div class="app">{titlebar('逻辑',crumb=('车身焊装线 A','侧围焊接站 OP30'))}
<div class="body">{rail(['tree','lib','signal','logic','code','|',('shield','1'),'bell'],'logic')}{left}
<div style="flex:1;display:flex;flex-direction:column;min-width:0">{canvas}{bottom}</div>{right}</div>
{statusbar([f'<span class="dot" style="background:var(--ok)"></span>实时调试','10 块 · 8 连线','已选 1 · 传感器全部到位'],['逻辑步长 4 ms','<span class="mono">Tab 添加 · F 聚焦 · Ctrl+G 编组</span>',ic('globe','s')+'本地 · 离线'])}</div>'''
open('m2_logic.html','w').write(page('逻辑块编辑器',body))
