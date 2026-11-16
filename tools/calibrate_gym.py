#!/usr/bin/env python3
"""AutoDRIVE 기준 실측(results/calib_refs.jsonl)에 맞춰 gymsim 파라미터 격자 탐색.
목적함수 = 플라잉랩 평균 차이(%) + 충돌률 불일치 페널티."""
import json, itertools, numpy as np, sys, os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); import gymsim
REFS=[]
for line in open('results/calib_refs.jsonl'):
    r=json.loads(line); REFS.append(r)
CFGS=[("maps/porto_rl_a8_t7_s6.csv",1.0),("maps/porto_rl_a9_t7.5_s15.csv",1.0),("maps/porto_rl_a10_t8_s6.csv",1.0),("maps/porto_rl_a8_t7_s6.csv",0.7)]
assert len(REFS)==len(CFGS), (len(REFS),len(CFGS))
ref=[]
for r in REFS:
    fl=r['timed'][1:]; ref.append((float(np.mean(fl)), r['coll']/len(r['timed'])))
print('AutoDRIVE refs (flying mean, coll/lap):',[(round(a,3),round(b,2)) for a,b in ref])
T=gymsim.load_track('maps/porto_map.npz','maps/porto_center.csv')
best=[]
grid=dict(mu=[1.2,1.4,1.6,1.9],ay_cap=[0.0,10.0,11.0,12.0],ay_collapse=[0.0],delay=[0.05,0.08],half_w=[0.12,0.135])
for mu,ayc,col,dl,hw in itertools.product(*grid.values()):
    cost=0; out=[]
    for (line,vs),(rt,rc) in zip(CFGS,ref):
        cs=[];ts=[]
        for seed in (0,1):
            r=gymsim.run(None,None,line,5,ctrl=dict(vscale=vs),veh=dict(mu=mu),sim=dict(kv=10,ay_cap=ayc,ay_collapse=col,delay=dl,seed=seed,half_w=hw),track=T)
            fl=r['laps'][1:] if len(r['laps'])>1 else [99]; ts.append(np.mean(fl)); cs.append(r['coll']/max(len(r['laps']),1))
        gt,gc=float(np.mean(ts)),float(np.mean(cs)); out.append((round(gt,3),round(gc,2)))
        cost+=abs(gt-rt)/rt*100 + 10*abs(gc-rc)
    best.append((cost,dict(mu=mu,ay_cap=ayc,ay_collapse=col,delay=dl,half_w=hw),out))
best.sort(key=lambda b:b[0])
for b in best[:8]: print(f'cost {b[0]:6.2f} {b[1]} -> {b[2]}')
json.dump(dict(best=best[0][1],refs=ref,sim=best[0][2]),open('results/gym_calib.json','w'),indent=1)
