#!/usr/bin/env python3
"""gym 그립붕괴 모델(ay_cap, ay_slip, ay_collapse) 보정: 프레임률 실험(조향제한 OFF, GT)과 안전설정 기준 일치."""
import os, sys, json, itertools, numpy as np
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); import gymsim
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
T=gymsim.load_track('maps/porto_map.npz','maps/porto_center.csv')
CASES=[  # (line, ctrl, ctrl_hz, noise, target flying, target coll/lap)
 ('maps/porto_aggr.csv',dict(ld0=0.7,ldk=0.1,steer_ay=0),18.0,False,6.14,0.25),
 ('maps/porto_aggr.csv',dict(ld0=0.7,ldk=0.1,steer_ay=0),11.5,False,8.0,3.0),
 ('maps/porto_best.csv',dict(ld0=0.7,ldk=0.1,steer_ay=7.5),18.0,True,6.62,0.0),
 ('maps/porto_rl_a8_t7_s6.csv',dict(),18.0,True,6.80,0.0)]
rows=[]
for cap,slip,col in itertools.product([8.0,8.5,9.0,10.0],[1.5,2.5,3.5],[0.5,1.0,2.0]):
    cost=0; out=[]
    for line,ctrl,hz,noise,tf,tc in CASES:
        fl=[];cl=[]
        for seed in (0,1):
            sim=dict(kv=10,delay=0.05,ctrl_hz=hz,ay_cap=cap,ay_slip=slip,ay_collapse=col,seed=seed)
            if not noise: sim.update(noise_along=0,noise_cross=0,v_noise=0)
            r=gymsim.run(None,None,line,4,ctrl=ctrl,veh=dict(mu=1.6),sim=sim,track=T)
            fl.append(np.mean(r['laps'][1:]) if len(r['laps'])>1 else 20); cl.append(r['coll']/max(len(r['laps']),1))
        f,c=float(np.mean(fl)),float(np.mean(cl)); out.append((round(f,2),round(c,2)))
        cost+=abs(f-tf)/tf*100+5*abs(c-tc)
    rows.append((cost,dict(ay_cap=cap,ay_slip=slip,ay_collapse=col),out)); print(round(cost,2),rows[-1][1],out,flush=True)
rows.sort(key=lambda r:r[0])
print('BEST',rows[:5])
json.dump(dict(best=rows[0][1],out=rows[0][2]),open('results/gym_collapse_calib.json','w'))
