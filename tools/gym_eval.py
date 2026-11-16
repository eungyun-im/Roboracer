#!/usr/bin/env python3
"""보정된 gymsim으로 레이싱라인/제어 파라미터 평가. usage: gym_eval.py <track> <line.csv> [laps] [ctrl json]"""
import sys, os, json, numpy as np
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); import gymsim
CAL=json.load(open(os.path.join(os.path.dirname(__file__),'..','results','gym_calib.json')))['best']
VEH=dict(mu=CAL['mu']); SIM=dict(kv=10,delay=CAL['delay'],half_w=CAL['half_w'],ay_cap=CAL['ay_cap'])
def evaluate(track,line,laps=4,ctrl=None,seeds=(0,1),T=None):
    T=T or gymsim.load_track(f'maps/{track}_map.npz',f'maps/{track}_center.csv')
    rs=[gymsim.run(None,None,line,laps,ctrl=ctrl or {},veh=VEH,sim=dict(SIM,seed=s),track=T) for s in seeds]
    fl=[np.mean(r['laps'][1:]) if len(r['laps'])>1 else 99 for r in rs]
    return dict(flying=float(np.mean(fl)),coll=float(np.mean([r['coll'] for r in rs])),done=all(r['done'] for r in rs))
if __name__=='__main__':
    tr,line=sys.argv[1],sys.argv[2]; laps=int(sys.argv[3]) if len(sys.argv)>3 else 4
    ctrl=json.loads(sys.argv[4]) if len(sys.argv)>4 else {}
    print(json.dumps(evaluate(tr,line,laps,ctrl)))
