#!/usr/bin/env python3
"""AutoDRIVE 루프 평가: cfg(json) -> 라인 생성 -> run_loc_test(위치추정) -> 점수.
점수 = 플라잉랩 평균 + 10*충돌/랩 (대회 페널티 반영). 결과는 results/ad_eval.jsonl 누적."""
import os, sys, json, subprocess, hashlib, numpy as np
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT)
def line_for(c,track='porto'):
    g=f"maps/search/{track}_g_m{c['margin']}_s{c['smooth']}.csv"
    if not os.path.exists(g):
        subprocess.run(f"bash -c 'source env.sh; python3 tools/raceline.py maps/{track}_center.csv {g} --alat 8 --alat_tight 7 --smooth {c['smooth']} --margin {c['margin']} --aacc 4.5 --abrk 5 --band_map maps/{track}_map.npz'",shell=True,capture_output=True)
    h=hashlib.md5(json.dumps(c,sort_keys=True).encode()).hexdigest()[:8]; f=f"maps/search/{track}_ad_{h}.csv"
    subprocess.run(f"bash -c 'source env.sh; python3 tools/raceline.py maps/{track}_center.csv {f} --from_line {g} --alat {c['alat']} --alat_tight {c['tight']} --k_tight {c.get('k_tight',0.8)} --aacc {c.get('aacc',4.5)} --abrk {c.get('abrk',5)} --vmax {c.get('vmax',15)}'",shell=True,capture_output=True)
    return f
def run(c,laps=4,pose='loc',track='porto'):
    f=line_for(c,track)
    args=f"--vscale 1.0 --ld0 {c['ld0']} --ldk {c['ldk']} --steer_ay {c['steer_ay']} --steer_slip {c.get('steer_slip',3.0)} --vlead {c.get('vlead',0.25)} --lat {c.get('lat',0.08)} --gov_e0 {c.get('gov_e0',0.0)} --vff {c.get('vff',0.0)} --aff {c.get('aff',0.0)} --aff_neg {c.get('aff_neg',1.0)}"
    if c.get('ctrl')=='mpc': args+=' --ctrl mpc '+' '.join(f"--{k} {c[k]}" for k in ('qy','rd','mdelay','qpsi','N') if k in c)
    env=f"MAP=maps/{track}_map.npz CENTER=maps/{track}_center.csv"
    if pose=='loc': cmd=f"{env} tools/run_loc_test.sh {f} {laps} {args}"
    else: cmd=f"bash -c 'source env.sh; python3 tools/reset_car.py; python3 tools/racer.py --line {f} --pose gt --reset 0 --laps {laps} {args}'"
    out=subprocess.run(cmd,shell=True,capture_output=True,text=True).stdout
    r=[l for l in out.splitlines() if l.startswith('RESULT')]
    if not r: return dict(cfg=c,score=99,err=out[-300:])
    r=json.loads(r[-1][7:])
    if not r['timed']:
        res=dict(cfg=c,pose=pose,track=track,flying=99,best=99,coll=r.get('coll',0),laps=0,score=99)
        open('results/ad_eval.jsonl','a').write(json.dumps(res)+'\n'); return res
    t=r['timed'][1:] if len(r['timed'])>1 else [99]
    nl=max(len(r['timed']),1); score=float(np.mean(t))+10*r['coll']/nl
    res=dict(cfg=c,pose=pose,track=track,flying=round(float(np.mean(t)),3),best=round(min(r['timed']),3),coll=r['coll'],laps=nl,score=round(score,3))
    open('results/ad_eval.jsonl','a').write(json.dumps(res)+'\n'); return res
if __name__=='__main__':
    c=json.loads(sys.argv[1]); print(json.dumps(run(c,int(sys.argv[2]) if len(sys.argv)>2 else 4,sys.argv[3] if len(sys.argv)>3 else 'loc')))
