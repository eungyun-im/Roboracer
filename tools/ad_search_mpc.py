#!/usr/bin/env python3
"""AutoDRIVE(18Hz) 좌표 하강 탐색. 4랩 평가, 점수=플라잉+10*충돌/랩. 로그 logs/ad_search_mpc.log"""
import json, sys, os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); import ad_eval
cur=dict(ctrl='mpc',margin=0.35,smooth=6,alat=11,tight=7,k_tight=0.8,aacc=4.5,abrk=7,steer_ay=8.5,steer_slip=2.5,ld0=0.9,ldk=0.1,lat=0.08,vlead=0.15,gov_e0=0.15,qy=2,rd=5,mdelay=0.13,vff=1.0,qpsi=1.0)
SPACE=[('rd',[8,15]),('qy',[1,0.5]),('mdelay',[0.09,0.11]),('alat',[10.5,11.5,12]),('tight',[6.5,7.5,8]),('vff',[0.5,1.5]),('qpsi',[0.5,2]),
       ('steer_ay',[9.5,0]),('steer_slip',[1.5,3.5]),('vlead',[0.1,0.2]),('abrk',[6]),('gov_e0',[0.1,0.25]),('margin',[0.3,0.4])]
cache={}
# 이전 평가 재사용(results/ad_eval.jsonl, porto/loc)
for l in open('results/ad_eval.jsonl'):
    try:
        d=json.loads(l)
        if d.get('track')=='porto' and d.get('pose')=='loc' and d.get('laps',0)>=4 or d.get('score')==99: cache[json.dumps(d['cfg'],sort_keys=True)]=d
    except Exception: pass
if len(sys.argv)>1 and sys.argv[1]: cur=json.load(open(sys.argv[1]))['cfg'] if sys.argv[1].endswith('.json') else json.loads(sys.argv[1])
NR=int(sys.argv[2]) if len(sys.argv)>2 else 2
def ev(c):
    k=json.dumps(c,sort_keys=True)
    if k not in cache: cache[k]=ad_eval.run(c,4); print(json.dumps(cache[k]),flush=True)
    return cache[k]['score']
best=ev(cur)
for rnd in range(NR):
    for key,vals in SPACE:
        for v in vals:
            if v==cur[key]: continue
            c=dict(cur); c[key]=v
            if c['tight']>c['alat']: continue
            s=ev(c)
            if s<best-0.01: best,cur=s,c; print('IMPROVED',key,v,best,flush=True)
    print('ROUND',rnd,'best',best,json.dumps(cur),flush=True)
json.dump(dict(best=best,cfg=cur),open('results/ad_search_mpc_best.json','w'))
