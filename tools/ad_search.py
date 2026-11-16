#!/usr/bin/env python3
"""AutoDRIVE(18Hz) 좌표 하강 탐색. 4랩 평가, 점수=플라잉+10*충돌/랩. 로그 logs/ad_search.log"""
import json, sys, os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); import ad_eval
cur=dict(margin=0.35,smooth=15,alat=10,tight=8,k_tight=0.8,aacc=4.5,abrk=5,steer_ay=8.5,steer_slip=2.5,ld0=0.7,ldk=0.1,lat=0.08,vlead=0.25,gov_e0=0.15)
SPACE=[('alat',[9,10,11,12]),('tight',[7,8,9,10]),('steer_ay',[7.5,8.5,9.5]),('steer_slip',[1.5,2.5,3.5]),('gov_e0',[0.0,0.1,0.15,0.25]),
       ('lat',[0.05,0.08,0.11,0.14]),('ld0',[0.5,0.7,0.9]),('ldk',[0.05,0.1,0.15]),('vlead',[0.15,0.25,0.35]),('abrk',[5,6,7]),('aacc',[4.5,5.5]),
       ('margin',[0.3,0.35,0.4]),('smooth',[6,15,30])]
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
json.dump(dict(best=best,cfg=cur),open('results/ad_search_best.json','w'))
