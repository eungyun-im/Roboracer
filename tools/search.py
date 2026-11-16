#!/usr/bin/env python3
"""5단계: 보정된 gymsim으로 다맵 파라미터 탐색.
1) 맵별 레이싱라인 기하(band, margin×smooth) 생성  2) 랜덤 조합(속도프로파일+제어) 평가
선정 기준: 모든 맵·시드에서 충돌 0 -> 맵별 정규화 플라잉랩(최고값 대비) 평균 최소"""
import os, sys, json, itertools, subprocess, random, numpy as np
from multiprocessing import Pool
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0,'tools')
MAPS=['porto','iros26','gym_berlin','gym_example_map','gym_vegas']
GEOM=list(itertools.product([0.30,0.40],[6,15]))   # margin, smooth
OUT='results/search'; os.makedirs(OUT,exist_ok=True); os.makedirs('maps/search',exist_ok=True)
ENV=dict(os.environ)
def geom_job(args):
    m,(mg,sm)=args; f=f'maps/search/{m}_g_m{mg}_s{sm}.csv'
    if not os.path.exists(f):
        subprocess.run(['python3','tools/raceline.py',f'maps/{m}_center.csv',f,'--alat','8','--alat_tight','7','--smooth',str(sm),'--margin',str(mg),'--aacc','4.5','--abrk','5','--band_map',f'maps/{m}_map.npz'],capture_output=True,env=ENV)
    return f
def eval_job(args):
    cid,cfg,m=args
    import gymsim, gym_eval
    g=f"maps/search/{m}_g_m{cfg['margin']}_s{cfg['smooth']}.csv"
    line=f"maps/search/tmp_{cid}_{m}.csv"
    subprocess.run(['python3','tools/raceline.py',f'maps/{m}_center.csv',line,'--from_line',g,'--alat',str(cfg['alat']),'--alat_tight',str(cfg['alat_tight']),'--aacc','4.5','--abrk','5'],capture_output=True,env=ENV)
    T=gymsim.load_track(f'maps/{m}_map.npz',f'maps/{m}_center.csv')
    ctrl={k:cfg[k] for k in ('ld0','ldk','steer_ay','vlead')}
    r=gym_eval.evaluate(m,line,laps=3,ctrl=ctrl,seeds=(0,1),T=T)
    os.remove(line); return cid,m,r
if __name__=='__main__':
    N=int(sys.argv[1]) if len(sys.argv)>1 else 60
    with Pool(12) as P: P.map(geom_job,[(m,g) for m in MAPS for g in GEOM])
    print('geometry ready',flush=True)
    random.seed(1)
    space=dict(margin=[0.30,0.40],smooth=[6,15],alat=[7,8,9,10],alat_tight=[6,7,8],ld0=[0.4,0.5,0.7],ldk=[0.1,0.15,0.2],steer_ay=[7.0,7.5,8.5],vlead=[0.25])
    cfgs=[dict(margin=0.30,smooth=6,alat=8,alat_tight=7,ld0=0.5,ldk=0.15,steer_ay=7.5,vlead=0.25)]  # 현재 기준(Porto best)
    while len(cfgs)<N:
        c={k:random.choice(v) for k,v in space.items()}
        if c['alat_tight']<=c['alat'] and c not in cfgs: cfgs.append(c)
    jobs=[(i,c,m) for i,c in enumerate(cfgs) for m in MAPS]
    res={}
    with Pool(12) as P:
        for cid,m,r in P.imap_unordered(eval_job,jobs):
            res.setdefault(cid,{})[m]=r
    best_per_map={m:min(res[i][m]['flying'] for i in res if res[i][m]['coll']==0 and res[i][m]['done']) for m in MAPS}
    rows=[]
    for i,c in enumerate(cfgs):
        R=res[i]; coll=sum(R[m]['coll'] for m in MAPS); done=all(R[m]['done'] for m in MAPS)
        norm=float(np.mean([R[m]['flying']/best_per_map[m] for m in MAPS]))
        rows.append(dict(id=i,cfg=c,coll=coll,done=done,norm=norm,**{m:round(R[m]['flying'],3) for m in MAPS}))
    json.dump(rows,open(f'{OUT}/rows.json','w'),indent=1)
    ok=sorted([r for r in rows if r['coll']==0 and r['done']],key=lambda r:r['norm'])
    print(f'configs {len(rows)}, collision-free on all maps: {len(ok)}')
    for r in ok[:8]: print(json.dumps(r))
    print('baseline(id0):',json.dumps(rows[0]))
