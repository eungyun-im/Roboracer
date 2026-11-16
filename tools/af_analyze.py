#!/usr/bin/env python3
"""racer 로그로 헤어핀 구간 앞바퀴 슬립각(추정: δ_cmd(지연) - atan(L·r/v)) 분포와 속도."""
import sys,numpy as np
L=0.324
def run(logf,line):
    d=np.load(logf); t,v,wz,de,idx=d[:,0],d[:,4],d[:,6],d[:,7],d[:,9].astype(int)
    Lc=np.loadtxt(line,delimiter=','); S=Lc[:,0]; V=Lc[:,5]
    dd=np.interp(t-0.11,t,de)            # 명령→실조향 지연 근사
    af=np.degrees(np.sign(wz)*(dd-np.arctan(L*wz/np.maximum(v,0.5))))
    m=t>t[0]+11; out=[]
    for nm,(a,b) in (('H1',(17,20.5)),('H2',(40.8,43.8)),('M',(22.5,25.7))):
        k=m&(S[idx]>=a)&(S[idx]<=b)&(v>1.5)
        if k.sum()<3: continue
        out.append(f'{nm}: af p50 {np.median(af[k]):.2f} p90 {np.percentile(af[k],90):.2f} >1.5° {100*np.mean(af[k]>1.5):.0f}% | vmin {v[k].min():.2f} (plan {V[(S>=a)&(S<=b)].min():.2f})')
    return ' || '.join(out)
if __name__=='__main__':
    for lf,ln in zip(sys.argv[1::2],sys.argv[2::2]): print(lf.split('/')[-1],'::',run(lf,ln))
