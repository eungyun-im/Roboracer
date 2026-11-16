"""타이어 관측기(시뮬 소스 기반): 바퀴 표면속도 u(엔코더)로 차 속도 v 적분.
v' = g·sign(S)·mu(|S|)·k_mu - drag·v,  S=(u-v)/max(|v|,4)  (PhysX 최소 슬립 분모 4m/s)
mu: 상승 t(2-t) (t=|S|/xe, 0.10에서 피크 89%), 하강 smoothstep → 점근값, 이후 평탄."""
import numpy as np
XE,YE,XA,YA=0.15,0.72,0.25,0.464
def mu(s):
    s=abs(s)
    if s<=XE: t=s/XE; return YE*t*(2-t)
    if s<=XA: t=(s-XE)/(XA-XE); return YE+(YA-YE)*(3*t*t-2*t**3)
    return YA
class TireObs:
    def __init__(s,k_mu=1.0,drag=0.273,sub=10): s.v=0.0; s.k=k_mu; s.drag=drag; s.sub=sub; s.init=False
    def step(s,dt,u):
        if not s.init: s.v=u; s.init=True; return s.v
        h=dt/s.sub
        for _ in range(s.sub):
            S=(u-s.v)/max(abs(s.v),4.0); a=9.81*np.sign(S)*mu(S)*s.k-s.drag*s.v
            s.v+=a*h
        return s.v
