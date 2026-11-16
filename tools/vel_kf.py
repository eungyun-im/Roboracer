"""속도 추정 1D 칼만필터: IMU 종가속도 예측 + 엔코더 속도 보정(슬립 의심 시 R 팽창). localizer와 오프라인 평가 공용."""
import numpy as np
class VelKF:
    def __init__(s,q_acc=1.5,r_enc=0.2,gate=0.4,r_slip=3.0,scale=0.96,bias_q=0.0):
        s.v=0.0; s.P=1.0; s.q=q_acc; s.r=r_enc; s.gate=gate; s.rs=r_slip; s.scale=scale; s.init=False
    def reset(s): s.v=0.0; s.P=1.0
    def step(s,dt,ax,venc):
        z=venc*s.scale
        if not s.init: s.v=z; s.init=True; return s.v
        s.v+=ax*dt; s.P+=(s.q*dt)**2
        inn=z-s.v; R=s.r**2
        if abs(inn)>s.gate: R*=(abs(inn)/s.gate)**2*s.rs   # 슬립(휠스핀/잠김) 의심 -> 엔코더 신뢰 낮춤
        K=s.P/(s.P+R); s.v+=K*inn; s.P*=(1-K)
        s.v=max(s.v,0.0) if venc>=0 else s.v
        return s.v
