"""공용 제어기(ROS 노드 racer.py와 gymsim.py가 동일 로직 사용).
속도연동 lookahead Pure Pursuit + 지연 보상 + vlead 속도 + 속도별 조향 제한."""
import numpy as np
L=0.324; MAXSTEER=0.5236
DEFAULTS=dict(vscale=1.0,ld0=0.5,ldk=0.15,ldmax=2.5,lat=0.08,vlead=0.25,steer_ay=7.5,steer_slip=3.0,gov_e0=0.0,gov_e1=0.4,gov_min=0.6,vff=0.0,aff=0.0,aff_neg=1.0,vm_kp=0.0,vm_blend=0.1,steer_abs=0.0,aff_lead=-1.0,cmode=0,ck=0.3,cdv=0.3,smode=0,sband=0.10,stau=0.15,sk=0.76,sbrk=0.15,sdb=0.3,aff_kmax=0.0)
class Controller:
    def __init__(s,line_csv,**kw):
        D=np.loadtxt(line_csv,delimiter=','); s.S,s.X,s.Y,s.V=D[:,0],D[:,1],D[:,2],D[:,5]; s.M=len(s.X); s.ds=s.S[1]-s.S[0]
        s.p=dict(DEFAULTS); s.p.update({k:v for k,v in kw.items() if v is not None}); s.idx=None
        s.A=s.V*np.gradient(s.V,s.S)   # 계획 가속도 v dv/ds
    def reset(s): s.idx=None
    def step(s,x,y,th,v,wz):
        p=s.p; X,Y,V,M,ds=s.X,s.Y,s.V,s.M,s.ds
        if p['lat']>0:
            x+=v*p['lat']*np.cos(th+wz*p['lat']/2); y+=v*p['lat']*np.sin(th+wz*p['lat']/2); th=th+wz*p['lat']
        if s.idx is None: s.idx=int(np.argmin((X-x)**2+(Y-y)**2))
        else:
            w=np.arange(s.idx-5,s.idx+40)%M; s.idx=int(w[np.argmin((X[w]-x)**2+(Y[w]-y)**2)])
        err=float(np.hypot(X[s.idx]-x,Y[s.idx]-y))
        ld=float(np.clip(p['ld0']+p['ldk']*v,p['ld0'],p['ldmax'])); j=(s.idx+int(round(ld/ds)))%M
        dx,dy=X[j]-x,Y[j]-y; ly=-np.sin(th)*dx+np.cos(th)*dy; d2=dx*dx+dy*dy
        delta=float(np.arctan(2*L*ly/max(d2,1e-3)))
        if p['steer_ay']>0:
            dmax=np.arctan(L*p['steer_ay']/max(v,0.5)**2)+np.radians(p['steer_slip'])
            if p['steer_abs']>0: dmax=max(dmax,np.radians(p['steer_abs']))   # 저속: 붕괴각(16°) 아래 고정각까지 허용
            delta=float(np.clip(delta,-dmax,dmax))
        delta=float(np.clip(delta,-MAXSTEER,MAXSTEER))
        jv=(s.idx+int(round(max(v,0.5)*p['vlead']/ds)))%M; vref=p['vscale']*min(V[s.idx],V[jv])
        if p['gov_e0']>0 and err>p['gov_e0']:   # 경로 이탈 시 감속(충돌 방지 거버너)
            vref*=max(p['gov_min'],1-(1-p['gov_min'])*(err-p['gov_e0'])/max(p['gov_e1']-p['gov_e0'],1e-3))
        if p['aff']>0 and not (p['aff_kmax']>0 and hasattr(s,'KAP') and abs(float(s.KAP[s.idx]))>p['aff_kmax']):   # 급코너 안에선 가속FF 끔(출구 과속 방지)
            ja=jv if p['aff_lead']<0 else (s.idx+int(round(max(v,0.5)*p['aff_lead']/ds)))%M   # 가속FF 선행점(제동 선행 vlead와 분리)
            A=float(s.A[ja]); vref=max(0.0,vref+p['aff']*(A if A>0 else p['aff_neg']*A))   # 계획 가속도 FF(aff_neg=감속측 배율)
        vfb=v
        if p['vm_kp']>0:   # 모델 속도: 보낸 명령으로 내부 P 속도제어기를 모사(측정 노이즈 없음), 측정으로 천천히 보정
            import time as _t
            now=_t.time(); dt=min(max(now-getattr(s,'_vmt',now),0.0),0.2); s._vmt=now
            vm=getattr(s,'_vm',v); vm+=float(np.clip(p['vm_kp']*(getattr(s,'_cmd',v)-vm),-7.0,4.6))*dt; vm+=p['vm_blend']*(v-vm); s._vm=vm; vfb=vm
        if p['vff']>0: vref=max(0.0,vref+p['vff']*(vref-vfb))   # 내부 P 속도제어기 포화 유도(가감속 피드포워드)
        if p['cmode']>0 and abs(float(s.KAP[s.idx] if hasattr(s,'KAP') else 0.0))>p['ck']:
            if v>1.0 and p['cmode']==1: vref=min(vref,v)            # 유지: 코너 안에서 가속 요구 안 함 (저속/출발 제외)
            elif v>1.0 and p['cmode']==2: vref=min(vref,max(v-p['cdv'],1.0))  # 트레일: 코너 안 약제동
        if p['smode']==2:   # 하이브리드: 가속은 기존 명령, 목표보다 빠르면(제동) 최적 슬립 제동
            A=float(s.A[s.idx]); vl=v+min(A,0.0)*p['stau']
            if vref<vl-p['sdb']: vref=max(vref,vl-p['sbrk']*max(vl,4.0))
            vref=max(vref,0.05)
        elif p['smode']>0:   # 슬립 제어: 명령 반영(~stau) 시점의 차속 예측, 그 대비 ±sband 슬립 안에서 바퀴속도 명령
            A=float(s.A[s.idx]); vl=v+(A*p['stau'] if (vref-v)*A>0 else 0.0)
            band=p['sband']*max(vl,4.0)
            vref=float(np.clip(vref+p['sk']*(vref-vl),vl-band,vl+band)); vref=max(vref,0.05)
        s._cmd=vref
        return delta,vref,err

# ---------------- 지연·서보 모델 기반 선형 MPC ----------------
# 식별(로그): 명령→조향 지연 ~0.13s, 서보 속도 ~2 rad/s, 언더스티어 K~0.005 (w = v*tanδ/(L(1+Kv²)))
MPC_DEFAULTS=dict(mdelay_hz0=0.0,mdelay_dev=0.15,merge_d=0.0,afcap=0.0,afmax=0.0,af_vmin=1.5,af_dmin=6.0,qy_hi=0.0,qy_v1=5.0,qy_v2=7.0,servo_asym=0,N=15,mdelay=0.13,srate=2.0,K=0.005,qy=8.0,qpsi=1.0,rd=0.5,rref=0.05,qterm=3.0,ay_cap=9.5,cmd_lead=0)
class MPCController(Controller):
    def __init__(s,line_csv,**kw):
        mk={k:kw.pop(k) for k in list(kw) if k in MPC_DEFAULTS}
        super().__init__(line_csv,**kw)
        D=np.loadtxt(line_csv,delimiter=','); s.PSI,s.KAP=D[:,3],D[:,4]
        s.m=dict(MPC_DEFAULTS); s.m.update({k:v for k,v in mk.items() if v is not None})
        s.hist=[]; s.act=0.0; s.act_t=None; s.dt_est=1/18.0; s.t_prev=None
        import osqp, scipy.sparse as sp; s.osqp, s.sp = osqp, sp
    def reset(s): s.idx=None; s.hist=[]; s.act=0.0; s.act_t=None
    def start_merge(s): s._merge_pending=True   # 충돌복귀: 다음 스텝의 오차를 기준으로 merge_d[m]에 걸쳐 라인 합류
    def _cmd_at(s,tau):
        c=0.0
        for (ts,v) in s.hist:
            if ts<=tau: c=v
            else: break
        return c
    def _servo(s,a,t0,t1):
        # 실제 조향 a를 t0->t1 적분 (지연된 명령을 속도제한으로 추종). 소구간 0.01s
        d=s.m['mdelay']; r=s.m['srate']; tau=t0
        while tau<t1-1e-9:
            h=min(0.01,t1-tau); c=s._cmd_at(tau-d); a=s._step_servo(a,c,r*h); tau+=h
        return a
    def _step_servo(s,a,c,dmax):
        # 결승 빌드 실측: 조향 크기 감소(|c|<=|a|, 중앙쪽)는 즉시, 크기 증가만 속도제한
        if s.m['servo_asym'] and abs(c)<=abs(a): return float(c)
        return float(a+np.clip(c-a,-dmax,dmax))
    def _yawrate(s,v,dl):
        w=v*np.tan(dl)/(L*(1+s.m['K']*v*v)); cap=s.m['ay_cap']/max(v,0.3)
        return float(np.clip(w,-cap,cap))
    def step(s,x,y,th,v,wz,t=None):
        import time as _t
        t=_t.time() if t is None else t; p=s.p; m=s.m
        if s.t_prev is not None: s.dt_est=0.9*s.dt_est+0.1*float(np.clip(t-s.t_prev,0.02,0.2))
        s.t_prev=t
        if m['mdelay_hz0']>0:   # 명령 지연은 프레임 수 단위(시뮬 lockstep): 측정 주기가 기준(18Hz)에서 dev 이상 벗어날 때만 비례 조정
            if not hasattr(s,'_md0'): s._md0=m['mdelay']
            r=m['mdelay_hz0']*s.dt_est
            m['mdelay']=float(np.clip(s._md0*r,0.04,0.3)) if abs(r-1)>m['mdelay_dev'] else s._md0
        if s.act_t is None: s.act_t=t
        s.act=s._servo(s.act,s.act_t,t); s.act_t=t
        s.hist=[h for h in s.hist if h[0]>t-m['mdelay']-0.5]
        # 지연 구간 동안 예측(이미 보낸 명령들로 조향 궤적이 정해짐)
        te=t+m['mdelay']; a=s.act; tau=t
        while tau<te-1e-9:
            h=min(0.01,te-tau); c=s._cmd_at(tau-m['mdelay']); a=s._step_servo(a,c,m['srate']*h)
            w=s._yawrate(v,a); x+=v*h*np.cos(th+w*h/2); y+=v*h*np.sin(th+w*h/2); th+=w*h; tau+=h
        X,Y,V,M,ds=s.X,s.Y,s.V,s.M,s.ds
        if s.idx is None: s.idx=int(np.argmin((X-x)**2+(Y-y)**2))
        else:
            wdw=np.arange(s.idx-5,s.idx+40)%M; s.idx=int(wdw[np.argmin((X[wdw]-x)**2+(Y[wdw]-y)**2)])
        i0=s.idx; ps=s.PSI[i0]
        ey=float(-(x-X[i0])*np.sin(ps)+(y-Y[i0])*np.cos(ps)); ep=float((th-ps+np.pi)%(2*np.pi)-np.pi)
        err=abs(ey)
        if m['merge_d']>0:
            if getattr(s,'_merge_pending',False): s._merge=(ey,ep,i0); s._merge_pending=False
            mg=getattr(s,'_merge',None)
            if mg is not None:
                trav=((i0-mg[2])%M)*ds; w=max(0.0,1.0-trav/m['merge_d'])
                if w<=0 or trav>M*ds/2: s._merge=None
                else: ey-=mg[0]*w; ep-=mg[1]*w   # 복귀 지점 오차를 거리에 따라 서서히 0으로
        # 지평 구성
        N=int(m['N']); dt=s.dt_est; vk=np.zeros(N); kk=np.zeros(N); sk=0.0; vc=max(v,0.5)
        for k in range(N):
            j=(i0+int(round(sk/ds)))%M; vt=p['vscale']*V[j]; vc=min(vt,vc+4.6*dt) if vt>vc else max(vt,vc-6.0*dt)
            vk[k]=max(vc,0.5); kk[k]=s.KAP[j]; sk+=vk[k]*dt
        Lg=L*(1+m['K']*vk**2); dr=np.arctan(Lg*kk); sec2=1/np.cos(dr)**2
        # 상태 [ey,ep]; ep[k+1]=ep+dt*(v/Lg*(tan dr + sec2 (δ-dr)) - v κ)
        # => ep[k+1]=ep + b_k δ_k + c_k,  b_k=dt v sec2/Lg,  c_k=dt v (tan dr - sec2 dr)/Lg - dt v κ
        b=dt*vk*sec2/Lg; c=dt*vk*(np.tan(dr)-sec2*dr)/Lg-dt*vk*kk
        # 응축: ep_k (k=1..N), ey_k (k=1..N) 를 δ의 선형식으로
        Ep=np.zeros((N,N)); ep0=np.zeros(N); Ey=np.zeros((N,N)); ey0=np.zeros(N)
        ecur=ep; ccur=np.zeros(N); ycur=ey; ycoef=np.zeros(N)
        for k in range(N):
            # ey[k+1]=ey[k]+dt v_k ep[k]
            ycoef=ycoef+dt*vk[k]*ccur; ycur=ycur+dt*vk[k]*ecur
            ccur=ccur.copy(); ccur[k]+=b[k]; ecur=ecur+c[k]
            Ep[k]=ccur; ep0[k]=ecur; Ey[k]=ycoef; ey0[k]=ycur
        W=np.ones(N); W[-1]=m['qterm']
        D1=np.eye(N)-np.eye(N,k=-1)   # Δδ
        qy=m['qy']
        if m['qy_hi']>0:   # 속도 스케줄: 저속(헤어핀)은 기존 qy, 고속일수록 qy_hi
            qy=m['qy']+(m['qy_hi']-m['qy'])*float(np.clip((v-m['qy_v1'])/max(m['qy_v2']-m['qy_v1'],1e-3),0,1))
        H=qy*Ey.T@np.diag(W)@Ey+m['qpsi']*Ep.T@np.diag(W)@Ep+m['rd']*D1.T@D1+m['rref']*np.eye(N)
        f=qy*Ey.T@(W*ey0)+m['qpsi']*Ep.T@(W*ep0)-m['rref']*dr
        d0=np.zeros(N); d0[0]=a; f=f-m['rd']*D1.T@d0
        # 제약: |δ|≤dmax_k, |Δδ|≤srate*dt (첫 칸은 현재 실제조향 a 기준)
        if p['steer_ay']>0:
            dmax=np.arctan(L*p['steer_ay']/vk**2)+np.radians(p['steer_slip'])
            if p['steer_abs']>0: dmax=np.maximum(dmax,np.radians(p['steer_abs']))
            dmax=np.minimum(dmax,MAXSTEER)
        else: dmax=np.full(N,MAXSTEER)
        rr=m['srate']*dt
        A=s.sp.vstack([s.sp.eye(N),s.sp.csc_matrix(D1)]).tocsc()
        lo=np.r_[-dmax,np.full(N,-rr)]; hi=np.r_[dmax,np.full(N,rr)]
        if m['afcap']>0:   # 곡률 기반 조향 상한: |δ_k| ≤ |atan(Lg·κ_k)| + afcap (회전 방향만) → 밀릴 때 과조향(앞바퀴 슬립 과대) 방지
            cap=np.radians(m['afcap'])
            for k in range(N):
                if dr[k]>=0: hi[k]=min(hi[k],dr[k]+cap)
                else: lo[k]=max(lo[k],dr[k]-cap)
        if m['afmax']>0 and v>m['af_vmin']:   # 앞바퀴 슬립각 제약: δ0를 정상상태 조향(atan(L·r/v)) ± afmax 안으로(조향 증가 방향만)
            dss=float(np.arctan(L*wz/max(v,0.5))); am=np.radians(m['afmax'])
            if abs(dss)>np.radians(m['af_dmin']):   # 이미 코너 안(회전 중)일 때만: 진입(요레이트 형성 전)은 막지 않음
                if dss>=0: hi[0]=min(hi[0],max(dss+am,lo[0]+1e-3))
                else: lo[0]=max(lo[0],min(dss-am,hi[0]-1e-3))
        if m['servo_asym']:   # 기준조향(곡률 FF) 부호 쪽으로 커질 때만 속도제한, 반대(감소)는 완화
            sg=np.sign(dr); sg[sg==0]=1
            lo[N:]=np.where(sg>0,-2*MAXSTEER,-rr); hi[N:]=np.where(sg>0,rr,2*MAXSTEER)
        lo[N]+=a; hi[N]+=a
        lo=np.minimum(lo,hi)
        try:
            pr=s.osqp.OSQP(); pr.setup(P=s.sp.triu(s.sp.csc_matrix(H)).tocsc(),q=f,A=A,l=lo,u=hi,verbose=False,polishing=False,eps_abs=1e-4,eps_rel=1e-4,max_iter=400)
            sol=pr.solve(); dl=sol.x
            if dl is None or not np.all(np.isfinite(dl)): raise RuntimeError
        except Exception:
            dl=np.clip(dr,-dmax,dmax)
        delta=float(np.clip(dl[min(int(m['cmd_lead']),N-1)],-MAXSTEER,MAXSTEER))
        s.hist.append((t,delta))
        jv=(s.idx+int(round(max(v,0.5)*p['vlead']/ds)))%M; vref=p['vscale']*min(V[s.idx],V[jv])
        if p['gov_e0']>0 and err>p['gov_e0']:
            vref*=max(p['gov_min'],1-(1-p['gov_min'])*(err-p['gov_e0'])/max(p['gov_e1']-p['gov_e0'],1e-3))
        if p['aff']>0 and not (p['aff_kmax']>0 and hasattr(s,'KAP') and abs(float(s.KAP[s.idx]))>p['aff_kmax']):   # 급코너 안에선 가속FF 끔(출구 과속 방지)
            ja=jv if p['aff_lead']<0 else (s.idx+int(round(max(v,0.5)*p['aff_lead']/ds)))%M   # 가속FF 선행점(제동 선행 vlead와 분리)
            A=float(s.A[ja]); vref=max(0.0,vref+p['aff']*(A if A>0 else p['aff_neg']*A))   # 계획 가속도 FF(aff_neg=감속측 배율)
        vfb=v
        if p['vm_kp']>0:   # 모델 속도: 보낸 명령으로 내부 P 속도제어기를 모사(측정 노이즈 없음), 측정으로 천천히 보정
            import time as _t
            now=_t.time(); dt=min(max(now-getattr(s,'_vmt',now),0.0),0.2); s._vmt=now
            vm=getattr(s,'_vm',v); vm+=float(np.clip(p['vm_kp']*(getattr(s,'_cmd',v)-vm),-7.0,4.6))*dt; vm+=p['vm_blend']*(v-vm); s._vm=vm; vfb=vm
        if p['vff']>0: vref=max(0.0,vref+p['vff']*(vref-vfb))   # 내부 P 속도제어기 포화 유도(가감속 피드포워드)
        if p['cmode']>0 and abs(float(s.KAP[s.idx] if hasattr(s,'KAP') else 0.0))>p['ck']:
            if v>1.0 and p['cmode']==1: vref=min(vref,v)            # 유지: 코너 안에서 가속 요구 안 함 (저속/출발 제외)
            elif v>1.0 and p['cmode']==2: vref=min(vref,max(v-p['cdv'],1.0))  # 트레일: 코너 안 약제동
        if p['smode']==2:   # 하이브리드: 가속은 기존 명령, 목표보다 빠르면(제동) 최적 슬립 제동
            A=float(s.A[s.idx]); vl=v+min(A,0.0)*p['stau']
            if vref<vl-p['sdb']: vref=max(vref,vl-p['sbrk']*max(vl,4.0))
            vref=max(vref,0.05)
        elif p['smode']>0:   # 슬립 제어: 명령 반영(~stau) 시점의 차속 예측, 그 대비 ±sband 슬립 안에서 바퀴속도 명령
            A=float(s.A[s.idx]); vl=v+(A*p['stau'] if (vref-v)*A>0 else 0.0)
            band=p['sband']*max(vl,4.0)
            vref=float(np.clip(vref+p['sk']*(vref-vl),vl-band,vl+band)); vref=max(vref,0.05)
        s._cmd=vref
        return delta,vref,err

# ---------------- LiDAR 직접 안전층(Guard) ----------------
# 맵·위치추정과 무관하게 최신 LiDAR 점(차체 좌표)으로 앞으로 horizon[s] 동안의 차체 궤적을 예측해
# 벽과 겹칠 것 같으면 조향을 최소한으로 비틀고(후보 오프셋), 그래도 안 되면 감속.
# 예측: 지연(delay) 동안은 이미 보낸 조향, 이후는 후보 조향으로 운동학 자전거 모델(언더스티어 K) 적분.
# 실험적(기본 꺼짐): 오프라인 미보정 시뮬에서 충돌 감소 효과 불확실 — 실주행에서 개입률/충돌로 판단할 것
GUARD_DEFAULTS=dict(g_h=0.45,g_dt=0.05,g_delay=0.13,g_front=0.32,g_rear=-0.05,g_hw=0.125,g_cmin=0.01,
                    g_offs=0.10,g_nofs=4,g_brake=1.5,g_rmax=4.0,g_K=0.005)
class Guard:
    def __init__(s,**kw):
        s.p=dict(GUARD_DEFAULTS); s.p.update({k:v for k,v in kw.items() if v is not None and k in GUARD_DEFAULTS})
        s.pts=None; s.t_scan=None; s.last_delta=0.0; s.n_act=0; s.n_calls=0
        n=int(s.p['g_nofs']); o=np.linspace(0,s.p['g_offs'],n+1)[1:]
        s.cands=np.r_[0.0,np.ravel(np.c_[o,-o])]   # 0, +o1, -o1, +o2, -o2 ...
    def set_scan(s,pts_body,t):
        r=np.hypot(pts_body[:,0],pts_body[:,1]); s.pts=pts_body[r<s.p['g_rmax']]; s.t_scan=t
    def clearance(s,v,d_now,d_new,shift=0.0):
        """후보 조향 배열 d_new(n,)별 예측 궤적 동안 차체 사각형-점 최소거리 (n,)"""
        p=s.p; P=s.pts.copy(); P[:,0]-=shift   # 스캔 이후 이동분 보정(직진 근사)
        n=len(d_new); x=np.zeros(n); y=np.zeros(n); th=np.zeros(n); cmin=np.full(n,9.0)
        Lg=L*(1+p['g_K']*v*v); tt=0.0; h=p['g_dt']
        while tt<p['g_h']-1e-9:
            d=np.full(n,d_now) if tt<p['g_delay'] else d_new
            w=v*np.tan(d)/Lg; x=x+v*h*np.cos(th+w*h/2); y=y+v*h*np.sin(th+w*h/2); th=th+w*h; tt+=h
            c,sn=np.cos(th),np.sin(th)
            dx=P[None,:,0]-x[:,None]; dy=P[None,:,1]-y[:,None]
            bx=c[:,None]*dx+sn[:,None]*dy; by=-sn[:,None]*dx+c[:,None]*dy
            ex=np.maximum(np.maximum(p['g_rear']-bx,bx-p['g_front']),0.0); ey=np.maximum(np.abs(by)-p['g_hw'],0.0)
            cmin=np.minimum(cmin,np.hypot(ex,ey).min(1))
        return cmin
    def filter(s,v,delta,vref,t=None):
        """반환 (delta, vref, 개입여부). 스캔 없거나 저속이면 그대로."""
        s.n_calls+=1
        if s.pts is None or len(s.pts)==0 or v<1.0: s.last_delta=delta; return delta,vref,False
        shift=v*max(0.0,min((t-s.t_scan) if (t is not None and s.t_scan is not None) else 0.0,0.2))
        cand=np.clip(delta+s.cands,-MAXSTEER,MAXSTEER)
        cl=s.clearance(v,s.last_delta,cand,shift)
        act=False
        if cl[0]<s.p['g_cmin']:
            ok=np.where(cl>=s.p['g_cmin'])[0]
            if len(ok): delta=float(cand[ok[0]])          # 가장 작은 비틀기
            else: delta=float(cand[int(np.argmax(cl))]); vref=min(vref,max(v-s.p['g_brake'],0.5))   # 피할 수 없으면 감속까지
            act=True; s.n_act+=1
        s.last_delta=delta
        return delta,vref,act
