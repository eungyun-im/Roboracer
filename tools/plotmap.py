import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
d=np.load(sys.argv[1]); out=sys.argv[2]; tmax=float(sys.argv[3]) if len(sys.argv)>3 else 1e9
m=d['t']<tmax; R=d['ranges'][m]; n=R.shape[1]; ang=np.radians(-135)+np.arange(n)*np.radians(270)/(n-1)
pts=[]
for i in range(0,m.sum(),3):
    r=R[i]; ok=(r>0.06)&(r<9.5); a=ang[ok]+d['yaw'][m][i]
    yw=d['yaw'][m][i]; lx=d['x'][m][i]+0.2733*np.cos(yw); ly=d['y'][m][i]+0.2733*np.sin(yw)
    pts.append(np.c_[lx+r[ok]*np.cos(a), ly+r[ok]*np.sin(a)])
P=np.vstack(pts); plt.figure(figsize=(12,12)); plt.plot(P[:,0],P[:,1],',k',alpha=0.3)
plt.plot(d['x'][m],d['y'][m],'-b',lw=0.8); c=d['coll'][m]; j=np.where(np.diff(c)>0)[0]
plt.plot(d['x'][m][j],d['y'][m][j],'rx',ms=10); plt.plot(d['x'][0],d['y'][0],'go',ms=10)
plt.axis('equal'); plt.grid(); plt.savefig(out,dpi=70)
