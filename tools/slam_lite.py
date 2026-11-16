#!/usr/bin/env python3
"""경량 2D SLAM (허용 센서만: IMU yaw + 엔코더 속도 + LiDAR). GT 미사용.
- IMU yaw가 절대값(노이즈 없음)이므로 자세는 yaw 고정, 위치(x,y)만 추정
- 예측: 엔코더 속도 추측항법(물리 가속한계로 휠스핀 클리핑)
- 보정: 지금까지 만든 우도격자(likelihood grid)에 스캔 정합(CSM, ±win 격자탐색 + 포물선 보간)
- 루프 폐합: 한 바퀴 뒤 출발 지점 근처에서 출발 직후 스캔으로 만든 서브맵에 넓은 탐색 정합 →
  누적 오차를 주행거리 비례로 분배, 격자 재생성
좌표계: 출발 위치 (0,0), 방향은 IMU yaw(시뮬 월드와 같은 방향).
오프라인: slam_lite.py <sensor.npz> <out_rec.npz>  (scan_sim.py 출력 또는 explore.py 기록)
출력 rec.npz 는 build_track.py 입력 형식(x,y,yaw,ranges,coll + traj 구간)."""
import sys, numpy as np

LIDAR_X = 0.2733; RW = 0.059


class SlamLite:
    def __init__(s, res=0.05, size=200.0, win=0.3, sig=0.05, beams=4, enc_scale=0.941,
                 amax=4.5, amin=8.0, close_min=15.0, close_r=6.0, close_win=1.2, close_score=0.7,
                 prior=10.0, close_sharp=0.15, lap_r=1.0):
        s.res = res; s.N = int(size / res); s.o = -size / 2
        s.L = np.zeros((s.N, s.N), np.float32)
        s.win = win; s.beams = beams; s.enc_scale = enc_scale; s.amax = amax; s.amin = amin
        r = 2; ks = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                w = np.exp(-0.5 * ((dx * res) ** 2 + (dy * res) ** 2) / sig ** 2)
                if w > 0.05: ks.append((dx, dy, np.float32(w)))
        s.K = ks
        s.x = s.y = 0.0; s.v = 0.0; s.dist = 0.0; s.t = None; s.enc = None; s.tenc = None
        s.poses = []; s.pls = []; s.dists = []; s.idx_raw = []
        s.L0 = None; s.n0 = 0; s.closed = False; s.close_at = None; s.corr = (0.0, 0.0)
        s.close_min, s.close_r, s.close_win, s.close_score = close_min, close_r, close_win, close_score
        s.prior = prior; s.close_sharp = close_sharp; s.lap_r = lap_r; s.close_tried = 0
        s.last_score = 0.0

    # ---- 격자 ----
    def _cells(s, W):
        return np.floor((W[:, 0] - s.o) / s.res).astype(int), np.floor((W[:, 1] - s.o) / s.res).astype(int)

    def splat(s, W, G=None):
        G = s.L if G is None else G
        ix, iy = s._cells(W); ok = (ix > 2) & (iy > 2) & (ix < s.N - 3) & (iy < s.N - 3)
        ix, iy = ix[ok], iy[ok]
        for dx, dy, w in s.K:   # 최댓값 갱신 = 가장 가까운 관측점까지 거리 기반 우도장(누적합은 포화돼 정합이 미끄러짐)
            np.maximum.at(G, (iy + dy, ix + dx), w)

    def world(s, x, y, yaw, pl):
        c, sn = np.cos(yaw), np.sin(yaw)
        return np.c_[x + c * pl[:, 0] - sn * pl[:, 1], y + sn * pl[:, 0] + c * pl[:, 1]]

    def match(s, x, y, yaw, pl, win, G=None, prior=None, sharp=False):
        """격자 탐색 정합. prior: 이동량 벌점(평행 벽 복도에서 종방향 미끄러짐 억제 — 그 방향은 추측항법이 지배).
        sharp=True면 (x,y,score,선명도) 반환: 선명도=최고점 - (0.3m 밖 최고점), 모호한 정합(긴 직선) 판별용"""
        prior = s.prior if prior is None else prior
        G = s.L if G is None else G
        W = s.world(x, y, yaw, pl)
        fx = (W[:, 0] - s.o) / s.res - 0.5; fy = (W[:, 1] - s.o) / s.res - 0.5
        ix0 = np.round(fx).astype(int); iy0 = np.round(fy).astype(int)
        n = int(round(win / s.res)); off = np.arange(-n, n + 1)
        OX, OY = np.meshgrid(off, off); OX = OX.ravel(); OY = OY.ravel()
        ii = np.clip(iy0[None, :] + OY[:, None], 0, s.N - 1); jj = np.clip(ix0[None, :] + OX[:, None], 0, s.N - 1)
        sc = G[ii, jj].mean(1)
        sc2 = sc - prior * ((OX * s.res) ** 2 + (OY * s.res) ** 2)
        b = int(np.argmax(sc2)); m = len(off); S = sc2.reshape(m, m); r, c = divmod(b, m)
        ddx = ddy = 0.0
        if 0 < c < m - 1:
            l, mm, rr = S[r, c - 1], S[r, c], S[r, c + 1]; den = l - 2 * mm + rr
            if den < 0: ddx = 0.5 * (l - rr) / den * s.res
        if 0 < r < m - 1:
            l, mm, rr = S[r - 1, c], S[r, c], S[r + 1, c]; den = l - 2 * mm + rr
            if den < 0: ddy = 0.5 * (l - rr) / den * s.res
        if sharp:
            if abs(OX[b]) >= n - 1 or abs(OY[b]) >= n - 1:   # 탐색창 끝 = 능선이 창 밖으로 이어짐(모호)
                return x, y, float(sc[b]), 0.0
            far = (OX - OX[b]) ** 2 + (OY - OY[b]) ** 2 > (0.3 / s.res) ** 2
            return x + OX[b] * s.res + ddx, y + OY[b] * s.res + ddy, float(sc[b]), float(sc[b] - (sc[far].max() if far.any() else 0.0))
        return x + OX[b] * s.res + ddx, y + OY[b] * s.res + ddy, float(sc[b])

    # ---- 센서 ----
    def points(s, ranges, amin, ainc):
        r = np.asarray(ranges, float); k = np.arange(0, len(r), s.beams); rr = r[k]
        ok = np.isfinite(rr) & (rr > 0.1) & (rr < 9.5); ang = amin + k * ainc
        return np.c_[LIDAR_X + rr[ok] * np.cos(ang[ok]), rr[ok] * np.sin(ang[ok])]

    def speed(s, t, enc):
        if s.enc is not None and t > s.tenc:
            de = enc - s.enc
            if abs(de) < 20:
                dt = min(t - s.tenc, 0.2); vr = de / (t - s.tenc) * RW * s.enc_scale
                s.v = float(np.clip(vr, s.v - s.amin * dt, s.v + s.amax * dt))
        s.enc, s.tenc = enc, t
        return s.v

    # ---- 1 프레임 ----
    def add(s, t, ranges, amin, ainc, yaw, wz, enc):
        pl = s.points(ranges, amin, ainc); v = s.speed(t, enc)
        if s.t is not None:
            dt = max(0.0, min(t - s.t, 0.2))
            nx = s.x + v * dt * np.cos(yaw - wz * dt / 2); ny = s.y + v * dt * np.sin(yaw - wz * dt / 2)
            s.dist += np.hypot(nx - s.x, ny - s.y); s.x, s.y = nx, ny
        s.t = t
        if len(s.poses) >= 1 and len(pl) > 20:
            s.x, s.y, s.last_score = s.match(s.x, s.y, yaw, pl, s.win)
        if not s.closed and s.dist > s.close_min:
            dstart = np.hypot(s.x - s.poses[0][0], s.y - s.poses[0][1])
            if dstart < s.close_r: s.try_close(yaw, pl)
            if not s.closed and dstart < s.lap_r:   # 선명한 폐합 정합이 없었으면(드리프트 작음 → 증분 정합이 이미 맞춤) 한 바퀴 완료로만 표시
                s.closed = True; s.close_at = len(s.poses); s.close_dist = s.dist
                print(f'[slam] lap complete at {s.dist:.1f} m (no explicit correction; tried {s.close_tried})', flush=True)
        s.poses.append((s.x, s.y, yaw)); s.pls.append(pl); s.dists.append(s.dist)
        s.splat(s.world(s.x, s.y, yaw, pl))
        if s.L0 is None or (s.dist < 0.5 and s.n0 < 15):   # 출발 서브맵(루프 폐합 기준)
            if s.L0 is None: s.L0 = np.zeros_like(s.L)
            s.splat(s.world(s.x, s.y, yaw, pl), s.L0); s.n0 += 1
        return s.x, s.y

    def try_close(s, yaw, pl):
        x, y, sc, sh = s.match(s.x, s.y, yaw, pl, s.close_win, G=s.L0, prior=0.02, sharp=True); s.close_tried += 1
        if sc < s.close_score or sh < s.close_sharp: return False   # 약하거나 모호(긴 직선 등)하면 다음 프레임에 재시도
        dx, dy = x - s.x, y - s.y; D = max(s.dist, 1e-6)
        P = np.array(s.poses, float); d = np.array(s.dists)
        P[:, 0] += dx * d / D; P[:, 1] += dy * d / D
        s.poses = [tuple(p) for p in P]; s.x, s.y = x, y
        s.L[:] = 0
        for (px, py, pyaw), q in zip(s.poses, s.pls): s.splat(s.world(px, py, pyaw, q))
        s.closed = True; s.close_at = len(s.poses); s.close_dist = s.dist; s.corr = (dx, dy)
        print(f'[slam] loop closed at {s.dist:.1f} m: correction ({dx:+.3f},{dy:+.3f}) score {sc:.2f} sharp {sh:.2f}', flush=True)
        return True

    def lap_frames(s):
        """루프 폐합 지점까지(한 바퀴) 궤적 인덱스"""
        return np.arange(s.close_at if s.closed else len(s.poses))


def run_offline(sensor_npz, out, enc_scale=0.941, overlap=4.0, **kw):
    d = np.load(sensor_npz); R = d['ranges']
    sl = SlamLite(enc_scale=enc_scale, **kw)
    amin = float(d['angle_min']) if 'angle_min' in d.files else np.radians(-135)
    ainc = float(d['angle_inc']) if 'angle_inc' in d.files else np.radians(270) / (R.shape[1] - 1)
    enc = 0.5 * (d['encL'] + d['encR'])
    n = len(R)
    for i in range(n):
        sl.add(float(d['t'][i]), R[i], amin, ainc, float(d['yaw'][i]), float(d['wz'][i]), float(enc[i]))
        if sl.closed and sl.dist > sl.close_dist + overlap: break
    save_rec(sl, R[:len(sl.poses)], out)
    return sl, d


def save_rec(sl, ranges, out):
    P = np.array(sl.poses)
    np.savez_compressed(out, x=P[:, 0], y=P[:, 1], yaw=P[:, 2], ranges=np.asarray(ranges, np.float32),
                        coll=np.zeros(len(P), int), lap_end=np.int64(sl.close_at if sl.closed else len(P)),
                        closed=np.bool_(sl.closed), corr=np.array(sl.corr))


if __name__ == '__main__':
    if len(sys.argv) < 3: sys.exit(__doc__)
    sl, d = run_offline(sys.argv[1], sys.argv[2], enc_scale=float(sys.argv[3]) if len(sys.argv) > 3 else 0.941)
    P = np.array(sl.poses)
    if 'gx' in d.files:   # 합성 데이터: 참값 대비 오차(출발점 정렬)
        gx = d['gx'][:len(P)] - d['gx'][0]; gy = d['gy'][:len(P)] - d['gy'][0]
        e = np.hypot(P[:, 0] - gx, P[:, 1] - gy)
        print(f'pose err vs truth: mean {e.mean():.3f} p95 {np.percentile(e, 95):.3f} max {e.max():.3f} m (closed={sl.closed})')
    print('saved', sys.argv[2], len(P), 'frames')
