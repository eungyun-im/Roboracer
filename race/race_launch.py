#!/usr/bin/env python3
"""대회 실행기 (제출 컨테이너 엔트리포인트에서 브리지와 함께 실행).
1) 허용 센서(LiDAR, IMU)의 첫 데이터를 기다림(= 시뮬 연결)
2) 첫 스캔을 race/tracks.json 각 트랙 맵의 출발 pose에서 레이캐스팅한 예상 거리와 비교 → 트랙 판별
   (환경변수 TRACK=porto|iros26|explore 로 강제 가능, 기본 auto)
3) 알려진 트랙: localizer + racer 를 대회 모드(--comp 1)로 기동, 죽으면 재기동
   미지 트랙: tools/explore.py(탐색 주행+SLAM) → tools/auto_track.py(맵/라인 생성) → 인계 → 주행,
   이후 최소시간 라인(2단계)이 나오면 racer 가 파일 변경을 감지해 교체(--reload 1)
금지 토픽(/odom,/ips,/tf,lap/collision, reset_command)은 구독/발행하지 않는다."""
import os, sys, json, time, signal, subprocess, threading, shlex
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
P = '/autodrive/roboracer_1/'


def load_cfg():
    return json.load(open(os.path.join(WS, 'race', 'tracks.json')))


def identify(ranges, amin, ainc, yaw, cfg, win=0.2, step=0.05, verbose=True):
    """첫 스캔과 IMU yaw로 트랙 판별: 각 트랙 맵의 출발 pose(±win) 에서 레이캐스팅한 예상 거리와
    실제 빔 거리의 평균 차이(빔당 최대 1m, 한쪽만 반사 없으면 1m)를 점수로 사용. 반환 (이름 또는 None, {이름: 점수})"""
    sys.path.insert(0, os.path.join(WS, 'tools'))
    from scan_sim import load_df, raycast
    r = np.asarray(ranges, float); k = np.arange(0, len(r), 4)
    rm = np.where(np.isfinite(r[k]) & (r[k] > 0.1) & (r[k] < 9.5), r[k], np.inf)
    scores = {}
    for name, t in cfg['tracks'].items():
        z = np.load(os.path.join(WS, t['map']))
        if 'start' not in z.files or len(z['start']) < 3:
            continue
        sx, sy, syaw = map(float, z['start'][:3])
        dyaw = abs((yaw - syaw + np.pi) % (2 * np.pi) - np.pi)
        DF, ox, oy, res = load_df(os.path.join(WS, t['map']))
        best = 1e9
        for dx in np.arange(-win, win + 1e-9, step):
            for dy in np.arange(-win, win + 1e-9, step):
                re = raycast(DF, ox, oy, res, sx + dx, sy + dy, yaw)[k]
                re = np.where(re < 9.5, re, np.inf)
                both = np.isfinite(rm) & np.isfinite(re); one = np.isfinite(rm) ^ np.isfinite(re)
                e = np.where(both, np.minimum(np.abs(np.where(both, rm, 0) - np.where(both, re, 0)), 1.0), 0.0) + one * 1.0
                best = min(best, float(e.mean()))
        scores[name] = best + (1.0 if dyaw > 0.5 else 0.0)   # 출발 방향이 다르면 다른 트랙
    if verbose:
        print('[launch] track scores:', {kk: round(v, 3) for kk, v in scores.items()}, flush=True)
    if not scores:
        return None, scores
    srt = sorted(scores.values()); name = min(scores, key=scores.get)
    gap = (srt[1] - srt[0]) if len(srt) > 1 else 1.0   # 2위와 충분히 차이 나야 확정(모호하면 explore)
    return (name if scores[name] < cfg.get('id_thr', 0.12) and gap > cfg.get('id_gap', 0.04) else None), scores


def wait_first_scan():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, Imu
    rclpy.init(); n = Node('race_launch'); st = {}
    n.create_subscription(LaserScan, P + 'lidar', lambda m: st.__setitem__('scan', m), 1)
    n.create_subscription(Imu, P + 'imu', lambda m: st.__setitem__('imu', m), 1)
    print('[launch] waiting for simulator data (lidar+imu)...', flush=True)
    while not ('scan' in st and 'imu' in st):
        rclpy.spin_once(n, timeout_sec=0.5)
    m, q = st['scan'], st['imu'].orientation
    yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    out = (np.asarray(m.ranges, float), float(m.angle_min), float(m.angle_increment), yaw)
    n.destroy_node(); rclpy.shutdown()
    return out


class Sup:
    """자식 프로세스 감시: 죽으면 재기동(로그는 logs/race_<name>.log)."""
    def __init__(s):
        s.procs = {}; s.cmds = {}; s.restarts = {}; s.stop = False
    def start(s, name, cmd):
        s.cmds[name] = cmd; s.restarts.setdefault(name, 0)
        log = open(os.path.join(WS, 'logs', f'race_{name}.log'), 'a')
        log.write(f'\n=== {time.strftime("%H:%M:%S")} start: {" ".join(cmd)}\n'); log.flush()
        s.procs[name] = subprocess.Popen(cmd, cwd=WS, stdout=log, stderr=subprocess.STDOUT)
        print(f'[launch] started {name} (pid {s.procs[name].pid})', flush=True)
    def kill(s, name):
        p = s.procs.pop(name, None); s.cmds.pop(name, None)
        if p and p.poll() is None:
            p.terminate()
            try: p.wait(3)
            except subprocess.TimeoutExpired: p.kill()
    def poll(s):
        for name, p in list(s.procs.items()):
            if p.poll() is not None and name in s.cmds and not s.stop:
                s.restarts[name] += 1
                print(f'[launch] {name} exited (code {p.returncode}) -> restart #{s.restarts[name]}', flush=True)
                time.sleep(0.2); s.start(name, s.cmds[name])
    def shutdown(s, *_):
        s.stop = True
        for name in list(s.procs): s.kill(name)
        sys.exit(0)


def node_cmds(t, line=None, init_wait=0.0):
    py = sys.executable
    loc = [py, 'tools/localizer.py', '--comp', '1', '--map', t['map'], '--center', t['center'],
           '--enc_scale', str(t['enc_scale'])] + shlex.split(t.get('loc_args', ''))
    if init_wait > 0: loc += ['--init_wait', str(init_wait)]
    racer = [py, 'tools/racer.py', '--comp', '1', '--line', line or t['line']] + shlex.split(t.get('racer_args', ''))
    return loc, racer


def run_known(sup, name, t):
    print(f'[launch] track = {name}: line {t["line"]}', flush=True)
    loc, racer = node_cmds(t)
    sup.start('localizer', loc); time.sleep(1.0); sup.start('racer', racer)
    while True:
        sup.poll(); time.sleep(0.5)


def run_explore(sup, cfg):
    e = cfg['explore']; out = os.path.join('maps', 'auto'); os.makedirs(os.path.join(WS, out), exist_ok=True)
    rec = os.path.join(out, 'auto_rec.npz'); pre = os.path.join(out, 'auto')
    for f in (rec, pre + '_ready.json'):
        if os.path.exists(os.path.join(WS, f)): os.remove(os.path.join(WS, f))
    sup.start('explore', [sys.executable, 'tools/explore.py', '--out', rec, '--speed', str(e['speed']),
                          '--enc_scale', str(e['enc_scale'])] + shlex.split(e.get('explore_args', '')))
    while not os.path.exists(os.path.join(WS, rec)):   # 한 바퀴 매핑(루프 폐합) 완료 대기
        sup.poll(); time.sleep(0.5)
    print('[launch] mapping done -> building track (stage A)', flush=True)
    pa = [sys.executable, 'tools/auto_track.py', rec, pre] + shlex.split(e.get('pipeline_args', ''))
    with open(os.path.join(WS, 'logs', 'race_auto_track.log'), 'a') as lg:
        r = subprocess.run(pa + ['--stage', 'A'], cwd=WS, stdout=lg, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        print('[launch] stage A failed (logs/race_auto_track.log) — 탐색 주행 계속', flush=True)
        while True: sup.poll(); time.sleep(0.5)
    ready = json.load(open(os.path.join(WS, pre + '_ready.json')))
    save_state(dict(track='explore', ready=pre + '_ready.json'))
    t = dict(e, map=ready['map'], center=ready['center'], line=ready['line'])
    loc, racer = node_cmds(t, init_wait=5.0)
    sup.start('localizer', loc); time.sleep(0.5); sup.start('racer', racer)   # explore 노드는 /racer/pose 를 보면 스스로 종료
    sup.cmds.pop('explore', None)
    def stage_b():
        with open(os.path.join(WS, 'logs', 'race_auto_track.log'), 'a') as lg:
            subprocess.run(pa + ['--stage', 'B'], cwd=WS, stdout=lg, stderr=subprocess.STDOUT)
        print('[launch] stage B finished (min-time line; racer reloads if updated)', flush=True)
    threading.Thread(target=stage_b, daemon=True).start()
    while True:
        sup.poll(); time.sleep(0.5)


STATE = os.path.join(WS, 'logs', 'race_state.json')   # 실행기 재기동 시 이어가기(엔트리포인트가 컨테이너 시작 때 삭제)


def save_state(d):
    json.dump(d, open(STATE + '.tmp', 'w')); os.replace(STATE + '.tmp', STATE)


def resume(sup, cfg, st):
    """실행기가 레이스 도중 재기동된 경우: 트랙 재판별(차가 출발점에 없음) 없이 이전 결정으로 노드만 다시 띄움"""
    print(f'[launch] resume: {st}', flush=True)
    subprocess.run(['pkill', '-f', r'tools/(racer|localizer|explore)\.py'])   # 고아 노드 정리(중복 방지)
    if st['track'] == 'explore':
        e = cfg['explore']; ready = json.load(open(os.path.join(WS, st['ready'])))
        loc, racer = node_cmds(dict(e, map=ready['map'], center=ready['center'], line=ready['line']))
    else:
        loc, racer = node_cmds(cfg['tracks'][st['track']])
    sup.start('localizer', loc); time.sleep(1.0); sup.start('racer', racer)
    while True:
        sup.poll(); time.sleep(0.5)


def main():
    os.chdir(WS); os.makedirs(os.path.join(WS, 'logs'), exist_ok=True)
    cfg = load_cfg(); sup = Sup()
    signal.signal(signal.SIGTERM, sup.shutdown); signal.signal(signal.SIGINT, sup.shutdown)
    if os.path.exists(STATE):
        st = json.load(open(STATE))
        if st.get('track') in cfg['tracks'] or (st.get('track') == 'explore' and 'ready' in st):
            return resume(sup, cfg, st)
    track = os.environ.get('TRACK', 'auto')
    if track == 'auto':
        ranges, amin, ainc, yaw = wait_first_scan()
        track, _ = identify(ranges, amin, ainc, yaw, cfg)
        track = track or 'explore'
    if track == 'explore':
        run_explore(sup, cfg)
    elif track in cfg['tracks']:
        save_state(dict(track=track)); run_known(sup, track, cfg['tracks'][track])
    else:
        sys.exit(f'unknown TRACK={track} (choices: auto, explore, {", ".join(cfg["tracks"])})')


if __name__ == '__main__':
    main()
