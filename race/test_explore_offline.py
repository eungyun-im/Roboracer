#!/usr/bin/env python3
"""미지 트랙 흐름 오프라인 시험(ROS/시뮬 없이): 기존 맵에서 합성한 가상 센서를 explore.py 노드 콜백에 직접 주입
→ 루프 폐합·기록 저장 → auto_track.py(stage A+B) → 생성 맵/중심선을 원래 맵과 비교.
usage: python3 race/test_explore_offline.py [map=maps/iros26_map.npz center=maps/iros26_center.csv] (작업 디렉터리: 임시폴더)"""
import os, sys, runpy, subprocess, tempfile, types, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(WS, 'tools'))
import ros_stub; ros_stub.install()

def main():
    mp = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WS, 'maps/iros26_map.npz')
    ce = sys.argv[2] if len(sys.argv) > 2 else os.path.join(WS, 'maps/iros26_center.csv')
    tmp = tempfile.mkdtemp(prefix='explore_test_'); sens = os.path.join(tmp, 'sens.npz'); out = os.path.join(tmp, 'auto_rec.npz')
    subprocess.run([sys.executable, os.path.join(WS, 'tools/scan_sim.py'), mp, ce, sens, '--v', '2.0', '--laps', '1.4', '--enc_true', '0.941'], check=True)
    sys.argv = ['explore.py', '--out', out, '--speed', '2.0', '--enc_scale', '0.941']
    g = runpy.run_path(os.path.join(WS, 'tools/explore.py'), run_name='__main__'); n = g['n']
    subs = [t for _, t in ros_stub.RECORD['sub']]; pubs = [t for _, t in ros_stub.RECORD['pub']]
    print('explore sub:', sorted(set(subs)), 'pub:', sorted(set(pubs)))
    assert not any(t.endswith(('/odom', '/ips', 'lap_count', 'lap_time', 'collision_count', 'reset_command')) for t in subs + pubs), 'restricted topic'
    d = np.load(sens); M = types.SimpleNamespace
    for i in range(len(d['t'])):
        yaw = float(d['yaw'][i]); t = float(d['t'][i])
        n.cb_imu(M(orientation=M(x=0.0, y=0.0, z=np.sin(yaw / 2), w=np.cos(yaw / 2)), angular_velocity=M(z=float(d['wz'][i]))))
        n.el = n.er = float(d['encL'][i])
        n.cb_scan(M(ranges=d['ranges'][i], angle_min=float(d['angle_min']), angle_increment=float(d['angle_inc']),
                    header=M(stamp=M(sec=int(t), nanosec=int((t % 1) * 1e9)))))
        if n.saved: break
    assert n.saved and os.path.exists(out), 'map record not saved (loop not closed)'
    pre = os.path.join(tmp, 'auto')
    subprocess.run([sys.executable, os.path.join(WS, 'tools/auto_track.py'), out, pre, '--stage', 'AB'], check=True, stdout=subprocess.DEVNULL)
    r = json.load(open(pre + '_ready.json')); print('ready:', r)
    # 정답 맵과 비교(합성 데이터라 출발점 오프셋만 보정)
    from scipy.spatial import cKDTree
    g0 = np.array([d['gx'][0], d['gy'][0]]); A = np.loadtxt(pre + '_center.csv', delimiter=',')[:, :2] + g0
    T = np.loadtxt(ce, delimiter=',')[:, :2]; e, _ = cKDTree(T).query(A)
    print(f'center deviation vs truth: mean {e.mean():.3f} p95 {np.percentile(e, 95):.3f} m')
    assert np.percentile(e, 95) < 0.2, 'centerline deviates too much'
    print('OK', tmp)

if __name__ == '__main__':
    main()
