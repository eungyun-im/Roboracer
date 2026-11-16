#!/usr/bin/env python3
"""localizer.py 오프라인 시험(ROS/시뮬 없이): 가상 센서를 노드 콜백에 주입하고 /racer/pose 를 참값과 비교.
case 1: 출발점에서 시작(맵 START)   case 2: 랩 중간에서 인계 pose(/racer/init_pose, 오차 포함)로 시작
합격 기준 p95<0.15m (합성 데이터의 yaw는 위치 차분이라 곡선에서 약간 어긋나 실제보다 오차가 큼 — 두 경우가 비슷하면 인계 경로 정상)
usage: python3 race/test_localizer_offline.py [track=iros26]"""
import os, sys, runpy, subprocess, tempfile, types
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(WS, 'tools'))
import ros_stub; ros_stub.install()
import race_launch as RL
M = types.SimpleNamespace

def stamp(t): return M(stamp=M(sec=int(t), nanosec=int((t % 1) * 1e9)))

def run(t, d, i0, init=None):
    ros_stub.RECORD['msgs'].clear()
    loc, _ = RL.node_cmds(t, init_wait=(5.0 if init is not None else 0.0))
    sys.argv = loc[1:]; os.chdir(WS)
    n = runpy.run_path(os.path.join(WS, loc[1]), run_name='__main__')['n']
    if init is not None:
        n.cb_init(M(pose=M(position=M(x=init[0], y=init[1]))))
    errs = []
    for i in range(i0, len(d['t'])):
        tt = float(d['t'][i]); yaw = float(d['yaw'][i])
        n.cb_imu(M(orientation=M(x=0.0, y=0.0, z=np.sin(yaw / 2), w=np.cos(yaw / 2)), angular_velocity=M(z=float(d['wz'][i])), linear_acceleration=M(x=0.0)))
        js = M(header=stamp(tt), position=[float(d['encL'][i])]); n.cb_el(js); n.cb_er(js)
        n.cb_scan(M(header=stamp(tt), ranges=d['ranges'][i], angle_min=float(d['angle_min']), angle_increment=float(d['angle_inc'])))
        ps = ros_stub.RECORD['msgs'].get('/racer/pose', [])
        if ps: errs.append(np.hypot(ps[-1].pose.position.x - d['gx'][i], ps[-1].pose.position.y - d['gy'][i]))
    return np.array(errs)

def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'iros26'
    cfg = RL.load_cfg(); t = cfg['tracks'][name]
    tmp = tempfile.mkdtemp(prefix='loc_test_'); sens = os.path.join(tmp, 'sens.npz')
    subprocess.run([sys.executable, os.path.join(WS, 'tools/scan_sim.py'), os.path.join(WS, t['map']), os.path.join(WS, t['line']), sens,
                    '--v', '3.0', '--laps', '1.0', '--wobble', '0.05', '--enc_true', str(t['enc_scale'])], check=True)
    d = np.load(sens); ok = True
    e = run(t, d, 0); print(f'[start]    frames {len(e)}  err mean {e.mean():.3f} p95 {np.percentile(e, 95):.3f} max {e.max():.3f} m'); ok &= np.percentile(e, 95) < 0.15
    i0 = len(d['t']) // 2
    e = run(t, d, i0, init=(float(d['gx'][i0]) + 0.08, float(d['gy'][i0]) - 0.05))
    print(f'[handover] frames {len(e)}  err mean {e.mean():.3f} p95 {np.percentile(e, 95):.3f} max {e.max():.3f} m'); ok &= np.percentile(e, 95) < 0.15
    print('OK' if ok else 'FAIL'); sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()
