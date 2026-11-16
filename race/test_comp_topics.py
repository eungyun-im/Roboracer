#!/usr/bin/env python3
"""대회 모드(--comp 1) 노드가 금지(Restricted) 토픽을 구독/발행하지 않는지 ROS 없이 검사.
usage: python3 race/test_comp_topics.py   (실패 시 종료코드 1)"""
import os, sys, runpy
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(WS, 'tools'))
import ros_stub; ros_stub.install()
P = '/autodrive/roboracer_1/'
RESTRICTED = [P + t for t in ('ips', 'odom', 'lap_count', 'lap_time', 'last_lap_time', 'best_lap_time', 'collision_count')] + ['/tf', '/tf_static', '/autodrive/reset_command']

def run(script, argv):
    ros_stub.RECORD['sub'].clear(); ros_stub.RECORD['pub'].clear()
    sys.argv = [script] + argv
    try:
        runpy.run_path(os.path.join(WS, script), run_name='__main__')
    except SystemExit:
        pass
    return list(ros_stub.RECORD['sub']), list(ros_stub.RECORD['pub'])

def main():
    import race_launch as RL
    cfg = RL.load_cfg(); os.chdir(WS)
    bad = []
    runs = [(n, t, 0.0) for n, t in cfg['tracks'].items()]
    it = cfg['tracks']['iros26']   # 미지 트랙 설정은 실제 맵 대신 iros26 맵/라인으로 인자만 검사
    runs.append(('explore', dict(cfg['explore'], map=it['map'], center=it['center'], line=it['line']), 5.0))
    for name, t, iw in runs:
        loc, racer = RL.node_cmds(t, init_wait=iw)   # 실행기가 실제로 넘기는 인자 그대로
        cases = [(loc[1], loc[2:]), (racer[1], racer[2:])]
        for script, argv in cases:
            sub, pub = run(script, argv)
            hit = [x for _, x in sub + pub if x in RESTRICTED]
            print(f'[{name}] {script}: sub={sorted(set(x for _, x in sub))} pub={sorted(set(x for _, x in pub))}')
            if hit: bad.append((name, script, hit))
    # 비교용: 테스트 모드는 금지 토픽을 구독해야 정상(스텁이 제대로 기록하는지 확인)
    t = cfg['tracks']['iros26']
    sub, _ = run('tools/racer.py', ['--line', t['line'], '--pose', 'loc', '--reset', '0'])
    assert any(x == P + 'lap_count' for _, x in sub), '스텁 기록 실패(테스트 모드에서 lap_count 구독이 보여야 함)'
    if bad:
        print('FAIL: 금지 토픽 사용', bad); sys.exit(1)
    print('OK: 대회 모드 노드는 금지 토픽을 구독/발행하지 않음')

if __name__ == '__main__':
    main()
