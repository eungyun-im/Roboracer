#!/usr/bin/env python3
"""미지 트랙 탐색 주행(대회 모드 안전: 허용 센서만, GT/lap/collision 토픽 미사용).
Follow-the-gap(disparity extender) 저속 주행 + 온라인 SLAM(slam_lite) → 한 바퀴 루프 폐합 + overlap[m] 더 주행 후
매핑 기록을 <out>.npz 로 저장(build_track.py 입력 형식). 저장 후에도 탐색 주행·SLAM을 계속하며 현재 pose를
/racer/init_pose 로 발행(위치추정 인계용). racer 가 /racer/active 를 내면 제어를 넘기고 종료.
usage: explore.py --out maps/auto/auto_rec.npz [--speed 2.0 --enc_scale 0.941 --kthr 0.0396 --overlap 4]"""
import os, sys, time, argparse, threading, numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import PoseStamped
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slam_lite import SlamLite, save_rec

P = '/autodrive/roboracer_1/'; MAXSTEER = 0.5236
ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True)
ap.add_argument('--speed', type=float, default=2.0); ap.add_argument('--enc_scale', type=float, default=0.941)
ap.add_argument('--kthr', type=float, default=0.0396); ap.add_argument('--overlap', type=float, default=4.0)
ap.add_argument('--half', type=float, default=0.20)      # disparity extender 차폭 여유
ap.add_argument('--steer_gain', type=float, default=0.9)
a = ap.parse_args()


def ftg(r, amin, ainc, v0, half):
    """disparity extender: 가장 먼 틈(차폭만큼 장애물 확장) 방향으로 조향, 정면 거리로 감속"""
    r = np.nan_to_num(np.asarray(r, float), nan=0.0, posinf=10.0); n = len(r)
    ang = amin + np.arange(n) * ainc; rr = np.clip(r, 0, 10.0).copy(); d = np.diff(r)
    for i in np.where(np.abs(d) > 0.3)[0]:
        near = min(r[i], r[i + 1]); w = int(np.ceil(np.arctan2(half, max(near, 0.1)) / ainc))
        if r[i] < r[i + 1]: rr[i + 1:i + 1 + w] = np.minimum(rr[i + 1:i + 1 + w], near)
        else: rr[max(0, i - w):i + 1] = np.minimum(rr[max(0, i - w):i + 1], near)
    cand = np.where(np.abs(ang) < np.radians(90))[0]
    sm = np.convolve(rr, np.ones(15) / 15, 'same'); i = cand[np.argmax(sm[cand])]
    steer = float(np.clip(a.steer_gain * ang[i], -MAXSTEER, MAXSTEER))
    front = np.min(r[np.abs(ang) < np.radians(10)])
    v = v0 * float(np.clip((front - 0.5) / 3.0, 0.3, 1.0)) * float(np.clip(1.2 - abs(steer) / MAXSTEER, 0.5, 1.0))
    return steer, v


class Explore(Node):
    def __init__(s):
        super().__init__('explore')
        s.slam = SlamLite(enc_scale=a.enc_scale); s.R = []; s.saved = False; s.handed = False
        s.yaw = None; s.wz = 0.0; s.el = s.er = None
        s.create_subscription(Imu, P + 'imu', s.cb_imu, 10)
        s.create_subscription(JointState, P + 'left_encoder', lambda m: setattr(s, 'el', m.position[0]), 10)
        s.create_subscription(JointState, P + 'right_encoder', lambda m: setattr(s, 'er', m.position[0]), 10)
        s.create_subscription(LaserScan, P + 'lidar', s.cb_scan, 10)
        s.create_subscription(Bool, '/racer/active', s.cb_active, 1)
        s.pt = s.create_publisher(Float32, P + 'throttle_command', 1); s.ps = s.create_publisher(Float32, P + 'steering_command', 1)
        s.pi = s.create_publisher(PoseStamped, '/racer/init_pose', 1)
        s.t0 = time.time()

    def cb_imu(s, m):
        q = m.orientation; s.yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))); s.wz = m.angular_velocity.z

    def cb_active(s, m):
        if m.data and not s.handed:
            s.handed = True; print(f'[explore] racer active -> handover done ({time.time() - s.t0:.1f}s)', flush=True)
            threading.Timer(1.0, lambda: os._exit(0)).start()

    def cb_scan(s, m):
        if s.handed or s.yaw is None or s.el is None or s.er is None: return
        steer, v = ftg(m.ranges, m.angle_min, m.angle_increment, a.speed, a.half)
        s.pt.publish(Float32(data=float(np.clip(v * a.kthr, 0, 1)))); s.ps.publish(Float32(data=float(steer / MAXSTEER)))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y = s.slam.add(t, m.ranges, m.angle_min, m.angle_increment, s.yaw, s.wz, 0.5 * (s.el + s.er))
        if not s.saved: s.R.append(np.asarray(m.ranges, np.float32))
        sl = s.slam
        if not s.saved and sl.closed and sl.dist > sl.close_dist + a.overlap:
            tmp = a.out + '.tmp.npz'; save_rec(sl, s.R, tmp); os.replace(tmp, a.out); s.saved = True
            print(f'[explore] map record saved: {a.out} ({len(s.R)} frames, lap {sl.close_dist:.1f} m, {time.time() - s.t0:.1f}s)', flush=True)
        if s.saved:
            ps = PoseStamped(); ps.header = m.header; ps.header.frame_id = 'map'
            ps.pose.position.x = float(x); ps.pose.position.y = float(y)
            ps.pose.orientation.z = float(np.sin(s.yaw / 2)); ps.pose.orientation.w = float(np.cos(s.yaw / 2))
            s.pi.publish(ps)
        if len(sl.poses) % 90 == 0:
            print(f'[explore] dist {sl.dist:.1f} m pose ({x:.2f},{y:.2f}) score {sl.last_score:.2f} closed {sl.closed}', flush=True)


rclpy.init(); n = Explore()
try: rclpy.spin(n)
except (KeyboardInterrupt, SystemExit): pass
