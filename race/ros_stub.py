"""ROS 2 없이 노드 코드를 import/생성해 보는 최소 스텁(테스트 전용).
rclpy / std_msgs / sensor_msgs / nav_msgs / geometry_msgs 를 가짜 모듈로 등록하고,
생성된 구독·발행 토픽을 RECORD 에 기록한다. spin()은 즉시 반환."""
import sys, types
RECORD = {'sub': [], 'pub': [], 'msgs': {}}

class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)
    def __getattr__(self, k):   # 중첩 필드(pose.position.x 등) 자동 생성
        if k.startswith('__'): raise AttributeError(k)
        v = _Msg(); setattr(self, k, v); return v

def _msgmod(name, classes):
    m = types.ModuleType(name)
    for c in classes:
        setattr(m, c, type(c, (_Msg,), {}))
    sys.modules[name] = m
    return m

class _Node:
    def __init__(self, name, *a, **k):
        self._name = name
    def create_subscription(self, typ, topic, cb, qos=None):
        RECORD['sub'].append((self._name, topic))
    def create_publisher(self, typ, topic, qos=None):
        RECORD['pub'].append((self._name, topic))
        return types.SimpleNamespace(publish=lambda m, t=topic: RECORD['msgs'].setdefault(t, []).append(m))
    def create_timer(self, period, cb):
        return None
    def get_logger(self):
        return types.SimpleNamespace(info=print, warn=print, error=print)
    def destroy_node(self):
        pass

def install():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda n: None
    rclpy.spin_once = lambda n, timeout_sec=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    node = types.ModuleType('rclpy.node'); node.Node = _Node
    rclpy.node = node
    sys.modules['rclpy'] = rclpy; sys.modules['rclpy.node'] = node
    for pkg in ('std_msgs', 'sensor_msgs', 'nav_msgs', 'geometry_msgs'):
        sys.modules[pkg] = types.ModuleType(pkg)
    _msgmod('std_msgs.msg', ['Float32', 'Int32', 'Bool'])
    _msgmod('sensor_msgs.msg', ['LaserScan', 'Imu', 'JointState'])
    _msgmod('nav_msgs.msg', ['Odometry'])
    _msgmod('geometry_msgs.msg', ['PoseStamped', 'Point'])
