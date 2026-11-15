#!/usr/bin/env python3
"""녹화(rosbag)를 AMCL 에 다시 흘려 넣어 설정별 위치 정확도 비교 — 로봇 없이 AMCL 튜닝.

  python3 replay_amcl.py <bag> <지도 yaml> <amcl 파라미터 yaml> <truth.npz> [--rate 4] [--set key=value ...]

- 별도 ROS_DOMAIN_ID(기본 78)에서 map_server + amcl 을 띄우고 sim 시간(/clock)으로 재생한다 (실제 로봇과 안 섞임).
- 녹화의 /tf 중 map→odom(당시 AMCL 이 낸 것)은 빼고 odom→base·정적 TF 와 /scan 만 보낸다.
- 시작 위치는 truth(analyze_bag.py --save) 첫 값으로 /initialpose.
- 결과: truth 와 새 AMCL 위치(map→odom ∘ odom→base)의 거리·방향 오차 통계 (구간별).
"""
import argparse
import bisect
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

DOMAIN = os.environ.get('REPLAY_DOMAIN', '78')
os.environ['ROS_DOMAIN_ID'] = DOMAIN

import rclpy  # noqa: E402
import rosbag2_py  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def adiff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in topics:
            out.append((t * 1e-9, topic, deserialize_message(data, get_message(types[topic]))))
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('map_yaml')
    ap.add_argument('params')
    ap.add_argument('truth')
    ap.add_argument('--rate', type=float, default=4.0)
    ap.add_argument('--set', nargs='*', default=[], help='amcl 파라미터 덮어쓰기 key=value')
    ap.add_argument('--skip', nargs='*', default=[], help='평가에서 뺄 구간 (bag 시작 기준 초) a-b')
    ap.add_argument('--relocalizer', action='store_true', help='my_localization relocalizer 도 함께 (정지 중 보정·위치 잃음 복구 포함 — --rate 1 권장)')
    a = ap.parse_args()

    # 파라미터 파일: amcl 부분만 + 덮어쓰기 + sim 시간
    with open(a.params) as f:
        allp = yaml.safe_load(f)
    ap_ = allp['amcl']['ros__parameters']
    ap_['use_sim_time'] = True
    ap_['set_initial_pose'] = False
    for kv in a.set:
        k, v = kv.split('=')
        ap_[k] = yaml.safe_load(v)
    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({'amcl': {'ros__parameters': ap_},
                    'map_server': {'ros__parameters': {'use_sim_time': True, 'yaml_filename': a.map_yaml}}}, tmp)
    tmp.close()
    # relocalizer 는 지도 옆에 마지막 위치(.pose.json)를 쓰므로 임시 폴더의 지도 사본을 준다 (실제 지도 파일 보호)
    import shutil
    mdir = tempfile.mkdtemp()
    with open(a.map_yaml) as f:
        my = yaml.safe_load(f)
    shutil.copy(os.path.join(os.path.dirname(a.map_yaml), my['image']), mdir)
    shutil.copy(a.map_yaml, mdir)
    map_copy = os.path.join(mdir, os.path.basename(a.map_yaml))
    extra = []
    if a.relocalizer:
        from ament_index_python.packages import get_package_share_directory
        rp = os.path.join(get_package_share_directory('my_localization'), 'config', 'relocalizer.yaml')
        extra = [['ros2', 'run', 'my_localization', 'relocalizer_node', '--ros-args', '--params-file', rp,
                  '-p', 'use_sim_time:=true', '-p', f'map_yaml:={map_copy}']]
    procs = [subprocess.Popen(c, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True) for c in extra] + [subprocess.Popen(c, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True) for c in (
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args', '--params-file', tmp.name],
        ['ros2', 'run', 'nav2_amcl', 'amcl', '--ros-args', '--params-file', tmp.name],
        ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args', '-p', 'use_sim_time:=true',
         '-p', 'autostart:=true', '-p', "node_names:=['map_server','amcl']"])]
    try:
        run(a, procs)
    finally:
        # ros2 run 은 SIGINT 를 실제 노드에 전달하지 않을 때가 있다 → 프로세스 그룹째 종료
        for p in procs:
            try:
                os.killpg(p.pid, 2)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, 9)
        os.unlink(tmp.name)


def run(a, procs):
    msgs = read(a.bag, {'/scan', '/tf', '/tf_static', '/odom'})
    truth = np.load(a.truth)
    tt, tp = truth['t'], truth['pose']
    odom = sorted((stamp(m), (m.pose.pose.position.x, m.pose.pose.position.y, yaw_q(m.pose.pose.orientation)))
                  for _, topic, m in msgs if topic == '/odom')
    odom_t = [o[0] for o in odom]

    rclpy.init()
    n = rclpy.create_node('replay')
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_clock = n.create_publisher(Clock, '/clock', 10)
    pub_tf = n.create_publisher(TFMessage, '/tf', 100)
    pub_static = n.create_publisher(TFMessage, '/tf_static', latched)
    pub_scan = n.create_publisher(LaserScan, '/scan', 10)
    pub_init = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    map_odom = []   # (stamp, pose) — 새 AMCL 이 낸 map→odom

    def on_tf(m):
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                map_odom.append((stamp(tr), (tr.transform.translation.x, tr.transform.translation.y, yaw_q(tr.transform.rotation))))
    n.create_subscription(TFMessage, '/tf', on_tf, 100)

    for _, topic, m in msgs:
        if topic == '/tf_static':
            pub_static.publish(m)
    t_start = msgs[0][0]
    # AMCL 이 활성화될 때까지 시계를 시작 시각에 두고 기다린다
    clk = Clock()
    deadline = time.time() + 20
    while time.time() < deadline and n.count_subscribers('/scan') == 0:
        clk.clock.sec, clk.clock.nanosec = int(t_start), int((t_start % 1) * 1e9)
        pub_clock.publish(clk)
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(2.0)
    # 시작 위치 = truth 첫 값 (truth 시각까지 재생한 뒤 보낸다)
    init_sent = False
    wall0 = time.time()
    for t, topic, m in msgs:
        while (time.time() - wall0) * a.rate < t - t_start:
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(0.002)
        sim = t
        clk.clock.sec, clk.clock.nanosec = int(sim), int((sim % 1) * 1e9)
        pub_clock.publish(clk)
        if topic == '/tf':
            keep = [tr for tr in m.transforms if not (tr.header.frame_id == 'map' and tr.child_frame_id == 'odom')]
            if keep:
                m.transforms = keep
                pub_tf.publish(m)
        elif topic == '/scan':
            pub_scan.publish(m)
            if not init_sent and stamp(m) >= tt[0]:
                x, y, th = tp[0]
                ip = PoseWithCovarianceStamped()
                ip.header.frame_id = 'map'
                ip.header.stamp = m.header.stamp
                ip.pose.pose.position.x, ip.pose.pose.position.y = float(x), float(y)
                ip.pose.pose.orientation.z, ip.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
                cov = [0.0] * 36
                cov[0] = cov[7] = 0.01
                cov[35] = math.radians(5) ** 2
                ip.pose.covariance = cov
                pub_init.publish(ip)
                init_sent = True
        rclpy.spin_once(n, timeout_sec=0.0)
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    # 평가
    if not map_odom:
        print('새 AMCL 이 map→odom 을 내지 않음 (활성화·초기 위치 실패)')
        return
    map_odom.sort()
    mt = [x[0] for x in map_odom]
    skips = [tuple(float(v) for v in s.split('-')) for s in a.skip]
    rows = []
    for ti, (x, y, th) in zip(tt, tp):
        rel = ti - t_start
        if any(lo <= rel <= hi for lo, hi in skips):
            continue
        i = bisect.bisect_right(mt, ti) - 1
        j = bisect.bisect_right(odom_t, ti) - 1
        if i < 0 or j < 0:
            continue
        p = compose(map_odom[i][1], odom[j][1])
        rows.append((rel, math.hypot(p[0] - x, p[1] - y), abs(math.degrees(adiff(p[2], th)))))
    r = np.array(rows)
    d, ang = r[:, 1], r[:, 2]
    print(f'설정: {" ".join(a.set) or "(그대로)"}')
    print(f'  위치 오차  중앙 {np.median(d)*100:5.1f}cm  95% {np.percentile(d, 95)*100:5.1f}cm  최대 {d.max()*100:5.1f}cm   '
          f'방향 오차 중앙 {np.median(ang):4.1f}°  95% {np.percentile(ang, 95):4.1f}°   ({len(r)}개)')
    for lo, hi, name in ((115, 122, '출발 전 정지'), (122, 128, '출발~모퉁이'), (157, 172, '빠져나온 뒤 주행'), (172, 257, '도착 후 정지')):
        sel = r[(r[:, 0] >= lo) & (r[:, 0] < hi)]
        if len(sel):
            print(f'    {name:10s} {lo:3d}~{hi:3d}s: 위치 중앙 {np.median(sel[:, 1])*100:5.1f}cm  최대 {sel[:, 1].max()*100:5.1f}cm')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
