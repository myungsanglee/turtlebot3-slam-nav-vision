#!/usr/bin/env python3
"""주행 녹화(rosbag) 진단: AMCL 이 믿는 위치 vs 라이다로 따로 맞춘 '실제' 위치 vs odom, 속도 — 시간 순서로.

  python3 analyze_bag.py <bag 폴더> <지도 yaml> [--rows 1.0]

'실제' 위치(truth): 스캔마다 저장 지도에 맞춘다 (my_localization scan_matcher). 처음은 지도 전체 검색,
이후는 직전 실제 위치 + odom 이동량으로 예측한 자리 주변 검색 — AMCL·relocalizer 와 독립.
출력:
  1) 시간표: 명령 속도·odom 회전 속도·AMCL 오차(실제 대비 거리·방향)·스캔 일치 점수
  2) 회전 구간: odom 회전량 vs 실제 회전량 (자이로 배율 확인)
  3) 속도별 오차 통계 (빠를수록 틀어지나)
"""
import argparse
import bisect
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.dirname(__file__))
from fake_scan import load_map  # noqa: E402

from my_localization.scan_matcher import (Candidate, MapModel, Scan, angle_diff, consistency, global_search,  # noqa: E402
                                          local_search, rank)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def inverse(p):
    c, s = math.cos(p[2]), math.sin(p[2])
    return -c * p[0] - s * p[1], s * p[0] - c * p[1], -p[2]


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, data, t = r.read_next()
        out.setdefault(topic, []).append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


class Series:
    """시간 → 값 (가장 가까운 앞쪽 값)."""
    def __init__(self, pairs):
        pairs = sorted(pairs, key=lambda p: p[0])
        self.t = [p[0] for p in pairs]
        self.v = [p[1] for p in pairs]

    def at(self, t):
        i = bisect.bisect_right(self.t, t) - 1
        return None if i < 0 else self.v[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('map_yaml')
    ap.add_argument('--rows', type=float, default=1.0, help='시간표 간격 [s]')
    ap.add_argument('--save', help='실제 위치 시계열을 npz 로 저장 (replay_amcl.py 평가용)')
    a = ap.parse_args()

    msgs = read(a.bag)
    grid, res, origin = load_map(a.map_yaml)
    m = MapModel(grid, res, origin)

    # odom (헤더 시각 기준), map→odom (AMCL TF), 명령 속도
    odom = Series([(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_q(msg.pose.pose.orientation),
                     msg.twist.twist.linear.x, msg.twist.twist.angular.z)) for _, msg in msgs['/odom']])
    mo = []
    sensor = None
    for _, tfm in msgs.get('/tf', []) + msgs.get('/tf_static', []):
        for tr in tfm.transforms:
            p = (tr.transform.translation.x, tr.transform.translation.y, yaw_q(tr.transform.rotation))
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                mo.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, p))
            if tr.child_frame_id == 'base_scan':
                sensor = p   # base_link→base_scan (base_footprint 와 x·y 같음)
    map_odom = Series(mo)
    cmd = Series([(t, (msg.linear.x, msg.angular.z)) for t, msg in msgs.get('/cmd_vel', [])])
    inits = [(t, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_q(msg.pose.pose.orientation))
             for t, msg in msgs.get('/initialpose', [])]
    goals = []
    for t, msg in msgs.get('/plan', []):
        if msg.poses:
            g = msg.poses[-1].pose.position
            if not goals or math.hypot(goals[-1][1] - g.x, goals[-1][2] - g.y) > 0.05:
                goals.append((t, g.x, g.y))
    plan = Series([(t, [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]) for t, msg in msgs.get('/plan', [])])
    sensor = sensor or (-0.1, 0.0, 0.0)
    t0 = msgs['/scan'][0][0]
    print(f'라이다 장착 {sensor}, 스캔 {len(msgs["/scan"])}, 시작 {t0:.1f}')
    print('목표:', [(round(t - t0, 1), round(x, 2), round(y, 2)) for t, x, y in goals])
    print('/initialpose:', [(round(t - t0, 1), round(x, 2), round(y, 2), round(math.degrees(th))) for t, x, y, th in inits])

    rows = []
    truth = None
    prev_odom = None
    for t_rx, sc in msgs['/scan']:
        ts = sc.header.stamp.sec + sc.header.stamp.nanosec * 1e-9
        o = odom.at(ts)
        if o is None:
            continue
        angles = sc.angle_min + np.arange(len(sc.ranges)) * sc.angle_increment
        s = Scan(np.asarray(sc.ranges, np.float64), angles, sensor, 0.12, 3.5)
        pts = s.points()
        if len(pts) < 30:
            continue
        ob = o[:3]
        found = None
        if truth is not None and prev_odom is not None:
            pred = compose(truth, compose(inverse(prev_odom), ob))
            c = rank(m, grid, s, [local_search(m, pts, *pred, radius=0.3, angle_deg=25)])[0]
            if c.score >= 0.45:
                found = c
        if found is None:
            cands = rank(m, grid, s, global_search(m, pts))
            if cands and cands[0].score >= 0.5 and (len(cands) == 1 or cands[0].score - cands[1].score >= 0.1):
                found = cands[0]
        if found is None:
            continue
        truth, prev_odom = (found.x, found.y, found.yaw), ob
        mo_ = map_odom.at(ts)
        amcl = compose(mo_, ob) if mo_ else None
        amcl_score = None
        if amcl:
            lik = float(m.score(m.field(0.05), np.array([amcl]), pts)[0])
            amcl_score = lik * consistency(m, grid, s, Candidate(*amcl, lik))
        cv = cmd.at(ts) or (0.0, 0.0)
        # 실제 이동 속도 (스캔으로 맞춘 위치 변화) — 바퀴(odom v)는 도는데 이게 0 이면 미끄러짐·막힘
        tv = 0.0 if not rows else math.hypot(truth[0] - rows[-1]['truth'][0], truth[1] - rows[-1]['truth'][1]) / max(ts - t0 - rows[-1]['t'], 1e-3)
        pl = plan.at(ts)
        pd = '-' if not pl else f'{min(math.hypot(px - truth[0], py - truth[1]) for px, py in pl):.2f}'
        rows.append(dict(t=ts - t0, truth=truth, score=found.score, amcl=amcl, amcl_score=amcl_score,
                         odom=ob, v=o[3], w=o[4], cv=cv[0], cw=cv[1], tv=tv, pd=pd))

    if a.save:
        np.savez(a.save, t=np.array([r['t'] + t0 for r in rows]), pose=np.array([r['truth'] for r in rows]),
                 score=np.array([r['score'] for r in rows]), odom_v=np.array([r['v'] for r in rows]),
                 true_v=np.array([r['tv'] for r in rows]))
        print(f'실제 위치 {len(rows)}개 저장: {a.save}')

    # 1) 시간표
    print('\n시간(s)  명령 v/w   odom v/w      실제 이동속도  실제 x,y,θ°            AMCL 오차 거리(m)/방향(°)  점수 실제/AMCL  경로까지(m)')
    last = -1e9
    for r in rows:
        if r['t'] - last < a.rows:
            continue
        last = r['t']
        tx, ty, tth = r['truth']
        if r['amcl']:
            d = math.hypot(r['amcl'][0] - tx, r['amcl'][1] - ty)
            dth = math.degrees(angle_diff(r['amcl'][2], tth))
            err = f'{d:5.2f} / {dth:+6.1f}'
            asc = f'{r["amcl_score"]:.2f}'
        else:
            err, asc = '   -  /   -  ', '-'
        print(f'{r["t"]:6.1f}  {r["cv"]:+.2f}/{r["cw"]:+.2f}  {r["v"]:+.2f}/{r["w"]:+.2f}   {r["tv"]:5.2f} m/s   '
              f'({tx:5.2f},{ty:5.2f},{math.degrees(tth):+5.0f})   {err}            {r["score"]:.2f}/{asc}       {r["pd"]}')

    # 2) 회전 구간: odom 회전량 vs 실제 회전량 (제자리 회전 |w|>0.3, |v|<0.03 이 2초 이상)
    print('\n회전 구간 (제자리): odom 회전량 vs 실제(스캔) 회전량')
    seg = []
    for r in rows + [None]:
        spinning = r is not None and abs(r['w']) > 0.3 and abs(r['v']) < 0.03
        if spinning:
            seg.append(r)
            continue
        if len(seg) >= 8 and seg[-1]['t'] - seg[0]['t'] > 2.0:
            od = sum(angle_diff(b['odom'][2], a_['odom'][2]) for a_, b in zip(seg, seg[1:]))
            tr = sum(angle_diff(b['truth'][2], a_['truth'][2]) for a_, b in zip(seg, seg[1:]))
            wmax = max(abs(x['w']) for x in seg)
            print(f'  {seg[0]["t"]:6.1f}~{seg[-1]["t"]:6.1f}s  최대 {wmax:.2f} rad/s  odom {math.degrees(od):+7.1f}°  '
                  f'실제 {math.degrees(tr):+7.1f}°  비율 {od / tr if abs(tr) > 1e-3 else float("nan"):.3f}')
        seg = []

    # 3) 회전 속도별 AMCL 방향 오차·실제 스캔 점수
    print('\n회전 속도별: AMCL 방향 오차(중앙값) · 스캔 일치 점수(실제 자리, 중앙값)')
    for lo, hi in ((0, 0.2), (0.2, 0.5), (0.5, 0.8), (0.8, 3.0)):
        sel = [r for r in rows if lo <= abs(r['w']) < hi and r['amcl']]
        if not sel:
            continue
        e = [abs(math.degrees(angle_diff(r['amcl'][2], r['truth'][2]))) for r in sel]
        sc_ = [r['score'] for r in sel]
        print(f'  |w| {lo:.1f}~{hi:.1f} rad/s: {len(sel):4d}장  방향 오차 {np.median(e):5.1f}° (최대 {max(e):5.1f})  '
              f'점수 {np.median(sc_):.2f}')


if __name__ == '__main__':
    main()
