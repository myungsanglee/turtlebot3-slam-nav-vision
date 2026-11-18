#!/usr/bin/env python3
"""카메라 depth 의 '바닥 높이' 실측 — camera_layer 의 최소 높이(min_obstacle_height)를 정하는 근거.

  python3 depth_floor_check.py <bag> floor   [--t0 s --t1 s]           빈 바닥: 거리별 높이 분포·평면 맞춤·기준별 오검출
  python3 depth_floor_check.py <bag> object  --t0 s --t1 s             물체: 바닥보다 솟은 칸의 위치·높이
  공통 옵션: --pitch-deg / --roll-deg / --dz  (카메라 자세 보정을 가정하고 다시 계산)

camera_layer 와 같은 계산을 흉내 낸다: depth → 점 → base_footprint 좌표 (TF 는 녹화된 /tf_static 체인),
5cm 칸에 [기준, 25cm] 높이 점이 하나라도 있으면 그 칸은 장애물. 빈 바닥에서 그런 칸 = 오검출.
녹화: /camera/depth/compressed /camera/color/camera_info /tf_static /odom (대기 상태에서도 됨 — Nav2 불필요)
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from my_vision.depth_cloud import PixelRays, depth_to_points  # noqa: E402

CELL = 0.05                       # costmap 해상도
THRESHOLDS = [0.010, 0.015, 0.020, 0.025, 0.030, 0.035, 0.040]
MAX_H = 0.25
BINS = [(0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.0), (1.0, 1.25), (1.25, 1.5), (1.5, 2.0), (2.0, 2.5)]
RANGES = [0.8, 1.0, 1.25, 1.5, 2.0]   # 장애물로 찍는 최대 거리 후보 (obstacle_max_range)


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, data, t = r.read_next()
        out.setdefault(topic, []).append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def chain(tf_static, parent, child):
    """/tf_static 에서 parent → child 4x4 변환."""
    edges = {}
    for _, m in tf_static:
        for tr in m.transforms:
            T = np.eye(4)
            q = tr.transform.rotation
            T[:3, :3] = quat_to_mat((q.x, q.y, q.z, q.w))
            p = tr.transform.translation
            T[:3, 3] = (p.x, p.y, p.z)
            edges[tr.child_frame_id] = (tr.header.frame_id, T)
    T = np.eye(4)
    f = child
    while f != parent:
        if f not in edges:
            raise SystemExit(f'TF 체인 없음: {parent} → {child} ({f} 의 부모 모름)')
        p, Tp = edges[f]
        T = Tp @ T
        f = p
    return T


def correction(pitch_deg, roll_deg, dz):
    """base_footprint 에서 점들에 적용할 보정 (카메라가 실제로는 그만큼 더 숙여져/기울어져 있다고 가정)."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    T = np.eye(4)
    T[:3, :3] = Ry @ Rx
    T[2, 3] = dz
    return T


def frames(msgs, T, t0, t1, stride):
    info = msgs['/camera/color/camera_info'][-1][1]
    odom = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.twist.twist.linear.x, m.twist.twist.angular.z)
            for _, m in msgs.get('/odom', [])]
    ot = np.array([o[0] for o in odom]) if odom else None
    rays = None
    start = msgs['/camera/depth/compressed'][0][0]
    for t_rx, m in msgs['/camera/depth/compressed']:
        rel = t_rx - start
        if rel < t0 or rel > t1:
            continue
        d = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if d is None:
            continue
        h, w = d.shape[:2]
        if rays is None:
            rays = PixelRays(list(info.k), w, h, stride)
        p = depth_to_points(d, rays, 0.2, 3.0).astype(np.float64)
        pb = p @ T[:3, :3].T + T[:3, 3]
        v = wz = 0.0
        if ot is not None:
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            i = int(np.clip(np.searchsorted(ot, ts), 0, len(odom) - 1))
            v, wz = odom[i][1], odom[i][2]
        yield rel, pb, v, wz


def fp_cells(pb, thr, rmax):
    r = np.hypot(pb[:, 0], pb[:, 1])
    sel = (pb[:, 2] >= thr) & (pb[:, 2] <= MAX_H) & (r <= rmax)
    if not sel.any():
        return 0
    c = np.floor(pb[sel, :2] / CELL).astype(np.int64)
    return len(np.unique(c[:, 0] * 100000 + c[:, 1]))


def floor(msgs, T, a):
    data = list(frames(msgs, T, a.t0, a.t1, a.stride))
    print(f'프레임 {len(data)}개 ({a.t0}~{a.t1}s), 점 평균 {np.mean([len(x[1]) for x in data]):.0f}')
    allp = np.concatenate([x[1] for x in data])
    r = np.hypot(allp[:, 0], allp[:, 1])
    flat = np.abs(allp[:, 2]) < 0.08                    # 바닥 후보 (빈 바닥 녹화 전제)
    print('\n거리별 바닥 높이 (cm) — 0 이 정상. 중앙값이 거리에 따라 변하면 카메라 기울기 오차')
    print('  거리(m)     점수      중앙값   1%     99%    99.9%   최대')
    for lo, hi in BINS:
        s = flat & (r >= lo) & (r < hi)
        if s.sum() < 50:
            continue
        z = allp[s, 2] * 100
        print(f'  {lo:4.2f}~{hi:4.2f}  {s.sum():8d}   {np.median(z):+6.2f} {np.percentile(z, 1):+6.2f} '
              f'{np.percentile(z, 99):+6.2f} {np.percentile(z, 99.9):+6.2f} {z.max():+6.2f}')
    # 평면 맞춤 z = a + b·x + c·y (2.0m 이내 바닥 후보)
    s = flat & (r < 2.0)
    A = np.c_[np.ones(s.sum()), allp[s, 0], allp[s, 1]]
    coef = np.linalg.lstsq(A, allp[s, 2], rcond=None)[0]
    pitch = math.degrees(math.atan(coef[1]))
    roll = math.degrees(math.atan(coef[2]))
    print(f'\n바닥 평면 맞춤: 높이 {coef[0] * 100:+.2f}cm, 앞쪽 기울기 {pitch:+.2f}° (+ = 앞 바닥이 높게 보임 → 카메라가 '
          f'실제로 더 숙여짐), 옆 기울기 {roll:+.2f}°')
    print(f'  → 보정 가정 시: --pitch-deg {pitch:.2f} --roll-deg {-roll:.2f} --dz {-coef[0]:.3f}')
    # 프레임별 평면 앞 기울기 (주행 중 차체 숙임 확인)
    tilts = []
    for rel, pb, v, wz in data:
        rr = np.hypot(pb[:, 0], pb[:, 1])
        q = (np.abs(pb[:, 2]) < 0.08) & (rr < 2.0)
        if q.sum() < 300:
            continue
        c = np.linalg.lstsq(np.c_[np.ones(q.sum()), pb[q, 0], pb[q, 1]], pb[q, 2], rcond=None)[0]
        tilts.append((rel, v, math.degrees(math.atan(c[1])), c[0] * 100))
    if tilts:
        tl = np.array(tilts)
        still = np.abs(tl[:, 1]) < 0.005
        for name, m in (('정지', still), ('주행', ~still)):
            if m.sum():
                print(f'  {name} 프레임 {m.sum():4d}: 앞 기울기 {tl[m, 2].mean():+.2f}° (범위 {tl[m, 2].min():+.2f}~'
                      f'{tl[m, 2].max():+.2f}), 높이 {tl[m, 3].mean():+.2f}cm (범위 {tl[m, 3].min():+.2f}~{tl[m, 3].max():+.2f})')
    print('\n오검출 (빈 바닥이 장애물로 찍힌 5cm 칸) — 프레임당 평균 칸 수 / 한 칸이라도 찍힌 프레임 비율')
    print('  기준\\최대거리 ' + ''.join(f'{x:>13.2f}m' for x in RANGES))
    for thr in THRESHOLDS:
        row = []
        for rmax in RANGES:
            n = np.array([fp_cells(pb, thr, rmax) for _, pb, _, _ in data])
            row.append(f'{n.mean():6.2f}/{(n > 0).mean() * 100:4.0f}%')
        print(f'  {thr * 100:4.1f}cm      ' + '  '.join(row))


def obj(msgs, T, a):
    data = list(frames(msgs, T, a.t0, a.t1, a.stride))
    print(f'프레임 {len(data)}개 ({a.t0}~{a.t1}s)')
    cells = {}
    for _, pb, _, _ in data:
        r = np.hypot(pb[:, 0], pb[:, 1])
        s = (pb[:, 2] >= 0.005) & (pb[:, 2] <= MAX_H) & (r <= 2.5)
        c = np.floor(pb[s, :2] / CELL).astype(np.int64)
        for (i, j), z in zip(map(tuple, c), pb[s, 2]):
            cells.setdefault((i, j), []).append(z)
    n = len(data)
    rows = []
    for (i, j), zs in cells.items():
        zs = np.array(zs)
        rows.append(((i + 0.5) * CELL, (j + 0.5) * CELL, np.percentile(zs, 95), zs.max(), len(zs) / n))
    rows.sort(key=lambda x: -x[2])
    print('바닥보다 솟은 칸 (높이 95% 순, 상위 25) — x 앞 +, y 왼쪽 + (로봇 중심 기준)')
    print('  x(m)   y(m)   거리   높이95%  최대   프레임당점')
    for x, y, p95, mx, per in rows[:25]:
        print(f'  {x:+.2f} {y:+.2f}  {math.hypot(x, y):4.2f}   {p95 * 100:5.1f}cm {mx * 100:5.1f}cm  {per:5.1f}')
    print('\n기준별: 장애물로 찍히는 칸 수 (프레임 평균, 2.0m 이내)')
    for thr in THRESHOLDS:
        print(f'  {thr * 100:4.1f}cm: {np.mean([fp_cells(pb, thr, 2.0) for _, pb, _, _ in data]):6.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('mode', choices=['floor', 'object'])
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--stride', type=int, default=4, help='depth_cloud_node 와 같은 솎기 (4)')
    ap.add_argument('--pitch-deg', type=float, default=0.0)
    ap.add_argument('--roll-deg', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    a = ap.parse_args()
    msgs = read(a.bag)
    T = chain(msgs['/tf_static'], 'base_footprint', msgs['/camera/depth/compressed'][0][1].header.frame_id)
    if a.pitch_deg or a.roll_deg or a.dz:
        T = correction(a.pitch_deg, a.roll_deg, a.dz) @ T
    print(f'카메라 위치 (base_footprint 기준): {np.round(T[:3, 3], 4)}, 녹화 길이 '
          f'{msgs["/camera/depth/compressed"][-1][0] - msgs["/camera/depth/compressed"][0][0]:.1f}s')
    (floor if a.mode == 'floor' else obj)(msgs, T, a)


if __name__ == '__main__':
    main()
