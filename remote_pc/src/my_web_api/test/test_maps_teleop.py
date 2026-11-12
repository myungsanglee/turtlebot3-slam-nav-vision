"""maps.py · teleop.to_velocity 단위 테스트 (ROS 불필요)."""
import os

import cv2
import numpy as np
import pytest

from my_web_api.maps import MapStore, valid_name
from my_web_api.teleop import to_velocity


def test_valid_name():
    assert valid_name('living_room-1')
    for bad in ('', '../x', 'a/b', 'a b', 'x' * 41):
        assert not valid_name(bad)


def _write_map(d, name):
    img = np.full((20, 30), 205, np.uint8)
    img[5:15, 5:25] = 254
    img[10, 5:25] = 0
    cv2.imwrite(os.path.join(d, f'{name}.pgm'), img)
    open(os.path.join(d, f'{name}.yaml'), 'w').write(
        f'image: {name}.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.0, -0.5, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


def test_list_thumbnail_delete(tmp_path):
    s = MapStore(str(tmp_path))
    _write_map(tmp_path, 'room')
    (tmp_path / 'junk.yaml').write_text('broken')     # 깨진 파일은 무시
    maps = s.list()
    assert [m['name'] for m in maps] == ['room'] and maps[0]['size_m'] == [1.5, 1.0]
    png = cv2.imdecode(np.frombuffer(s.thumbnail('room'), np.uint8), cv2.IMREAD_UNCHANGED)
    assert png.shape == (20, 30, 4)
    assert s.delete('room') and s.list() == [] and not s.delete('room')


def test_path_rejects_traversal(tmp_path):
    with pytest.raises(ValueError):
        MapStore(str(tmp_path)).path('../etc', 'yaml')


def test_to_velocity_direction_and_clamp():
    assert to_velocity(0, 1, 0.2, 1.0) == (0.2, -0.0)          # 위 = 전진
    v, w = to_velocity(-1, 0, 0.2, 1.0)
    assert v == 0.0 and w == 1.0                                 # 왼쪽 = 좌회전(+)
    assert to_velocity(0, 5, 0.2, 1.0)[0] == 0.2                 # 범위 밖은 자름
    assert to_velocity(0.05, 0.05, 0.2, 1.0) == (0.0, -0.0)      # 데드존


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append((msg.linear.x, msg.angular.z))


class _FakeNode:
    def __init__(self):
        self.pub = _FakePub()

    def create_publisher(self, *a, **k):
        return self.pub

    def create_timer(self, *a, **k):
        return None


def test_teleop_watchdog_stops_and_goes_quiet():
    from my_web_api.teleop import STOP_REPEAT, TeleopController
    clock = [0.0]
    started = []
    node = _FakeNode()
    t = TeleopController(node, '/cmd_vel', 0.2, 1.0, timeout=0.3, on_start=lambda: started.append(1), clock=lambda: clock[0])
    t.command(0, 1)                      # 전진
    t._tick()
    assert node.pub.sent[-1] == (0.2, -0.0) and started == [1]
    clock[0] += 0.5                      # 0.3초 넘게 명령 없음 (손 뗌·연결 끊김)
    for _ in range(STOP_REPEAT + 3):
        t._tick()
    stops = node.pub.sent[1:]
    assert stops == [(0.0, 0.0)] * STOP_REPEAT   # 정지를 몇 번 보내고 조용해진다 (Nav2 명령과 싸우지 않게)
    t.command(1, 0)
    t._tick()
    assert node.pub.sent[-1] == (0.0, -1.0) and started == [1, 1]
