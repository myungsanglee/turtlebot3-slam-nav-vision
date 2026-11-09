"""monitor.py 단위 테스트 (ROS 불필요): python3 -m pytest remote_pc/src/my_web_api/test"""
from my_web_api.monitor import FrameStore, TopicMonitor, TopicSpec, battery_level


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _monitor(optional=False):
    clock = FakeClock()
    spec = TopicSpec('odom', '/odom', '오도메트리', 20.0, optional)
    return TopicMonitor(spec, window_sec=3.0, stale_sec=2.0, clock=clock), clock


def test_never_received_is_missing():
    m, _ = _monitor()
    s = m.snapshot()
    assert s['state'] == 'missing' and s['rate'] == 0.0 and s['age'] is None


def test_optional_never_received_is_off():
    m, _ = _monitor(optional=True)
    assert m.snapshot()['state'] == 'off'


def test_rate_20hz():
    m, clock = _monitor()
    for _ in range(41):           # 2초 동안 0.05초 간격 = 20Hz
        m.record()
        clock.t += 0.05
    s = m.snapshot()
    assert s['state'] == 'ok'
    assert abs(s['rate'] - 20.0) < 0.6


def test_goes_stale_and_rate_zero():
    m, clock = _monitor()
    for _ in range(10):
        m.record()
        clock.t += 0.05
    clock.t += 2.5                # 2.5초 동안 수신 없음
    s = m.snapshot()
    assert s['state'] == 'missing' and s['rate'] == 0.0 and s['age'] > 2.0


def test_frame_store_version():
    f = FrameStore()
    assert f.get() == (0, None)
    f.put(b'a')
    f.put(b'b')
    assert f.get() == (2, b'b')


def test_battery_level_3s():
    assert battery_level(12.0, 3, 3.5, 3.3) == 'ok'
    assert battery_level(10.2, 3, 3.5, 3.3) == 'warn'
    assert battery_level(9.8, 3, 3.5, 3.3) == 'critical'
