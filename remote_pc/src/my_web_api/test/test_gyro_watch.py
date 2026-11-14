"""gyro_watch: 정지 중 방향 드리프트 판정·자동 재보정 조건."""
import math

from my_web_api.gyro_watch import GyroDriftWatch, auto_reset_reason


def feed(w, seconds, rate_deg_min, x=0.0, wheel=0.0, wheel_rate=0.0, t0=0.0, yaw0=0.0):
    t = t0
    while t <= t0 + seconds:
        yaw = yaw0 + math.radians(rate_deg_min) * (t - t0) / 60.0
        w.add_odom(t, math.atan2(math.sin(yaw), math.cos(yaw)), x, 0.0)
        w.add_wheels(t, (wheel + wheel_rate * (t - t0), wheel))
        t += 0.05
    return t


def test_drift_detected_when_still():
    w = GyroDriftWatch(window_sec=30)
    t = feed(w, 40, -13.4)
    s = w.snapshot(t)
    assert s['state'] == 'drift' and abs(s['rate_deg_min'] + 13.4) < 0.2 and s['wheels']


def test_ok_after_calibration_and_wraparound():
    w = GyroDriftWatch(window_sec=30)
    t = feed(w, 40, 0.03, yaw0=math.pi - 0.0001)   # ±180° 경계를 넘어도 연속으로
    s = w.snapshot(t)
    assert s['state'] == 'ok' and abs(s['rate_deg_min']) < 0.1


def test_moving_wheels_is_not_drift():
    w = GyroDriftWatch(window_sec=30)
    t = feed(w, 40, 30.0, wheel_rate=0.5)   # 제자리 회전 (바퀴가 돎)
    assert w.snapshot(t)['state'] == 'moving'


def test_settling_right_after_stop_and_reset_clears():
    w = GyroDriftWatch(window_sec=30)
    t = feed(w, 10, -13.4)
    assert w.snapshot(t)['state'] == 'settling'
    w.reset()
    assert w.snapshot(t)['state'] == 'unknown'


def test_auto_reset_conditions():
    drift = {'state': 'drift', 'rate_deg_min': -13.4, 'still_sec': 40, 'wheels': True}
    ok = dict(enabled=True, min_deg_min=3.0, mode='NAVIGATION', nav_active=False, teleop_active=False,
              power_state='running', busy=False, since_last=None, cooldown_sec=600)
    assert auto_reset_reason(drift, **ok)
    assert auto_reset_reason(drift, **{**ok, 'mode': 'MAPPING'}) is None        # 지도 만들기 중엔 안 함
    assert auto_reset_reason(drift, **{**ok, 'nav_active': True}) is None
    assert auto_reset_reason(drift, **{**ok, 'since_last': 100}) is None         # 쿨다운
    assert auto_reset_reason({**drift, 'wheels': False}, **ok) is None           # 바퀴 정보 없으면 확신 못 함
    assert auto_reset_reason({**drift, 'rate_deg_min': 2.5}, **ok) is None       # 경고만 하는 수준
    assert auto_reset_reason(drift, **{**ok, 'enabled': False}) is None
