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


def test_motor_watch_torque_and_no_response():
    from my_web_api.motor_watch import MotorWatch
    m = MotorWatch()
    assert m.snapshot(0.0)['state'] == 'unknown'
    m.set_torque(0.0, False)
    assert m.snapshot(1.0) == {'state': 'torque_off', 'torque': False}
    assert m.snapshot(5.0)['torque'] is None                 # 3초 넘게 소식 없으면 모름
    m.set_torque(10.0, True)
    # 전진 명령 2초 + 바퀴 안 돎 → 무반응
    for i in range(21):
        t = 10.0 + i * 0.1
        m.add_cmd(t, 0.1, 0.0)
        m.add_wheels(t, [0.0, 0.0])
    m.set_torque(12.0, True)
    assert m.snapshot(12.0)['state'] == 'no_response'
    # 바퀴가 돌면 해제
    m.add_wheels(12.1, [2.0, 2.0])
    assert m.snapshot(12.1)['state'] == 'ok'
    # 명령 1초뿐 → 아직 무반응 아님 / 아주 작은 명령은 무시
    m2 = MotorWatch()
    m2.set_torque(0.0, True)
    for i in range(10):
        m2.add_cmd(i * 0.1, 0.1, 0.0)
        m2.add_wheels(i * 0.1, [0.0, 0.0])
    for i in range(30):
        m2.add_cmd(1.0 + i * 0.1, 0.005, 0.05)
    assert m2.snapshot(2.0)['state'] == 'ok'


def test_motor_watch_position_fallback():
    from my_web_api.motor_watch import MotorWatch
    m = MotorWatch()
    m.set_torque(0.0, True)
    for i in range(25):
        t = i * 0.1
        m.add_cmd(t, 0.1, 0.0)
        m.add_wheels(t, [], [0.3 * t, 0.3 * t])          # 속도 칸이 비어도 위치 변화(3 rad/s)로 돈다고 판단
    assert m.snapshot(2.5)['state'] == 'ok'
