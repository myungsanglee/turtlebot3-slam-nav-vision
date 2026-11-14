"""robot_power: Pi 응답·진행 중 동작·마지막 동작 → 화면 상태 판정 (SSH 없이)."""
import time

import pytest

from my_web_api import robot_power as rp


class Log:
    def info(self, *_):
        pass

    warn = error = info


@pytest.fixture
def power(tmp_path, monkeypatch):
    key, known = tmp_path / 'k', tmp_path / 'kh'
    key.write_text('x')
    known.write_text('x')
    monkeypatch.setattr(rp.RobotPower, '_poll_loop', lambda self: None)   # 백그라운드 SSH 안 함
    p = rp.RobotPower(Log(), '1.2.3.4', 'michael', str(key), str(known), lambda: p.connected, lambda: None)
    p.connected = False
    return p


def state(p, pi=None, reachable=True, busy=None, last=None):
    return p._state(pi, reachable, busy, last)


def test_basic_states(power):
    assert state(power, {'bringup': 'inactive'}) == 'standby'
    assert state(power, {'bringup': 'failed'}) == 'error'
    assert state(power, {'bringup': 'activating'}) == 'starting'
    assert state(power, {'bringup': 'deactivating'}) == 'stopping'
    assert state(power, reachable=False) == 'unreachable'
    power.connected = True
    assert state(power, {'bringup': 'active'}) == 'running'


def test_starting_until_robot_data(power):
    woke = {'action': 'wake', 'at': time.time() - 10, 'ok': True, 'message': ''}
    assert state(power, {'bringup': 'active'}, last=woke) == 'starting'
    power.connected = True
    assert state(power, {'bringup': 'active'}, last=woke) == 'running'
    power.connected = False   # 켠 지 오래됐는데 데이터 없음 → running (로봇 응답 없음 칩이 따로 알림)
    old = dict(woke, at=time.time() - 600)
    assert state(power, {'bringup': 'active'}, last=old) == 'running'


def test_reboot_and_poweroff(power):
    now = time.time()
    rb = {'action': 'reboot', 'at': now - 20, 'ok': True, 'message': ''}
    assert state(power, {'bringup': 'active', 'uptime': 5000}, last=rb) == 'rebooting'   # 아직 안 꺼짐
    assert state(power, reachable=False, last=rb) == 'rebooting'
    assert state(power, {'bringup': 'inactive', 'uptime': 10}, last=rb) == 'standby'      # 다시 켜짐
    off = {'action': 'poweroff', 'at': now, 'ok': True, 'message': ''}
    assert state(power, reachable=False, last=off) == 'off'
    assert state(power, {'bringup': 'inactive'}, busy='poweroff') == 'stopping'


def test_unconfigured_and_validation(tmp_path, monkeypatch):
    monkeypatch.setattr(rp.RobotPower, '_poll_loop', lambda self: None)
    p = rp.RobotPower(Log(), '1.2.3.4', 'michael', str(tmp_path / 'none'), str(tmp_path / 'none'),
                      lambda: False, lambda: None)
    assert p.snapshot()['state'] == 'unconfigured'
    assert p.request('wake')['reason'] == 'unavailable'
    assert p.request('format')['reason'] == 'bad_request'


def test_request_runs_ssh(power, monkeypatch):
    calls = []
    monkeypatch.setattr(power, '_ssh', lambda c, t: (calls.append(c), (True, {'bringup': 'inactive'}, ''))[1])
    stopped = []
    power._before_stop = lambda: stopped.append(1)
    assert power.request('standby')['ok']
    for _ in range(50):
        if power.snapshot()['last']:
            break
        time.sleep(0.02)
    assert calls == ['standby'] and stopped == [1]
    assert power.snapshot()['last']['ok']


def test_reboot_without_sudo_rule(power, monkeypatch):
    monkeypatch.setattr(power, '_ssh', lambda c, t: (False, {'bringup': 'active'}, 'sudo: a password is required'))
    power.request('reboot')
    for _ in range(50):
        if power.snapshot()['last']:
            break
        time.sleep(0.02)
    last = power.snapshot()['last']
    assert not last['ok'] and 'sudoers' in last['message']


def test_gyro_reset_guard_state_and_after(power, monkeypatch):
    """재보정: guard 가 막으면 SSH 안 함, 진행 중엔 'calibrating', 끝나면 after 호출, 오류 JSON 은 메시지로."""
    calls, after = [], []
    power._guard = lambda a: '로봇이 움직이는 중' if power.moving else None
    power._after = lambda a, ok: after.append((a, ok))
    power.moving = True
    monkeypatch.setattr(power, '_ssh', lambda c, t: (calls.append(c), (True, {'bringup': 'active'}, ''))[1])
    r = power.request('gyro_reset')
    assert not r['ok'] and '움직' in r['message'] and calls == []
    power.moving = False
    assert power._state({'bringup': 'active'}, True, 'gyro_reset', None) == 'calibrating'
    assert power.request('gyro_reset')['ok']
    for _ in range(50):
        if after:
            break
        time.sleep(0.02)
    assert calls == ['gyro_reset'] and after == [('gyro_reset', True)]
    # 실패: 스크립트가 {"error"} 줄 + 상태 줄을 내고 exit 4
    after.clear()
    monkeypatch.setattr(power, '_ssh', lambda c, t: (False, {'error': '/reset 실패', 'bringup': 'active'}, ''))
    power.request('gyro_reset')
    for _ in range(50):
        if after:
            break
        time.sleep(0.02)
    assert after == [('gyro_reset', False)] and power.snapshot()['last']['message'] == '/reset 실패'
