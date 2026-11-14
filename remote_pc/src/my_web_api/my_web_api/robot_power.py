"""로봇 전원 상태 — 웹의 대기·켜기·ROS 재시작·Pi 재부팅·끄기 버튼 (docs/my_web_api.md 4.19).

왜 SSH 인가 (ROS 서비스가 아니라):
  - 대기 상태에선 Pi 의 ROS(bringup)가 꺼져 있다. 켜는 명령은 ROS 바깥 경로로 보내야 한다.
  - zenoh 브리지는 허용 목록의 토픽만 넘기고, 목록을 바꾸려면 Pi 시스템 서비스 재시작(sudo)이 필요하다.
  - 전용 키 + Pi authorized_keys 의 command= 강제로 **robot/tools/robot_power.sh 만** 실행되게 묶는다
    (셸 불가, 포워딩 불가 — restrict). 키를 가진 쪽이 할 수 있는 일은 그 스크립트의 동작(상태·전원 5가지·자이로 재보정)뿐.

상태는 몇 초마다 Pi 에 status 를 물어 갱신한다 (전환 중엔 더 자주). 화면에 보이는 state:
  unconfigured  키·known_hosts 가 없음 (설정 안 됨)
  unreachable   Pi 에 연결 안 됨 (꺼짐·네트워크)          off        '끄기' 뒤 연결 안 됨 (배터리 재연결로 켬)
  rebooting     재부팅 요청 뒤 다시 연결될 때까지
  standby       bringup 꺼짐 (라이다·카메라 정지)          starting   bringup 시작 중 (센서 수신 전)
  running       bringup 실행 + 로봇 데이터 수신             stopping   bringup 정지 중
  error         bringup 실패 (systemd failed)
  calibrating   자이로 재보정 중 (gyro_reset, 5초 — 로봇을 건드리지 말 것)
"""
from __future__ import annotations

import json
import os
import subprocess
import threading
import time

ACTIONS = ('wake', 'standby', 'restart', 'reboot', 'poweroff', 'gyro_reset')
STOPPING_ACTIONS = ('standby', 'restart', 'reboot', 'poweroff')   # 서버 쪽 SLAM·Nav2 를 먼저 끌 동작
ACTION_LABEL = {'wake': '켜기', 'standby': '대기', 'restart': 'ROS 재시작', 'reboot': 'Pi 재부팅', 'poweroff': 'Pi 끄기',
                'gyro_reset': '자이로 재보정'}
POLL_SEC = 5.0
POLL_FAST_SEC = 1.5      # 전환 중
SSH_TIMEOUT = 8.0        # status·wake·restart
STOP_TIMEOUT = 50.0      # standby·poweroff: bringup 정상 종료(최대 30초)까지 기다림
STARTING_GRACE_SEC = 120.0   # 켜기 뒤 로봇 데이터가 들어올 때까지 'starting' 으로 보여 줄 최대 시간
REBOOT_WAIT_SEC = 240.0
GYRO_TIMEOUT = 40.0     # Pi 에서 ros2 CLI 시작(수 초) + OpenCR 영점 5초


class RobotPower:
    def __init__(self, logger, host: str, user: str, key: str, known_hosts: str,
                 robot_connected_fn, before_stop_fn, guard_fn=None, after_fn=None):
        self._log = logger
        self._host, self._user, self._key, self._known = host, user, key, known_hosts
        self._robot_connected = robot_connected_fn   # () -> bool : 로봇 데이터(odom) 수신 중인지
        self._before_stop = before_stop_fn           # () -> None : 서버 쪽 모드를 IDLE 로 (끝날 때까지 기다림)
        self._guard = guard_fn                       # (action) -> str|None : 지금 하면 안 되는 이유 (예: 재보정인데 움직이는 중)
        self._after = after_fn                       # (action, ok) -> None : 끝난 뒤 (예: 재보정 뒤 위치 다시 찾기)
        self._lock = threading.Lock()
        self._pi: dict | None = None                 # 마지막 status 응답
        self._reachable: bool | None = None
        self._checked = 0.0
        self._busy: str | None = None                # 진행 중인 동작
        self._last: dict | None = None               # 마지막 동작 {action, at, ok, message}
        self._wake = threading.Event()
        self.version = 0
        threading.Thread(target=self._poll_loop, daemon=True).start()

    # ---------- 웹에서 부르는 것 ----------
    def configured(self) -> bool:
        return bool(self._host) and os.path.isfile(self._key) and os.path.isfile(self._known)

    def snapshot(self) -> dict:
        with self._lock:
            pi, reachable, busy, last = self._pi, self._reachable, self._busy, self._last
        return {
            'state': self._state(pi, reachable, busy, last),
            'bringup': pi and pi.get('bringup'),
            'uptime': pi and pi.get('uptime'),
            'throttled': pi and pi.get('throttled'),
            'busy': busy,
            'last': last,
        }

    def request(self, action: str) -> dict:
        if action not in ACTIONS:
            return {'ok': False, 'reason': 'bad_request', 'message': f'동작은 {"/".join(ACTIONS)} 중 하나'}
        if not self.configured():
            return {'ok': False, 'reason': 'unavailable',
                    'message': '로봇 전원 제어가 설정되지 않음 (SSH 키 — docs/server_setup.md)'}
        reason = self._guard(action) if self._guard else None
        if reason:
            return {'ok': False, 'reason': 'busy', 'message': reason}
        with self._lock:
            if self._busy:
                return {'ok': False, 'reason': 'busy', 'message': f'{ACTION_LABEL[self._busy]} 진행 중'}
            self._busy = action
            self.version += 1
        threading.Thread(target=self._do, args=(action,), daemon=True).start()
        return {'ok': True, 'message': f'{ACTION_LABEL[action]} 요청'}

    # ---------- 내부 ----------
    def _state(self, pi, reachable, busy, last) -> str:
        if not self.configured():
            return 'unconfigured'
        now = time.time()

        def recent(*actions, sec):   # 마지막 동작이 actions 중 하나이고 성공했고 sec 초 안
            return (last is not None and last['action'] in actions and last['ok'] and now - last['at'] < sec)

        if busy in ('standby', 'poweroff'):
            return 'stopping'
        if busy == 'gyro_reset':
            return 'calibrating'
        if busy == 'reboot':
            return 'rebooting'
        if recent('reboot', sec=REBOOT_WAIT_SEC):
            # 아직 연결 안 됨, 또는 연결됐는데 uptime 이 요청 전부터 → 아직 안 꺼진 것
            if not reachable or (pi or {}).get('uptime', 0) > now - last['at']:
                return 'rebooting'
        if not reachable:
            return 'off' if last is not None and last['action'] == 'poweroff' and last['ok'] else 'unreachable'
        bringup = (pi or {}).get('bringup')
        if bringup == 'deactivating':
            return 'stopping'
        if bringup == 'failed':
            return 'error'
        if bringup == 'activating' or busy in ('wake', 'restart'):
            return 'starting'
        if bringup == 'active':
            # 실행됐는데 로봇 데이터가 아직 없으면: 막 켠 직후면 '시작 중', 오래됐으면 '실행 중'(+ 로봇 응답 없음 표시)
            if not self._robot_connected() and recent('wake', 'restart', sec=STARTING_GRACE_SEC):
                return 'starting'
            return 'running'
        return 'standby'

    def _ssh(self, command: str, timeout: float) -> tuple[bool, dict | None, str]:
        cmd = ['ssh', '-i', self._key, '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
               '-o', 'StrictHostKeyChecking=yes', '-o', f'UserKnownHostsFile={self._known}',
               '-o', 'ConnectTimeout=4', '-o', 'ServerAliveInterval=5', '-o', 'LogLevel=ERROR',
               f'{self._user}@{self._host}', command]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return False, None, f'응답 없음 ({timeout:.0f}초)'
        except OSError as e:
            return False, None, f'ssh 실행 실패: {e}'
        info = None   # 출력의 JSON 줄들을 합친다 (예: {"error": …} 다음 상태 줄)
        for line in r.stdout.strip().splitlines():
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if isinstance(obj, dict):
                info = {**(info or {}), **obj}
        if r.returncode == 255:   # ssh 자체 실패 (연결·인증) 또는 실행 중 연결 끊김 (재부팅·끄기)
            return False, info, (r.stderr.strip().splitlines() or ['연결 끊김'])[-1][:200]
        return r.returncode == 0, info, r.stderr.strip()[-200:]

    def _refresh(self):
        ok, info, _ = self._ssh('status', SSH_TIMEOUT)
        with self._lock:
            changed = (ok and info) != self._pi or self._reachable != bool(ok and info)
            self._reachable = bool(ok and info)
            if self._reachable:
                self._pi = info
            self._checked = time.time()
            if changed:
                self.version += 1

    def _poll_loop(self):
        while True:
            if self.configured():
                try:
                    self._refresh()
                except Exception as e:  # noqa: BLE001
                    self._log.warn(f'로봇 전원 상태 확인 실패: {e}')
            fast = self._busy is not None or self.snapshot()['state'] in ('starting', 'stopping', 'rebooting')
            self._wake.wait(POLL_FAST_SEC if fast else POLL_SEC)
            self._wake.clear()

    def _do(self, action: str):
        ok, message = False, ''
        try:
            if action in STOPPING_ACTIONS:
                self._before_stop()
            timeout = STOP_TIMEOUT if action in ('standby', 'poweroff') else \
                GYRO_TIMEOUT if action == 'gyro_reset' else SSH_TIMEOUT
            ok, info, err = self._ssh(action, timeout)
            if action in ('reboot', 'poweroff') and not ok and info is not None and 'password' not in err:
                ok = True   # 스크립트가 실행됐고(status 출력) 끄는 도중 연결이 먼저 끊긴 것 — 정상
            message = '' if ok else ((info or {}).get('error') or err or '실패')
            if 'password' in err:   # sudo -n 이 규칙 없이 비밀번호를 요구
                message = 'Pi 의 sudo 규칙이 없음 — robot/config/sudoers-robot-power 설치 필요 (docs/pi_setup.md 9-2)'
            self._log.info(f'로봇 전원: {ACTION_LABEL[action]} → {"성공" if ok else "실패: " + message}')
        except Exception as e:  # noqa: BLE001
            message = f'{type(e).__name__}: {e}'
            self._log.error(f'로봇 전원 {action} 오류: {message}')
        finally:
            with self._lock:
                self._busy = None
                self._last = {'action': action, 'at': time.time(), 'ok': ok, 'message': message}
                self.version += 1
            self._wake.set()
            if self._after:
                try:
                    self._after(action, ok)
                except Exception as e:  # noqa: BLE001
                    self._log.warn(f'로봇 전원 {action} 후처리 오류: {e}')
