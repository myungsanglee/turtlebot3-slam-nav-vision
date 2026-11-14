"""운용 모드 관리 — 웹 버튼으로 SLAM·Nav2·Vision 을 켜고 끈다 (docs/web_ui_design.md 4장).

  IDLE        아무것도 안 띄움 (web-api 만). 라이다·카메라·조이스틱은 그대로 쓸 수 있다
  MAPPING     system.launch.py use_slam:=true            → slam_toolbox + Nav2 (+ Vision)
  NAVIGATION  system.launch.py use_slam:=false map:=…    → 저장 지도 + AMCL + Nav2 (+ Vision)

launch 는 이 프로세스의 자식으로 띄우되 **새 세션(프로세스 그룹)** 으로 만들어, 끌 때 그룹 전체에 SIGINT 를 보낸다
(ros2 launch 의 정상 종료 경로). 백그라운드 launch 가 SIGINT 를 못 받던 함정(troubleshooting 2026-11-05)과 같은
이유로, 그룹 단위 신호 + 단계적 강제 종료(SIGINT → SIGTERM → SIGKILL)를 쓴다.

전환(시작·정지)은 수십 초 걸릴 수 있어 별도 스레드에서 하고, 그동안 다른 요청은 거절한다.
다른 곳(서버 터미널)에서 이미 SLAM·Nav2 를 띄워 두었으면 시작을 거절한다 — 두 벌이 뜨면 TF·지도가 꼬인다.
"""
from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from collections import deque

MODES = ('IDLE', 'MAPPING', 'NAVIGATION')
# 이 노드들이 보이면 SLAM/Nav2 가 떠 있는 것 (우리가 띄운 게 아니면 '외부 실행')
STACK_NODES = ('slam_toolbox', 'bt_navigator', 'amcl', 'controller_server', 'planner_server')
# 우리가 끈 노드도 ROS 그래프 목록에서 사라지기까지 몇 초 걸린다 → 직접 끈 직후엔 남은 이름을 '외부 실행'으로 보지 않는다
GRAPH_LINGER_SEC = 15.0


class ModeManager:
    def __init__(self, logger, map_store, ready_fn, graph_nodes_fn, on_stop=None,
                 launch=('my_bringup', 'system.launch.py'), log_path='/tmp/web_api_mode_launch.log',
                 ready_timeout=120.0):
        self._log = logger
        self._maps = map_store
        self._ready_fn = ready_fn            # (mode, started_at) -> bool : 시작이 끝났는지 (Nav2 준비·지도 수신 등)
        self._graph_nodes = graph_nodes_fn   # () -> set[str] : 지금 ROS 그래프에 있는 노드 이름
        self._on_stop = on_stop              # 정지 전에 할 일 (진행 중인 Nav2 목표 취소)
        self._launch = launch
        self._log_path = log_path
        self._ready_timeout = ready_timeout
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._stopped_at = 0.0
        self._tail: deque[str] = deque(maxlen=60)
        self._state = {'mode': 'IDLE', 'phase': 'idle', 'map': None, 'vision': False, 'since': time.time(), 'message': ''}
        self.version = 0
        threading.Thread(target=self._watch, daemon=True).start()

    # ---------- 웹에서 부르는 것 ----------
    def snapshot(self) -> dict:
        with self._lock:
            st = dict(self._state)
        own = self._proc is not None and self._proc.poll() is None
        st['external'] = (not own) and bool(self._stack_nodes())
        st['version'] = self.version
        return st

    def log_tail(self, n: int = 40) -> list[str]:
        with self._lock:
            return list(self._tail)[-n:]

    def request(self, mode: str, map_name: str | None = None, vision: bool = True) -> dict:
        """모드 전환 요청. 즉시 접수 여부만 돌려주고 실제 전환은 백그라운드 스레드에서."""
        if mode not in MODES:
            return {'ok': False, 'reason': 'bad_request', 'message': f'모드는 {"/".join(MODES)} 중 하나'}
        with self._lock:
            if self._state['phase'] in ('starting', 'stopping'):
                return {'ok': False, 'reason': 'busy', 'message': '전환 중 — 끝난 뒤 다시 시도'}
            current = self._state['mode']
        map_yaml = None
        if mode == 'NAVIGATION':
            if not map_name:
                return {'ok': False, 'reason': 'bad_request', 'message': '주행할 지도를 고르세요'}
            try:
                map_yaml = self._maps.path(map_name, 'yaml')
            except ValueError as e:
                return {'ok': False, 'reason': 'bad_request', 'message': str(e)}
            if not os.path.isfile(map_yaml):
                return {'ok': False, 'reason': 'not_found', 'message': f'지도 {map_name} 없음'}
        own = self._proc is not None and self._proc.poll() is None
        if mode != 'IDLE' and not own and self._stack_nodes():
            return {'ok': False, 'reason': 'external',
                    'message': '다른 곳(서버 터미널 등)에서 SLAM/Nav2 가 이미 실행 중 — 그쪽을 먼저 끄세요'}
        if mode == current and mode != 'IDLE' and map_name == self._state['map']:
            return {'ok': True, 'message': '이미 그 모드'}
        threading.Thread(target=self._switch, args=(mode, map_name, map_yaml, vision), daemon=True).start()
        return {'ok': True, 'message': '전환 시작'}

    def shutdown(self):
        """web-api 종료 시: 띄운 launch 를 정상 종료."""
        self._stop_process()

    # ---------- 내부 ----------
    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)
            self.version += 1

    def _stack_nodes(self) -> set[str]:
        if time.time() - self._stopped_at < GRAPH_LINGER_SEC:
            return set()
        return {n for n in self._graph_nodes() if n in STACK_NODES}

    def _switch(self, mode, map_name, map_yaml, vision):
        try:
            if self._proc is not None:
                self._set(phase='stopping', message='정지 중…')
                if self._on_stop:
                    self._on_stop()
                self._stop_process()
            if mode == 'IDLE':
                self._set(mode='IDLE', phase='idle', map=None, vision=False, since=time.time(), message='')
                return
            args = ['rviz:=false', f'vision:={"true" if vision else "false"}']
            args += ['use_slam:=true'] if mode == 'MAPPING' else ['use_slam:=false', f'map:={map_yaml}']
            cmd = ['ros2', 'launch', *self._launch, *args]
            self._log.info(f'모드 {mode} 시작: {" ".join(cmd)}')
            started = time.time()
            self._set(mode=mode, phase='starting', map=map_name, vision=vision, since=started,
                      message='SLAM·Nav2 시작 중…' if mode == 'MAPPING' else '지도·AMCL·Nav2 시작 중…')
            with self._lock:
                self._tail.clear()
            logf = open(self._log_path, 'w')
            # start_new_session: 새 프로세스 그룹 → 끌 때 그룹 전체에 신호 (launch 와 모든 노드)
            self._proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
            self._wait_ready(mode, started)
        except Exception as e:  # noqa: BLE001
            self._set(phase='error', message=f'{type(e).__name__}: {e}')

    def _wait_ready(self, mode, started):
        while time.time() - started < self._ready_timeout:
            if self._proc is None or self._proc.poll() is not None:
                return   # _watch 가 오류로 표시
            if self._ready_fn(mode, started):
                msg = '지도를 만드는 중 — 조이스틱이나 「목표 지정」으로 돌아다니세요' if mode == 'MAPPING' \
                    else '저장 지도로 주행 — 로봇 위치는 자동으로 찾습니다 (안 되면 「위치 찾기」·후보 선택·「초기 위치」)'
                self._set(phase='running', message=msg)
                return
            time.sleep(0.5)
        self._set(phase='running', message=f'{int(self._ready_timeout)}초 안에 준비 신호가 없음 — 로그 확인')

    def _stop_process(self):
        p = self._proc
        if p is None:
            return
        if p.poll() is None:
            for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    p.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    self._log.warn(f'launch 가 {sig.name} 후 {wait}s 안에 안 끝남 → 다음 단계')
        self._proc = None
        self._stopped_at = time.time()

    def _watch(self):
        """로그 꼬리 읽기 + 띄운 launch 가 스스로 죽었는지 감시."""
        pos = 0
        while True:
            time.sleep(1.0)
            try:
                if os.path.getsize(self._log_path) < pos:   # 새 launch 가 로그를 새로 씀
                    pos = 0
                with open(self._log_path, 'r', errors='replace') as f:
                    f.seek(pos)
                    new = f.read()
                    pos = f.tell()
                if new:
                    with self._lock:
                        self._tail.extend(line for line in new.splitlines() if line.strip())
            except FileNotFoundError:
                pos = 0
            p = self._proc
            if p is not None and p.poll() is not None and self._state['phase'] in ('starting', 'running'):
                code = p.returncode
                self._proc = None
                with self._lock:
                    last = ' / '.join(list(self._tail)[-2:])
                self._set(phase='error', message=f'launch 가 종료됨 (코드 {code}) — {last[-300:]}')
