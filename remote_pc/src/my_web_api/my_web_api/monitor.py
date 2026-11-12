"""토픽 수신 상태와 최신 영상 프레임을 담는 순수 파이썬 객체들 (ROS 의존 없음 → 단위 테스트 가능).

rclpy 콜백 스레드가 record()/put() 으로 쓰고, FastAPI(asyncio) 쪽이 읽는다. 둘이 다른 스레드이므로
모든 공유 상태는 Lock 으로 보호한다.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class TopicSpec:
    """감시할 토픽 하나의 설정 (config/web_api.yaml 의 topics 항목)."""
    key: str              # API 에서 쓰는 이름 (예: 'odom')
    topic: str            # ROS 토픽 이름
    label: str            # 화면 표시용 한글 이름
    expected_hz: float    # 정상일 때 기대 주기 — 화면에서 비교용
    optional: bool = False  # True 면 안 들어와도 '오류' 가 아니라 '꺼짐' (예: Vision 결과)


class TopicMonitor:
    """토픽 하나의 수신 주기(Hz)와 마지막 수신 후 경과 시간을 추적한다.

    주기는 최근 window_sec 동안 받은 메시지 시각들로 계산한다:
        rate = (개수 - 1) / (마지막 시각 - 첫 시각)
    메시지가 끊기면 window 안의 시각이 빠져나가 rate 가 0 으로 떨어진다.

    마지막 수신 시각은 window 와 **별도로** 보관한다 — 끊긴 뒤에도 "언제 마지막으로 받았는지"를 보여줘야 하므로.
    (처음 구현은 window 의 시각 목록에서 마지막 값을 꺼내 써서, 끊기고 3초가 지나면 마지막 수신 시각까지 사라졌다.)
    """

    def __init__(self, spec: TopicSpec, window_sec: float = 3.0, stale_sec: float = 2.0,
                 clock=time.monotonic, wall_clock=time.time):
        self.spec = spec
        self._window = window_sec
        self._stale = stale_sec
        self._clock = clock
        self._wall_clock = wall_clock
        self._stamps: deque[float] = deque()
        self._last: float | None = None        # 마지막 수신 (monotonic — 경과 시간 계산용)
        self._last_wall: float | None = None   # 마지막 수신 (실제 시각 — 화면에 '몇 시 몇 분' 표시용)
        self._lock = threading.Lock()

    def record(self) -> None:
        now = self._clock()
        with self._lock:
            self._stamps.append(now)
            self._last = now
            self._last_wall = self._wall_clock()
            self._trim(now)

    def _trim(self, now: float) -> None:
        while self._stamps and now - self._stamps[0] > self._window:
            self._stamps.popleft()

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            self._trim(now)
            stamps = list(self._stamps)
            last, last_wall = self._last, self._last_wall
        age = (now - last) if last is not None else None
        rate = 0.0
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        receiving = age is not None and age < self._stale
        if receiving:
            state = 'ok'
        else:
            state = 'off' if self.spec.optional else 'missing'
        return {
            'key': self.spec.key, 'topic': self.spec.topic, 'label': self.spec.label,
            'expected_hz': self.spec.expected_hz, 'optional': self.spec.optional,
            'rate': round(rate if receiving else 0.0, 1),
            'age': None if age is None else round(age, 2),          # 마지막 수신 후 경과 초 (끊겨도 유지)
            'last_received': None if last_wall is None else round(last_wall, 3),   # epoch 초
            'state': state,
        }


class FrameStore:
    """최신 JPEG 프레임 하나와 버전 번호. MJPEG 스트림이 버전이 바뀔 때만 새 프레임을 보낸다."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data: bytes | None = None
        self._version = 0

    def put(self, data: bytes) -> None:
        with self._lock:
            self._data = data
            self._version += 1

    def get(self) -> tuple[int, bytes | None]:
        with self._lock:
            return self._version, self._data


def battery_level(voltage: float, cells: int, warn_cell_v: float, critical_cell_v: float) -> str:
    """셀당 전압으로 배터리 상태를 나눈다. LiPo 는 잔량 % 가 전압에 비선형이라 셀 전압이 더 믿을 만하다
    (docs/my_web_api.md). 3S 기준: 셀당 3.5V(10.5V) 아래면 충전 권장, 3.3V(9.9V) 아래면 즉시 중단."""
    per_cell = voltage / cells
    if per_cell < critical_cell_v:
        return 'critical'
    if per_cell < warn_cell_v:
        return 'warn'
    return 'ok'
