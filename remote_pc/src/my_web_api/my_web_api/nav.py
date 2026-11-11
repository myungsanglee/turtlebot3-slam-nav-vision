"""Nav2 목표 전송·취소·진행 상태 (NavigateToPose 액션) + AMCL 초기 위치.

스레드 규칙: rclpy 호출(액션 전송·취소·publish)은 **ROS executor 스레드에서만** 한다.
FastAPI(asyncio) 쪽은 submit() 으로 요청을 큐에 넣고 concurrent.futures.Future 로 결과를 기다린다.
ROS 타이머(20Hz)가 큐를 비우며 실제 호출을 한다 → rclpy 객체를 두 스레드가 동시에 만지지 않는다.

상태(state):
  idle → sending → active → succeeded | aborted | canceled
                 ↘ rejected (Nav2 가 거절) / error (Nav2 미실행 등)
"""
from __future__ import annotations

import math
import queue
import threading
import time
from concurrent.futures import Future

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from .geometry import quaternion_from_yaw

RESULT_STATE = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


class NavController:
    def __init__(self, node, action_name: str, initialpose_topic: str, frame: str):
        self._node = node
        self._frame = frame
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, initialpose_topic, 10)
        self._queue: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._goal_handle = None
        self._goal_seq = 0          # 오래된 목표의 늦은 콜백이 새 목표 상태를 덮어쓰지 않도록
        self._state = {'state': 'idle', 'goal': None, 'feedback': None, 'message': ''}
        self.version = 0
        node.create_timer(0.05, self._drain)

    # ---------- FastAPI 쪽에서 부르는 것 ----------
    def submit(self, kind: str, **kw) -> Future:
        fut: Future = Future()
        self._queue.put((kind, kw, fut))
        return fut

    def server_ready(self) -> bool:
        return self._client.server_is_ready()

    def snapshot(self) -> dict:
        with self._lock:
            return {**self._state, 'version': self.version}

    # ---------- 이하 ROS 스레드 ----------
    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)
            self.version += 1

    def _drain(self):
        while True:
            try:
                kind, kw, fut = self._queue.get_nowait()
            except queue.Empty:
                return
            try:
                getattr(self, f'_do_{kind}')(fut, **kw)
            except Exception as e:  # noqa: BLE001 — 웹 요청에 오류를 돌려주고 노드는 계속
                fut.set_result({'ok': False, 'reason': 'error', 'message': f'{type(e).__name__}: {e}'})

    def _do_goal(self, fut: Future, x: float, y: float, yaw: float):
        if not self._client.server_is_ready():
            msg = 'Nav2 가 실행 중이 아님 (navigate_to_pose 액션 서버 없음)'
            self._set(state='error', goal={'x': x, 'y': y, 'yaw': yaw}, feedback=None, message=msg)
            fut.set_result({'ok': False, 'reason': 'unavailable', 'message': msg})
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        q = quaternion_from_yaw(float(yaw))
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        self._goal_seq += 1
        seq = self._goal_seq
        self._set(state='sending', goal={'x': x, 'y': y, 'yaw': yaw}, feedback=None, message='')
        send = self._client.send_goal_async(goal, feedback_callback=lambda fb: self._on_feedback(seq, fb))
        send.add_done_callback(lambda f: self._on_goal_response(seq, f, fut))

    def _on_goal_response(self, seq, f, fut: Future):
        handle = f.result()
        if seq != self._goal_seq:
            return
        if not handle.accepted:
            self._set(state='rejected', message='Nav2 가 목표를 거절함 (지도 밖·장애물 위·미지 영역일 수 있음)')
            fut.set_result({'ok': False, 'reason': 'rejected', 'message': self._state['message']})
            return
        self._goal_handle = handle
        self._set(state='active', message='')
        fut.set_result({'ok': True, 'message': '주행 시작'})
        handle.get_result_async().add_done_callback(lambda r: self._on_result(seq, r))

    def _on_feedback(self, seq, fb_msg):
        if seq != self._goal_seq:
            return
        fb = fb_msg.feedback
        self._set(feedback={
            'distance_remaining': round(float(fb.distance_remaining), 2),
            'navigation_time': round(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9, 1),
            'estimated_time_remaining': round(fb.estimated_time_remaining.sec
                                              + fb.estimated_time_remaining.nanosec * 1e-9, 1),
            'recoveries': int(fb.number_of_recoveries),
        })

    def _on_result(self, seq, r):
        if seq != self._goal_seq:
            return
        status = r.result().status
        state = RESULT_STATE.get(status, 'aborted')
        msg = {'succeeded': '목표 도착', 'canceled': '취소됨',
               'aborted': '주행 실패 (경로 없음·복구 동작 한도 초과 등 — Nav2 로그 확인)'}[state]
        self._goal_handle = None
        self._set(state=state, message=msg)

    def _do_cancel(self, fut: Future):
        if self._goal_handle is None:
            fut.set_result({'ok': False, 'reason': 'no_goal', 'message': '진행 중인 목표 없음'})
            return
        self._goal_handle.cancel_goal_async()
        fut.set_result({'ok': True, 'message': '취소 요청'})

    def _do_initial_pose(self, fut: Future, x: float, y: float, yaw: float):
        """AMCL 초기 위치 (저장 지도 주행 모드). RViz 의 '2D Pose Estimate' 와 같은 메시지."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = quaternion_from_yaw(float(yaw))
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25                 # x, y 표준편차 0.5 m
        cov[35] = (math.pi / 12) ** 2          # yaw 표준편차 15°
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        fut.set_result({'ok': True, 'message': '초기 위치 전송', 'time': time.time()})
