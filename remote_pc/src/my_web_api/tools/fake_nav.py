"""시험용 가짜 Nav2 — navigate_to_pose 액션만 흉내낸다 (cmd_vel 을 내지 않으므로 로봇이 움직이지 않는다).

웹 콘솔의 목표 전송·진행 표시·도착·취소·거절 흐름을 로봇 없이(또는 로봇이 책상 위에 있을 때) 확인할 때 쓴다.
  # 진짜 Nav2 가 떠 있지 않은 상태에서 (같은 액션 이름)
  python3 tools/fake_nav.py          # 2초 동안 남은 거리를 줄이며 feedback → 성공. x > 100 인 목표는 거절
  python3 tools/nav_api_check.py     # 다른 셸: REST 로 목표·취소를 보내고 WebSocket 으로 상태 변화 기록
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class Fake(Node):
    def __init__(self):
        super().__init__('fake_nav')
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute,
                     goal_callback=lambda g: GoalResponse.REJECT if g.pose.pose.position.x > 100 else GoalResponse.ACCEPT,
                     cancel_callback=lambda h: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
    def execute(self, h):
        fb = NavigateToPose.Feedback()
        for i in range(20):
            if h.is_cancel_requested:
                h.canceled(); return NavigateToPose.Result()
            fb.distance_remaining = 2.0 - i * 0.1; fb.navigation_time.sec = i // 10; fb.number_of_recoveries = 0
            h.publish_feedback(fb); time.sleep(0.1)
        h.succeed(); return NavigateToPose.Result()

rclpy.init(); n = Fake(); ex = MultiThreadedExecutor(); ex.add_node(n); ex.spin()
