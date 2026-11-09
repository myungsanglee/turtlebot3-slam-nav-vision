"""진입점: rclpy 노드를 백그라운드 스레드에서 돌리고, 메인 스레드에서 uvicorn(FastAPI)을 돌린다.

    ros2 launch my_web_api web_api.launch.py static_dir:=/web/dist

종료 순서 (Ctrl+C · docker stop 의 SIGINT/SIGTERM):
  1) 신호 처리기가 uvicorn 에 "종료 요청"만 한다 (should_exit)
  2) uvicorn 이 새 연결을 막고, 열린 연결(끝나지 않는 MJPEG 포함)을 최대 2초 기다린 뒤 닫는다
  3) server.run() 이 정상 반환 → ROS executor·노드 정리
신호를 uvicorn 기본 처리에 맡기지 않는 이유: uvicorn 은 종료 후 받은 신호를 다시 발생시켜(raise_signal)
KeyboardInterrupt 가 이벤트 루프를 뚫고 나오는데, 그 과정에서 열린 스트림이 강제 취소되며 traceback 이
찍히고 ROS 정리 도중 프로세스가 abort 됐다 (실측). rclpy 의 SIGINT 처리기도 끈다 — 신호 주인은 하나만.
"""
import contextlib
import signal
import threading
import time

import rclpy
import uvicorn
from rclpy.executors import MultiThreadedExecutor

from .app import create_app
from .ros_node import WebApiNode


class _Server(uvicorn.Server):
    @contextlib.contextmanager
    def capture_signals(self):
        # 신호는 main() 이 직접 받는다 (위 docstring)
        yield


def main():
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()

    node = WebApiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop = threading.Event()
    app = create_app(node, node.static_dir, stop)
    # MJPEG 는 끝나지 않는 응답이라, 종료 시 무한정 기다리지 않도록 유예 시간을 둔다
    config = uvicorn.Config(app, host=node.host, port=node.port, log_level='info',
                            timeout_graceful_shutdown=2)
    server = _Server(config)

    first_signal = [None]

    def request_exit(signum, frame):
        now = time.monotonic()
        # 터미널 Ctrl+C 는 launch 와 이 프로세스에 동시에 SIGINT 를 보내고, launch 가 한 번 더 전달한다.
        # uvicorn 은 두 번째 신호를 '강제 종료'로 처리해 정리 단계를 건너뛰므로, 1초 안의 중복은 무시한다.
        # (1초 뒤에 다시 오는 신호는 사람이 한 번 더 누른 것 → 강제 종료로 전달)
        if first_signal[0] is not None and now - first_signal[0] < 1.0:
            return
        if first_signal[0] is None:
            first_signal[0] = now
            node.get_logger().info(f'신호 {signal.Signals(signum).name} — 웹 서버 종료 요청')
        stop.set()                          # 열린 MJPEG·WebSocket 이 스스로 끝나게
        server.handle_exit(signum, frame)   # 첫 번째: 정상 종료 / 두 번째: 강제 종료

    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)

    try:
        server.run()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
