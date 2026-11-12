#!/usr/bin/env python3
"""/scan 감시 — 라이다 드라이버(hlds_laser_publisher)가 살아 있는데 발행만 멈추는 증상을 기록하고 복구한다.

배경 (docs/troubleshooting.md 2026-11-12 "라이다 /scan 발행 정지"):
  드라이버 프로세스는 살아서 시리얼(/dev/ttyUSB0)을 정상 속도(≈12.6KB/s)로 읽는데 /scan 이 수십 분째 0 개.
  LDS 프레임·USB·전원은 정상. 진단 중 다른 프로세스가 시리얼 포트를 1초 읽자 즉시 5Hz 로 돌아왔다.
  내부 원인은 아직 미확정 (sudo 없이는 strace 를 붙일 수 없음) → 다음 발생 때 증거를 남기는 것이 1차 목적.

동작:
  1) Pi 안에서 /scan 을 구독해 마지막 수신 시각을 본다 (zenoh·서버와 무관하게 '로봇 쪽' 상태만 판정)
  2) 드라이버가 살아 있고, 한 번이라도 발행한 뒤, STALL_SEC 이상 끊기면 '사고'로 보고
     - 진단 기록: 드라이버 시리얼 읽기량(2초 간격 rchar), 스레드 상태, 전압 플래그, bringup 로그 꼬리
     - 복구 시도: 오늘 효과가 있었던 '시리얼 포트 1초 같이 읽기' (드라이버·bringup 재시작은 하지 않음 —
       bringup 재시작은 자이로 영점·odom 초기화·카메라 재열거를 동반하므로 사람이 판단)
     - 결과(복구 여부·걸린 시간)를 기록. 안 되면 RETRY_SEC 마다 최대 MAX_NUDGES 번 다시 시도
  기록: ~/.ros/scan_watchdog/incidents.log (사고마다 한 덩어리) + journal (systemctl --user 로그)

실행: systemd 사용자 서비스 robot/systemd/scan-watchdog.service (docs/pi_setup.md 9단계)
"""
import fcntl
import os
import subprocess
import termios
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DRIVER = 'hlds_laser_publisher'
PORT = '/dev/ttyUSB0'
STALL_SEC = 10.0      # 5Hz 라이다가 이만큼 끊기면 사고
NUDGE_READ_SEC = 1.0  # 시리얼 같이 읽기 시간
RECOVER_WAIT_SEC = 10.0
RETRY_SEC = 60.0
MAX_NUDGES = 3
LOG_DIR = os.path.expanduser('~/.ros/scan_watchdog')


def driver_pid():
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/cmdline', 'rb') as f:
                argv0 = f.read().split(b'\0')[0]
        except OSError:
            continue
        if argv0.endswith(DRIVER.encode()):
            return int(d)
    return None


def read_file(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError as e:
        return f'({e.strerror})'


def rchar(pid):
    for line in read_file(f'/proc/{pid}/io').splitlines():
        if line.startswith('rchar:'):
            return int(line.split()[1])
    return None


def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return f'({type(e).__name__}: {e})'


class ScanWatchdog(Node):
    def __init__(self):
        super().__init__('scan_watchdog')
        self.last_scan = None          # 마지막 수신 (monotonic)
        self.scan_count = 0
        self.pid = None                # 지금 감시 중인 드라이버 pid (바뀌면 '첫 발행'부터 다시 기다림)
        self.busy = False
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.check)
        os.makedirs(LOG_DIR, exist_ok=True)
        self.get_logger().info(f'/scan 감시 시작 (정지 판정 {STALL_SEC:.0f}s, 기록 {LOG_DIR}/incidents.log)')

    def on_scan(self, _msg):
        self.last_scan = time.monotonic()
        self.scan_count += 1

    def check(self):
        if self.busy:
            return
        pid = driver_pid()
        if pid != self.pid:
            if pid:
                self.get_logger().info(f'드라이버 감지 pid={pid} — 첫 발행을 기다림')
            self.pid, self.last_scan = pid, None
            return
        if pid is None or self.last_scan is None:
            return   # 드라이버 없음(bringup 꺼짐) 또는 아직 한 번도 발행 안 함(시작 중) → 판정 안 함
        age = time.monotonic() - self.last_scan
        if age >= STALL_SEC:
            self.busy = True
            threading.Thread(target=self.incident, args=(pid, age), daemon=True).start()

    # ---------- 사고 처리 (별도 스레드: 구독 콜백은 계속 돌아야 복구를 알 수 있다) ----------
    def incident(self, pid, age):
        lines = []

        def log(s):
            lines.append(f'{time.strftime("%F %T")} {s}')
            self.get_logger().warn(s)

        try:
            log(f'/scan 정지 감지: {age:.1f}s 째 수신 없음, 드라이버 pid={pid} 살아 있음')
            log(f'  uptime: {read_file("/proc/uptime")}  load: {read_file("/proc/loadavg")}')
            log(f'  드라이버 실행 시간: {run(["ps", "-o", "etime=", "-p", str(pid)])}')
            r0 = rchar(pid)
            time.sleep(2.0)
            r1 = rchar(pid)
            if r0 is not None and r1 is not None:
                log(f'  드라이버 읽기량: {(r1 - r0) / 2:.0f} B/s (정상 LDS-01 ≈ 12600 B/s)')
            for t in sorted(os.listdir(f'/proc/{pid}/task')):
                stat = read_file(f'/proc/{pid}/task/{t}/stat').rsplit(')', 1)[-1].split()
                log(f'  thread {t} {read_file(f"/proc/{pid}/task/{t}/comm")} '
                    f'state={stat[0] if stat else "?"} wchan={read_file(f"/proc/{pid}/task/{t}/wchan")}')
            log(f'  전압 플래그: {run(["vcgencmd", "get_throttled"])}')
            log(f'  시리얼: {run(["ls", "-l", PORT])}')
            tail = run(['journalctl', '--user', '-u', 'tb3-bringup', '-n', '300', '--no-pager', '-o', 'cat'])
            tail = [ln for ln in tail.splitlines() if 'fps |' not in ln][-15:]   # 카메라 10초 주기 통계 줄은 제외
            log('  bringup 로그 꼬리 (카메라 fps 줄 제외):\n    ' + '\n    '.join(tail))

            for n in range(1, MAX_NUDGES + 1):
                before = self.scan_count
                queued, got = self.nudge()
                # 드라이버가 시리얼을 실제로 읽고 있으면 큐는 거의 비어 있고 같이 읽기는 수백 B 만 가져간다 (정상 시 실측 274 B).
                # 큐가 수 KB 이고 같이 읽기가 ≈12 KB 면 드라이버가 시리얼을 읽지 않고 있던 것
                log(f'  복구 시도 {n}/{MAX_NUDGES}: 시리얼 입력 큐 {queued} B 쌓여 있음, '
                    f'{NUDGE_READ_SEC:.0f}s 같이 읽어 {got} B 가져옴 (드라이버가 읽는 중이면 수백 B)')
                t0 = time.monotonic()
                while time.monotonic() - t0 < RECOVER_WAIT_SEC and self.scan_count == before:
                    time.sleep(0.2)
                if self.scan_count > before:
                    log(f'  → 복구됨 ({time.monotonic() - t0:.1f}s 뒤 /scan 재개)')
                    break
                if driver_pid() != pid:
                    log('  → 드라이버가 재시작/종료됨 — 감시 재개')
                    break
                log(f'  → 복구 안 됨 ({RECOVER_WAIT_SEC:.0f}s 대기)')
                if n < MAX_NUDGES:
                    time.sleep(RETRY_SEC)
            else:
                log('  → 자동 복구 실패: 수동 조치 필요 (systemctl --user restart tb3-bringup, '
                    '재시작 후 5초간 로봇 정지 — 자이로 영점)')
        except Exception as e:  # noqa: BLE001
            log(f'  감시 오류: {type(e).__name__}: {e}')
        finally:
            with open(os.path.join(LOG_DIR, 'incidents.log'), 'a') as f:
                f.write('\n'.join(lines) + '\n\n')
            self.busy = False

    def nudge(self):
        """시리얼 포트를 잠깐 같이 읽는다 (읽은 바이트는 버림). O_NOCTTY: 제어 터미널로 잡지 않게.
        돌려주는 값: (열었을 때 입력 큐에 쌓여 있던 바이트 — FIONREAD, 같이 읽어 가져온 바이트)"""
        total = 0
        fd = os.open(PORT, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
        try:
            try:
                queued = int.from_bytes(fcntl.ioctl(fd, termios.FIONREAD, b'\0' * 4), 'little')
            except OSError:
                queued = -1
            t_end = time.monotonic() + NUDGE_READ_SEC
            while time.monotonic() < t_end:
                try:
                    total += len(os.read(fd, 4096))
                except BlockingIOError:
                    time.sleep(0.01)
        finally:
            os.close(fd)
        return queued, total


def main():
    rclpy.init()
    node = ScanWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
