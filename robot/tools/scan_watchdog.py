#!/usr/bin/env python3
"""/scan·전원 감시 — 라이다가 끊기는 두 가지 경우를 기록하고, 고칠 수 있는 경우는 복구한다.

배경 (docs/troubleshooting.md 2026-11-12 "라이다 /scan 발행 정지", "저전압 → USB 전체 재연결"):
  (A) 드라이버 정지: hlds_laser_publisher 가 살아서 시리얼을 정상 속도로 읽는데 /scan 이 0 개.
      다른 프로세스가 시리얼을 1초 같이 읽자 즉시 복구. 내부 원인 미확정 → 증거 기록 + 같이 읽기 복구.
  (B) 전원 저하: Pi 5V 저전압이 몇 시간 이어지다 USB 포트 전체가 리셋 → 라이다·OpenCR·카메라가 끊겼다
      다른 이름(ttyUSB1·ttyACM1)으로 재연결. 라이다 드라이버는 쓰기 오류로 종료, OpenCR 노드는 멈춤.
      첫 버전은 (A) 만 봐서 (B) 를 기록하지 못했다 → 전원 기록과 사건 기록을 추가.

기록 (~/.ros/scan_watchdog/):
  power-YYYY-MM-DD.log  10초마다 한 줄: 배터리 전압, Pi 저전압(지금/구간 내 횟수), throttled, CPU 온도, /scan Hz
                        → "몇 시에, 배터리가 몇 V 일 때 저전압이 시작됐나" 를 사후에 본다 (KEEP_DAYS 일 보관)
  events.log            사건 한 줄씩: 저전압 시작/해소, USB 재연결·과전류(커널 로그), 시리얼 장치 변경,
                        드라이버 종료, OpenCR 배터리 수신 끊김/재개, /scan 정지·복구
  incidents.log         사건마다 진단 덩어리 ((A) 정지 진단 + 복구 시도, 드라이버 비정상 종료 때 로그 꼬리)

복구는 (A) 의 '시리얼 같이 읽기' 만 자동으로 한다. bringup 재시작은 자이로 영점·odom 초기화·카메라 재열거를
동반하므로 사람이 판단 (B 는 bringup 이 by-id 포트를 쓰므로 `systemctl --user restart tb3-bringup` 으로 복구).

실행: systemd 사용자 서비스 robot/systemd/scan-watchdog.service (docs/pi_setup.md 9-1)
"""
import fcntl
import glob
import os
import subprocess
import termios
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan

DRIVER = 'hlds_laser_publisher'
LDS_GLOB = '/dev/serial/by-id/*CP210*'   # LDS-01 의 USB-시리얼(CP2102). 없으면 PORT
PORT = '/dev/ttyUSB0'
STALL_SEC = 10.0      # 5Hz 라이다가 이만큼 끊기면 사고
NUDGE_READ_SEC = 1.0  # 시리얼 같이 읽기 시간
RECOVER_WAIT_SEC = 10.0
RETRY_SEC = 60.0
MAX_NUDGES = 3

POWER_SEC = 10.0      # 전원 기록 주기
BATT_STALE_SEC = 10.0  # turtlebot3_node 는 /battery_state 를 수 Hz 로 낸다 → 이만큼 없으면 OpenCR 통신 끊김
UV_QUIET_SEC = 60.0   # 저전압 경고가 이만큼 없으면 '해소'로 보고 다음 경고를 '새 시작'으로 기록
KEEP_DAYS = 14
LOG_DIR = os.path.expanduser(os.environ.get('SCAN_WATCHDOG_DIR', '~/.ros/scan_watchdog'))   # 시험 실행 때만 바꿈
UV_ALARM = '/sys/class/hwmon/hwmon1/in0_lcrit_alarm'   # rpi_volt: 1 = 지금 저전압 (경로는 init 에서 이름으로 찾음)
TEMP = '/sys/class/thermal/thermal_zone0/temp'

# 커널 로그에서 사건으로 남길 줄 (저전압 줄은 수천 번 반복되므로 따로 세기만 한다)
KLOG_KEYS = ('over-current', 'USB disconnect', 'cp210x', 'cdc_acm', 'ttyUSB', 'ttyACM',
             'new full-speed USB', 'new high-speed USB', 'new SuperSpeed USB', 'reset ')


def lds_port():
    found = sorted(glob.glob(LDS_GLOB))
    return found[0] if found else PORT


def find_uv_alarm():
    for h in glob.glob('/sys/class/hwmon/hwmon*'):
        if read_file(f'{h}/name') == 'rpi_volt':
            return f'{h}/in0_lcrit_alarm'
    return UV_ALARM


def serial_devices():
    return sorted(os.path.basename(p) for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))


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


def throttled():
    """Pi 전원 플래그 (0x50000 = 부팅 후 저전압·스로틀이 있었음, 0x50005 = 지금도). 'throttled=' 접두어는 뺀다."""
    return run(['vcgencmd', 'get_throttled']).replace('throttled=', '')


def bringup_tail(n=15, only=None):
    tail = run(['journalctl', '--user', '-u', 'tb3-bringup', '-n', '400', '--no-pager', '-o', 'cat'])
    lines = [ln for ln in tail.splitlines() if 'fps |' not in ln]   # 카메라 10초 주기 통계 줄은 제외
    if only:
        lines = [ln for ln in lines if any(k in ln for k in only)]
    return lines[-n:]


class ScanWatchdog(Node):
    def __init__(self):
        super().__init__('scan_watchdog')
        self.last_scan = None          # 마지막 수신 (monotonic)
        self.scan_count = 0
        self.pid = None                # 지금 감시 중인 드라이버 pid (바뀌면 '첫 발행'부터 다시 기다림)
        self.busy = False
        self.file_lock = threading.Lock()
        # 전원
        self.uv_alarm = find_uv_alarm()
        self.batt = None               # (전압, monotonic)
        self.batt_lost = False
        self.uv_events = 0             # 이번 POWER_SEC 구간의 커널 저전압 경고 수
        self.uv_last = None            # 마지막 저전압 경고 (monotonic)
        self.uv_episode = 0            # 이번 저전압 '구간'의 경고 수
        self.scan_mark = 0
        self.devices = serial_devices()

        os.makedirs(LOG_DIR, exist_ok=True)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery_state', self.on_batt, qos_profile_sensor_data)
        self.create_timer(1.0, self.check)
        self.create_timer(POWER_SEC, self.power_tick)
        threading.Thread(target=self.follow_kernel, daemon=True).start()
        self.event(f'감시 시작 (시리얼 {" ".join(self.devices) or "없음"}, 라이다 포트 {lds_port()}, '
                   f'throttled {throttled()})', warn=False)

    # ---------- 기록 ----------
    def write(self, name, text):
        with self.file_lock, open(os.path.join(LOG_DIR, name), 'a') as f:
            f.write(text)

    def event(self, s, warn=True):
        self.write('events.log', f'{time.strftime("%F %T")} {s}\n')   # 파일 먼저 — 기록이 1차 목적
        # rclpy 는 '같은 호출 위치에서 등급을 바꾸면' 예외를 낸다 → 등급별로 호출 위치를 나눈다
        if warn:
            self.get_logger().warn(s)
        else:
            self.get_logger().info(s)

    def batt_text(self):
        if self.batt is None:
            return '배터리 ?'
        v, t = self.batt
        age = time.monotonic() - t
        return f'배터리 {v:.2f}V' + (f' ({age:.0f}s 전 값)' if age > BATT_STALE_SEC else '')

    # ---------- 수신 ----------
    def on_scan(self, _msg):
        self.last_scan = time.monotonic()
        self.scan_count += 1

    def on_batt(self, msg):
        self.batt = (msg.voltage, time.monotonic())
        if self.batt_lost:
            self.batt_lost = False
            self.event(f'OpenCR 배터리 상태 수신 재개 ({msg.voltage:.2f}V)', warn=False)

    def follow_kernel(self):
        """커널 로그를 실시간으로 읽는다 (michael 은 adm 그룹 → sudo 불필요)."""
        while rclpy.ok():
            p = subprocess.Popen(['journalctl', '-k', '-f', '-n', '0', '-o', 'cat', '--no-pager'],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            try:
                for line in p.stdout:
                    try:
                        self.on_kernel(line.rstrip())
                    except Exception as e:  # noqa: BLE001 — 한 줄 처리 실패로 읽기를 멈추지 않는다
                        self.get_logger().error(f'커널 로그 줄 처리 실패: {type(e).__name__}: {e}')
            finally:
                p.kill()   # 다시 시작할 때 이전 journalctl 이 남지 않게
                p.wait()
            time.sleep(5.0)

    def on_kernel(self, line):
        if 'Undervoltage' in line:
            now = time.monotonic()
            self.uv_events += 1
            if self.uv_last is None or now - self.uv_last > UV_QUIET_SEC:
                self.uv_episode = 0
                self.event(f'Pi 저전압 시작 — {self.batt_text()}, throttled {throttled()}')
            self.uv_last = now
            self.uv_episode += 1
        elif 'Voltage normalised' in line:
            pass   # 저전압 경고와 짝으로 반복 — 횟수는 Undervoltage 로만 센다
        elif any(k in line for k in KLOG_KEYS):
            self.event(f'커널: {line}')

    # ---------- 1초 판정 ----------
    def check(self):
        devs = serial_devices()
        if devs != self.devices:
            self.event(f'시리얼 장치 변경: {" ".join(self.devices) or "없음"} → {" ".join(devs) or "없음"}')
            self.devices = devs
        if self.busy:
            return
        pid = driver_pid()
        if pid != self.pid:
            if self.pid is not None:
                self.driver_gone(self.pid, pid)
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

    def driver_gone(self, old, new):
        state = run(['systemctl', '--user', 'is-active', 'tb3-bringup'])
        if state != 'active':
            self.event(f'라이다 드라이버 종료 (pid {old}) — bringup {state} (정상 정지)', warn=False)
            return
        self.event(f'라이다 드라이버 비정상 종료 (pid {old}{f" → 새 pid {new}" if new else ""}), bringup 은 계속 실행 중')
        lines = [f'{time.strftime("%F %T")} 라이다 드라이버 비정상 종료 (pid {old})',
                 f'  {self.batt_text()}, throttled {throttled()}, '
                 f'시리얼 {" ".join(serial_devices()) or "없음"}, 라이다 포트 {lds_port()}',
                 '  bringup 로그 (라이다·OpenCR 관련):']
        lines += ['    ' + ln for ln in bringup_tail(12, only=('hlds', 'turtlebot3_ros', 'terminate', 'what()',
                                                                'Failed', 'ERROR', 'died'))]
        lines.append('  커널 로그 (최근 3분, USB·전원):')
        klog = run(['journalctl', '-k', '--since', '-3min', '--no-pager', '-o', 'short-iso'])
        klog = [ln for ln in klog.splitlines() if 'Undervoltage' in ln or any(k in ln for k in KLOG_KEYS)]
        lines += ['    ' + ln for ln in klog[-20:]]
        lines.append('  → 복구: 시리얼 이름이 바뀌었으면 by-id 포트로 `systemctl --user restart tb3-bringup` '
                     '(재시작 후 5초 로봇 정지 — 자이로 영점). 전원 원인이면 배터리·UBEC 먼저 확인')
        self.write('incidents.log', '\n'.join(lines) + '\n\n')

    # ---------- 10초 전원 기록 ----------
    def power_tick(self):
        now = time.monotonic()
        if self.batt and not self.batt_lost and now - self.batt[1] > BATT_STALE_SEC and self.pid:
            self.batt_lost = True
            self.event(f'OpenCR 배터리 상태 수신 끊김 (마지막 {self.batt[0]:.2f}V) — OpenCR 통신 확인')
        if self.uv_last is not None and self.uv_episode and now - self.uv_last > UV_QUIET_SEC:
            self.event(f'Pi 저전압 해소 ({UV_QUIET_SEC:.0f}초째 경고 없음, 이번 구간 경고 {self.uv_episode}회) — '
                       f'{self.batt_text()}', warn=False)
            self.uv_episode = 0
        uv, self.uv_events = self.uv_events, 0
        hz = (self.scan_count - self.scan_mark) / POWER_SEC
        self.scan_mark = self.scan_count
        batt = '-' if self.batt is None or now - self.batt[1] > BATT_STALE_SEC else f'{self.batt[0]:.2f}'
        try:
            temp = f'{int(read_file(TEMP)) / 1000:.1f}'
        except ValueError:
            temp = '-'
        flags = throttled()
        day = time.strftime('%F')
        path = os.path.join(LOG_DIR, f'power-{day}.log')
        if not os.path.exists(path):
            self.prune()
            self.write(f'power-{day}.log', '# 시각 batt=배터리V(- 는 수신 없음) uv_now=Pi 저전압 지금 '
                                           'uv=구간 내 저전압 경고 수 throttled temp=CPU°C scan_hz\n')
        self.write(f'power-{day}.log',
                   f'{time.strftime("%T")} batt={batt} uv_now={read_file(self.uv_alarm)} uv={uv} '
                   f'throttled={flags} temp={temp} scan_hz={hz:.1f}\n')

    def prune(self):
        cutoff = time.time() - KEEP_DAYS * 86400
        for p in glob.glob(os.path.join(LOG_DIR, 'power-*.log')):
            if os.path.getmtime(p) < cutoff:
                os.remove(p)

    # ---------- (A) 정지 사고 처리 (별도 스레드: 구독 콜백은 계속 돌아야 복구를 알 수 있다) ----------
    def incident(self, pid, age):
        lines = []

        def log(s):
            lines.append(f'{time.strftime("%F %T")} {s}')
            self.get_logger().warn(s)

        self.event(f'/scan 정지 감지 ({age:.0f}s, 드라이버 pid={pid} 살아 있음) — incidents.log 에 진단')
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
            log(f'  전압: throttled {throttled()}, {self.batt_text()}')
            log(f'  시리얼: {run(["ls", "-l", lds_port()])}')
            log('  bringup 로그 꼬리 (카메라 fps 줄 제외):\n    ' + '\n    '.join(bringup_tail()))

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
                    self.event('/scan 복구됨 (시리얼 같이 읽기)', warn=False)
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
                self.event('/scan 자동 복구 실패 — 수동 조치 필요')
        except Exception as e:  # noqa: BLE001
            log(f'  감시 오류: {type(e).__name__}: {e}')
        finally:
            self.write('incidents.log', '\n'.join(lines) + '\n\n')
            self.busy = False

    def nudge(self):
        """시리얼 포트를 잠깐 같이 읽는다 (읽은 바이트는 버림). O_NOCTTY: 제어 터미널로 잡지 않게.
        돌려주는 값: (열었을 때 입력 큐에 쌓여 있던 바이트 — FIONREAD, 같이 읽어 가져온 바이트)"""
        total = 0
        fd = os.open(lds_port(), os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
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
