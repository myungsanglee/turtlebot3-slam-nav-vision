#!/bin/bash
# =============================================================================
# robot_power.sh — 로봇 전원 상태 제어 (웹 콘솔의 대기·켜기·재시작·재부팅·끄기 버튼)
#
# 누가 부르나: 서버 web-api 가 전용 SSH 키로 접속하면, Pi 의 ~/.ssh/authorized_keys 의
#   command="…/robot_power.sh" 강제 설정 때문에 **이 스크립트만** 실행된다 (요청한 명령은 SSH_ORIGINAL_COMMAND 로 들어옴).
#   그래서 그 키로는 셸을 얻을 수 없고 아래 동작만 할 수 있다 (docs/pi_setup.md 9-2).
#
#   status    {"bringup": "active|inactive|activating|deactivating|failed", "uptime": 초, "throttled": "0x…"}
#   standby   bringup 정지 → 라이다 모터 정지('e' 명령), 카메라·모터 제어 정지. Pi 와 네트워크는 켜진 채
#   wake      bringup 시작 (시작 후 5초 동안 자이로 영점 — 로봇을 건드리지 말 것)
#   restart   bringup 재시작
#   reboot    Pi 재부팅            ┐ sudo 규칙 필요: robot/config/sudoers-robot-power
#   poweroff  Pi 끄기 (SD 카드 보호) ┘ 끈 뒤엔 네트워크가 없어 웹에서 켤 수 없다 → 배터리 재연결로 켬
#   boot      부팅 시 대기 상태 만들기 (robot-standby.service) — 라이다 모터 정지 명령만
#   gyro_reset  자이로 영점 재보정 + odom 0 초기화 — turtlebot3_node 의 /reset 서비스 (bringup 실행 중일 때만).
#             OpenCR 이 5초 동안 영점을 잡는다 → 그동안 로봇이 완전히 멈춰 있어야 한다 (서버가 정지 확인 후 부름).
#             정지 중인데 방향이 저절로 도는 드리프트 대응 (troubleshooting 2026-11-12, 11-14)
#
# 라이다(LDS-01): 시리얼로 'b' 를 받으면 회전·송신 시작, 'e' 를 받으면 정지. 드라이버(hlds_laser_publisher)도
#   시작 때 'b', 종료 때 'e' 를 보낸다 — bringup 정지 후 시리얼 수신 0 B 로 실측 확인 (2026-11-12).
# =============================================================================
set -u
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}   # systemctl --user 에 필요
SVC=tb3-bringup
cmd=${SSH_ORIGINAL_COMMAND:-${1:-status}}

lds_port() { ls /dev/serial/by-id/*CP210* 2>/dev/null | head -1; }

lds_stop() {   # bringup 이 꺼져 있을 때만 (드라이버가 포트를 쓰는 중이면 끼어들지 않음)
    local p; p=$(lds_port)
    [ -n "$p" ] || return 0
    [ "$(systemctl --user is-active $SVC)" = active ] && return 0
    stty -F "$p" 230400 raw -echo 2>/dev/null && printf 'e' > "$p"
}

status() {
    printf '{"bringup": "%s", "uptime": %s, "throttled": "%s"}\n' \
        "$(systemctl --user is-active $SVC)" "$(cut -d' ' -f1 /proc/uptime)" \
        "$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
}

gyro_reset() {
    if [ "$(systemctl --user is-active $SVC)" != active ]; then
        echo '{"error": "bringup 이 실행 중이 아님"}'; status; exit 3
    fi
    local out rc
    # ROS 환경 (setup.bash 는 정의 안 된 변수를 써서 set -u 를 잠시 끈다). 값은 tb3-bringup.service 와 같게.
    out=$(set +u; source /opt/ros/humble/setup.bash
          export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
          timeout 25 ros2 service call /reset std_srvs/srv/Trigger 2>&1)
    rc=$?
    if [ $rc -eq 0 ] && echo "$out" | grep -q 'success=True'; then
        status
    else
        echo "$out" | tail -2 >&2
        echo '{"error": "/reset 실패"}'; status; exit 4
    fi
}

case "$cmd" in
    status)   status ;;
    standby)  systemctl --user stop $SVC; lds_stop; status ;;           # stop 은 정상 종료까지 기다림 (최대 30초)
    wake)     systemctl --user start --no-block $SVC; status ;;
    restart)  systemctl --user restart --no-block $SVC; status ;;
    reboot)   status; sudo -n /usr/bin/systemctl reboot ;;
    poweroff) systemctl --user stop $SVC; lds_stop; status; sudo -n /usr/bin/systemctl poweroff ;;
    gyro_reset) gyro_reset ;;
    boot)     for _ in $(seq 60); do [ -n "$(lds_port)" ] && break; sleep 1; done; lds_stop ;;
    *)        echo '{"error": "unknown command"}'; exit 2 ;;
esac
