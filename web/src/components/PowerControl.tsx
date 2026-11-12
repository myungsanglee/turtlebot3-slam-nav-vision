import PowerSettingsNewIcon from '@mui/icons-material/PowerSettingsNew'
import {
  Button,
  Chip,
  Dialog,
  DialogActions,
  DialogContent,
  DialogContentText,
  DialogTitle,
  Divider,
  IconButton,
  ListItemText,
  Menu,
  MenuItem,
  Tooltip,
} from '@mui/material'
import { useState } from 'react'
import { usePowerAction } from '../api/power'
import type { Power, PowerAction, PowerState } from '../api/types'

// ───────────────────────────────────────────────────────────────────────────
// 로봇 전원 — 대기·켜기·ROS 재시작·Pi 재부팅·끄기 (docs/my_web_api.md 4.19)
//   대기 = Pi 의 bringup(라이다·카메라·모터 제어)만 끔. Pi·네트워크는 켜진 채라 웹에서 다시 켤 수 있다.
//   Pi 끄기 뒤엔 네트워크가 없어 웹에서 켤 수 없다 → 배터리 재연결.
//   부팅 시 기본은 대기 (robot-standby.service) — 쓰지 않을 때 라이다가 돌며 배터리를 쓰지 않게.
// ───────────────────────────────────────────────────────────────────────────

const STATE: Record<PowerState, { label: string; color: 'default' | 'success' | 'info' | 'warning' | 'error' }> = {
  unconfigured: { label: '전원 제어 미설정', color: 'default' },
  unreachable: { label: 'Pi 연결 안 됨', color: 'error' },
  off: { label: '로봇 꺼짐', color: 'default' },
  rebooting: { label: '재부팅 중', color: 'info' },
  standby: { label: '로봇 대기', color: 'default' },
  starting: { label: '로봇 시작 중', color: 'info' },
  running: { label: '로봇 실행 중', color: 'success' },
  stopping: { label: '로봇 정지 중', color: 'info' },
  error: { label: '로봇 시작 실패', color: 'error' },
}

const ACTION: Record<PowerAction, { label: string; detail: string; danger?: boolean }> = {
  wake: {
    label: '켜기',
    detail: '라이다·카메라·모터 제어를 시작합니다. 준비까지 1분쯤 걸립니다. 시작 후 5초 동안은 로봇을 건드리지 마세요 (자이로 영점).',
  },
  standby: {
    label: '대기',
    detail: '라이다·카메라·모터 제어를 멈춰 배터리를 아낍니다. Pi 는 켜진 채라 웹에서 다시 켤 수 있습니다.',
  },
  restart: {
    label: 'ROS 재시작',
    detail: '로봇 쪽 ROS(bringup)를 다시 시작합니다 — 센서가 끊겼을 때 복구용. odom 이 0 으로 초기화되고, 5초 동안 로봇을 건드리지 마세요.',
  },
  reboot: {
    label: 'Pi 재부팅',
    detail: 'Raspberry Pi 를 재부팅합니다. 1~2분 뒤 다시 연결되면 대기 상태로 켜집니다.',
    danger: true,
  },
  poweroff: {
    label: 'Pi 끄기',
    detail: 'Raspberry Pi 를 안전하게 끕니다 (SD 카드 보호). 끈 뒤에는 웹에서 켤 수 없습니다 — 배터리를 뺐다가 다시 연결해야 켜집니다. 보관할 땐 배터리를 분리하세요 (Pi 전원은 로봇 스위치를 거치지 않음).',
    danger: true,
  },
}

const MODE_LABEL: Record<string, string> = { MAPPING: '지도 만들기', NAVIGATION: '주행' }

// 상태별로 고를 수 있는 동작
function available(state: PowerState): PowerAction[] {
  switch (state) {
    case 'standby':
      return ['wake', 'reboot', 'poweroff']
    case 'error':
      return ['wake', 'standby', 'reboot', 'poweroff']
    case 'running':
    case 'starting':
      return ['standby', 'restart', 'reboot', 'poweroff']
    default:
      return []
  }
}

export default function PowerControl({ power, mode }: { power: Power | undefined; mode: string | undefined }) {
  const act = usePowerAction()
  const [anchor, setAnchor] = useState<HTMLElement | null>(null)
  const [confirm, setConfirm] = useState<PowerAction | null>(null)
  const state: PowerState = power?.state ?? 'unconfigured'
  const s = STATE[state]
  const actions = available(state)
  const busy = !!power?.busy || act.isPending
  const failed = power?.last && !power.last.ok ? `${ACTION[power.last.action].label} 실패: ${power.last.message}` : ''
  const error = act.error?.message || failed

  const run = (a: PowerAction) => {
    setConfirm(null)
    act.mutate(a)
  }
  const stopsMode = confirm && confirm !== 'wake' && mode && mode !== 'IDLE'

  return (
    <>
      <Tooltip title={error || (power?.throttled && power.throttled !== '0x0' ? `Pi 전원 플래그 ${power.throttled} (저전압 이력)` : '')}>
        <Chip size="small" label={s.label} color={error ? 'error' : s.color} variant={state === 'running' ? 'filled' : 'outlined'} />
      </Tooltip>
      {(state === 'standby' || state === 'error') && (
        <Button size="small" variant="contained" disabled={busy} onClick={() => setConfirm('wake')}>
          켜기
        </Button>
      )}
      <Tooltip title="로봇 전원">
        <span>
          <IconButton size="small" disabled={actions.length === 0 || busy} onClick={(e) => setAnchor(e.currentTarget)}>
            <PowerSettingsNewIcon fontSize="small" />
          </IconButton>
        </span>
      </Tooltip>
      <Menu anchorEl={anchor} open={!!anchor} onClose={() => setAnchor(null)}>
        {actions.flatMap((a, i) => [
          ...(ACTION[a].danger && i > 0 && !ACTION[actions[i - 1]].danger ? [<Divider key={`d-${a}`} />] : []),
          <MenuItem
            key={a}
            onClick={() => {
              setAnchor(null)
              setConfirm(a)
            }}
          >
            <ListItemText slotProps={{ primary: { color: ACTION[a].danger ? 'error' : undefined } }}>{ACTION[a].label}</ListItemText>
          </MenuItem>,
        ])}
      </Menu>
      <Dialog open={!!confirm} onClose={() => setConfirm(null)} maxWidth="xs">
        {confirm && (
          <>
            <DialogTitle>{ACTION[confirm].label}</DialogTitle>
            <DialogContent>
              <DialogContentText>{ACTION[confirm].detail}</DialogContentText>
              {stopsMode && (
                <DialogContentText color="warning.main" sx={{ mt: 1 }}>
                  지금 「{MODE_LABEL[mode ?? ""] ?? mode}」 중입니다 — 먼저 끄고 진행합니다 (저장하지 않은 지도는 사라집니다).
                </DialogContentText>
              )}
            </DialogContent>
            <DialogActions>
              <Button color="inherit" onClick={() => setConfirm(null)}>
                취소
              </Button>
              <Button variant="contained" color={ACTION[confirm].danger ? 'error' : 'primary'} onClick={() => run(confirm)}>
                {ACTION[confirm].label}
              </Button>
            </DialogActions>
          </>
        )}
      </Dialog>
    </>
  )
}
