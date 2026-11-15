import KeyboardIcon from '@mui/icons-material/Keyboard'
import { Box, Paper, Slider, Stack, Tooltip, Typography, useMediaQuery } from '@mui/material'
import { useCallback, useEffect, useRef, useState } from 'react'

// ───────────────────────────────────────────────────────────────────────────
// 수동 조작 — 화면 위에 겹쳐 두는 조이스틱 + 키보드(방향키·WASD)
//   누르고 있는 동안만 0.1초마다 (x, y) 를 서버로 보낸다. 떼면 즉시 정지를 보낸다.
//   진짜 안전 장치는 서버에 있다: 0.3초 동안 명령이 없으면 서버가 정지시킨다 (teleop.py) — 브라우저가 멈추거나
//   탭이 닫히거나 와이파이가 끊겨도 로봇이 계속 달리지 않는다. 속도 상한도 서버가 자른다.
// ───────────────────────────────────────────────────────────────────────────

type Vec = { x: number; y: number }
const KEYS: Record<string, Vec> = {
  ArrowUp: { x: 0, y: 1 }, KeyW: { x: 0, y: 1 },
  ArrowDown: { x: 0, y: -1 }, KeyS: { x: 0, y: -1 },
  ArrowLeft: { x: -1, y: 0 }, KeyA: { x: -1, y: 0 },
  ArrowRight: { x: 1, y: 0 }, KeyD: { x: 1, y: 0 },
}

// 입력창에 글자를 치는 중이면 키보드 조작을 하지 않는다
const typing = (t: EventTarget | null) =>
  t instanceof HTMLElement && (t.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(t.tagName))

export default function Joystick({ send, connected, maxLinear = 0.15, maxAngular = 1.0 }: { send: (m: object) => void; connected: boolean; maxLinear?: number; maxAngular?: number }) {
  const touch = useMediaQuery('(pointer: coarse)')
  const SIZE = touch ? 150 : 128
  const KNOB = touch ? 58 : 50
  const baseRef = useRef<HTMLDivElement>(null)
  const vec = useRef<Vec>({ x: 0, y: 0 })
  const keys = useRef(new Set<string>())
  const [active, setActive] = useState<'pointer' | 'keys' | null>(null)
  const [knob, setKnob] = useState<Vec>({ x: 0, y: 0 }) // 손잡이 위치 (px, 화면 표시용)
  const [stick, setStick] = useState<Vec>({ x: 0, y: 0 }) // 정규화 값 (화면 표시용 — 전송은 vec ref 로)
  const [scale, setScale] = useState(0.6)
  const R = (SIZE - KNOB) / 2

  const show = useCallback(
    (v: Vec) => {
      vec.current = v
      setStick(v)
      setKnob({ x: v.x * R, y: -v.y * R })
    },
    [R],
  )

  const release = useCallback(() => {
    keys.current.clear()
    setActive(null)
    if (vec.current.x !== 0 || vec.current.y !== 0) send({ type: 'teleop_release' })
    show({ x: 0, y: 0 })
  }, [send, show])

  // 누르고 있는 동안 0.1초마다 전송
  useEffect(() => {
    if (!active) return
    const id = window.setInterval(() => send({ type: 'teleop', x: vec.current.x, y: vec.current.y, scale }), 100)
    return () => window.clearInterval(id)
  }, [active, scale, send])

  // 창을 떠나거나(탭 전환·앱 전환) 포커스를 잃으면 즉시 정지
  useEffect(() => {
    window.addEventListener('blur', release)
    document.addEventListener('visibilitychange', release)
    return () => {
      window.removeEventListener('blur', release)
      document.removeEventListener('visibilitychange', release)
    }
  }, [release])

  // 키보드: 누른 키들의 합 (위+왼쪽 = 전진하며 왼쪽으로 회전). 스페이스 = 즉시 정지
  useEffect(() => {
    const fromKeys = () => {
      let x = 0
      let y = 0
      keys.current.forEach((k) => {
        x += KEYS[k].x
        y += KEYS[k].y
      })
      return { x: Math.max(-1, Math.min(1, x)), y: Math.max(-1, Math.min(1, y)) }
    }
    const down = (e: KeyboardEvent) => {
      if (typing(e.target) || e.altKey || e.ctrlKey || e.metaKey) return
      if (e.code === 'Space' && keys.current.size) {
        e.preventDefault()
        release()
        return
      }
      if (!(e.code in KEYS) || !connected) return
      e.preventDefault() // 방향키로 페이지가 스크롤되지 않게
      if (keys.current.has(e.code)) return // 키 자동 반복
      keys.current.add(e.code)
      setActive('keys')
      show(fromKeys())
    }
    const up = (e: KeyboardEvent) => {
      if (!keys.current.delete(e.code)) return
      if (keys.current.size === 0) release()
      else show(fromKeys())
    }
    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
    }
  }, [connected, release, show])

  const update = (clientX: number, clientY: number) => {
    const r = baseRef.current!.getBoundingClientRect()
    let dx = clientX - (r.left + r.width / 2)
    let dy = clientY - (r.top + r.height / 2)
    const d = Math.hypot(dx, dy)
    if (d > R) {
      dx = (dx / d) * R
      dy = (dy / d) * R
    }
    show({ x: dx / R, y: -dy / R }) // 화면 위 = 전진(+y)
  }

  const v = stick.y * maxLinear * scale
  const w = -stick.x * maxAngular * scale

  return (
    <Paper elevation={6} sx={{ p: 1, borderRadius: 3, bgcolor: 'rgba(17,21,27,0.82)', backdropFilter: 'blur(4px)', width: SIZE + 16 }}>
      <Box
        ref={baseRef}
        onPointerDown={(e) => {
          if (!connected) return
          e.currentTarget.setPointerCapture(e.pointerId)
          setActive('pointer')
          update(e.clientX, e.clientY)
        }}
        onPointerMove={(e) => active === 'pointer' && update(e.clientX, e.clientY)}
        onPointerUp={release}
        onPointerCancel={release}
        onLostPointerCapture={() => active === 'pointer' && release()}
        sx={{
          width: SIZE, height: SIZE, borderRadius: '50%', position: 'relative', touchAction: 'none', userSelect: 'none',
          bgcolor: 'rgba(11,13,16,0.7)', border: '2px solid', borderColor: active ? 'primary.main' : 'divider', opacity: connected ? 1 : 0.4,
          cursor: connected ? 'grab' : 'not-allowed',
        }}
      >
        {['전진', '후진'].map((t, i) => (
          <Typography key={t} variant="caption" color="text.secondary" sx={{ position: 'absolute', left: 0, right: 0, textAlign: 'center', fontSize: 11, [i ? 'bottom' : 'top']: 4 }}>
            {t}
          </Typography>
        ))}
        <Box
          sx={{
            position: 'absolute', width: KNOB, height: KNOB, borderRadius: '50%', left: R - 2 + knob.x, top: R - 2 + knob.y,
            bgcolor: active ? 'primary.main' : '#3a4250', boxShadow: 3, transition: active ? 'none' : 'left 0.15s, top 0.15s',
          }}
        />
      </Box>
      <Typography variant="caption" sx={{ display: 'block', textAlign: 'center', fontVariantNumeric: 'tabular-nums', mt: 0.5 }}>
        {v.toFixed(2)} m/s · {w.toFixed(2)} rad/s
      </Typography>
      <Stack direction="row" spacing={1} sx={{ alignItems: 'center', px: 0.5 }}>
        <Tooltip title={`속도 ${Math.round(scale * 100)}% (최대 ${(maxLinear * scale).toFixed(2)} m/s)`}>
          <Slider size="small" min={0.2} max={1} step={0.1} value={scale} onChange={(_, val) => setScale(val as number)} aria-label="속도" />
        </Tooltip>
        {!touch && (
          <Tooltip title="키보드: 방향키 또는 W·A·S·D 를 누르고 있는 동안 이동, 스페이스 = 정지. 떼면 멈춘다. 움직이기 시작하면 진행 중인 자동 주행은 취소된다.">
            <KeyboardIcon fontSize="small" color={active === 'keys' ? 'primary' : 'disabled'} />
          </Tooltip>
        )}
      </Stack>
    </Paper>
  )
}
