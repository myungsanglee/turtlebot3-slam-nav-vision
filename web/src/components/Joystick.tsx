import { Box, Card, CardContent, Slider, Stack, Typography } from '@mui/material'
import { useCallback, useEffect, useRef, useState } from 'react'

// 가상 조이스틱 — 누르고 있는 동안만 0.1초마다 (x, y) 를 서버로 보낸다. 손을 떼면 즉시 정지를 보낸다.
// 진짜 안전 장치는 서버에 있다: 0.3초 동안 명령이 없으면 서버가 정지시킨다 (teleop.py) — 브라우저가 멈추거나
// 탭이 닫히거나 와이파이가 끊겨도 로봇이 계속 달리지 않는다. 속도 상한도 서버가 자른다.
const SIZE = 168
const KNOB = 60

export default function Joystick({ send, connected, maxLinear = 0.15, maxAngular = 1.0 }: { send: (m: object) => void; connected: boolean; maxLinear?: number; maxAngular?: number }) {
  const baseRef = useRef<HTMLDivElement>(null)
  const vec = useRef({ x: 0, y: 0 })
  const [active, setActive] = useState(false)
  const [knob, setKnob] = useState({ x: 0, y: 0 }) // 손잡이 위치 (px, 화면 표시용)
  const [stick, setStick] = useState({ x: 0, y: 0 }) // 정규화 값 (화면 표시용 — 전송은 vec ref 로)
  const [scale, setScale] = useState(0.6)

  const release = useCallback(() => {
    setActive(false)
    setKnob({ x: 0, y: 0 })
    setStick({ x: 0, y: 0 })
    if (vec.current.x !== 0 || vec.current.y !== 0) send({ type: 'teleop_release' })
    vec.current = { x: 0, y: 0 }
  }, [send])

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

  const update = (clientX: number, clientY: number) => {
    const r = baseRef.current!.getBoundingClientRect()
    const R = (SIZE - KNOB) / 2
    let dx = clientX - (r.left + r.width / 2)
    let dy = clientY - (r.top + r.height / 2)
    const d = Math.hypot(dx, dy)
    if (d > R) {
      dx = (dx / d) * R
      dy = (dy / d) * R
    }
    setKnob({ x: dx, y: dy })
    vec.current = { x: dx / R, y: -dy / R } // 화면 위 = 전진(+y)
    setStick(vec.current)
  }

  const v = stick.y * maxLinear * scale
  const w = -stick.x * maxAngular * scale

  return (
    <Card>
      <CardContent>
        <Typography variant="subtitle1" sx={{ fontWeight: 600, mb: 1 }}>
          수동 조작
        </Typography>
        <Stack direction="row" spacing={3} sx={{ alignItems: 'center' }}>
          <Box
            ref={baseRef}
            onPointerDown={(e) => {
              if (!connected) return
              e.currentTarget.setPointerCapture(e.pointerId)
              setActive(true)
              update(e.clientX, e.clientY)
            }}
            onPointerMove={(e) => active && update(e.clientX, e.clientY)}
            onPointerUp={release}
            onPointerCancel={release}
            onLostPointerCapture={release}
            sx={{
              width: SIZE, height: SIZE, flex: 'none', borderRadius: '50%', position: 'relative', touchAction: 'none', userSelect: 'none',
              bgcolor: '#11151b', border: '2px solid', borderColor: active ? 'primary.main' : 'divider', opacity: connected ? 1 : 0.4,
              cursor: connected ? 'grab' : 'not-allowed',
            }}
          >
            {['전진', '후진'].map((t, i) => (
              <Typography key={t} variant="caption" color="text.secondary" sx={{ position: 'absolute', left: 0, right: 0, textAlign: 'center', [i ? 'bottom' : 'top']: 6 }}>
                {t}
              </Typography>
            ))}
            <Box
              sx={{
                position: 'absolute', width: KNOB, height: KNOB, borderRadius: '50%', left: (SIZE - KNOB) / 2 - 2 + knob.x, top: (SIZE - KNOB) / 2 - 2 + knob.y,
                bgcolor: active ? 'primary.main' : '#3a4250', boxShadow: 3, transition: active ? 'none' : 'left 0.15s, top 0.15s',
              }}
            />
          </Box>
          <Stack spacing={1} sx={{ flex: 1, minWidth: 0 }}>
            <Typography variant="body2" sx={{ fontVariantNumeric: 'tabular-nums' }}>
              전진 {v.toFixed(2)} m/s · 회전 {w.toFixed(2)} rad/s
            </Typography>
            <Typography variant="caption" color="text.secondary">
              속도 {Math.round(scale * 100)}% (최대 {(maxLinear * scale).toFixed(2)} m/s)
            </Typography>
            <Slider size="small" min={0.2} max={1} step={0.1} value={scale} onChange={(_, val) => setScale(val as number)} />
            <Typography variant="caption" color="text.secondary">
              누르고 있는 동안만 움직이고, 손을 떼면 멈춘다. 연결이 끊겨도 0.3초 안에 서버가 멈춘다. 움직이기 시작하면 진행 중인 자동 주행은 취소된다.
            </Typography>
          </Stack>
        </Stack>
      </CardContent>
    </Card>
  )
}
