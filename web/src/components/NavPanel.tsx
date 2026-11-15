import { Alert, Button, Card, CardContent, Chip, LinearProgress, Paper, Stack, Typography } from '@mui/material'
import { useNavCancel } from '../api/nav'
import type { NavState, NavigationStatus } from '../api/types'

const STATE: Record<NavState['state'], { label: string; color: 'default' | 'info' | 'success' | 'warning' | 'error' }> = {
  idle: { label: '대기', color: 'default' },
  sending: { label: '전송 중', color: 'info' },
  active: { label: '주행 중', color: 'info' },
  succeeded: { label: '도착', color: 'success' },
  canceled: { label: '취소됨', color: 'warning' },
  aborted: { label: '실패', color: 'error' },
  rejected: { label: '거절됨', color: 'error' },
  error: { label: '오류', color: 'error' },
}

export default function NavPanel({ nav, navigation }: { nav: NavState | null; navigation: NavigationStatus | undefined }) {
  const cancel = useNavCancel()
  const state = nav?.state ?? 'idle'
  const active = state === 'active' || state === 'sending'
  const fb = nav?.feedback

  return (
    <Card>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'center', mb: 1, gap: 1 }}>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, mr: 'auto' }}>
            주행
          </Typography>
          <Chip size="small" variant="outlined" label={navigation?.nav2_ready ? 'Nav2 준비됨' : 'Nav2 꺼짐'} color={navigation?.nav2_ready ? 'success' : 'default'} />
          <Chip size="small" label={STATE[state].label} color={STATE[state].color} />
        </Stack>

        {active && <LinearProgress sx={{ mb: 1.5, borderRadius: 2 }} />}
        {nav?.goal && (
          <Typography variant="body2" color="text.secondary">
            목표 ({nav.goal.x.toFixed(2)}, {nav.goal.y.toFixed(2)}) m
          </Typography>
        )}
        {fb && (
          <Stack direction="row" spacing={2} sx={{ mt: 1, flexWrap: 'wrap', fontVariantNumeric: 'tabular-nums' }}>
            <Typography variant="body2">남은 거리 {fb.distance_remaining.toFixed(2)} m</Typography>
            <Typography variant="body2">경과 {fb.navigation_time.toFixed(0)} s</Typography>
            {fb.estimated_time_remaining > 0 && <Typography variant="body2">예상 {fb.estimated_time_remaining.toFixed(0)} s</Typography>}
            {fb.recoveries > 0 && <Typography variant="body2" color="warning.main">복구 동작 {fb.recoveries}회</Typography>}
          </Stack>
        )}
        {nav?.message && !active && (
          <Typography variant="body2" sx={{ mt: 1 }} color={STATE[state].color === 'error' ? 'error' : 'text.secondary'}>
            {nav.message}
          </Typography>
        )}
        {!nav?.goal && (
          <Typography variant="body2" color="text.secondary">
            지도에서 「목표 지정」으로 목적지를 찍는다
          </Typography>
        )}

        <Button fullWidth variant="contained" color="error" sx={{ mt: 2 }} disabled={!active || cancel.isPending} onClick={() => cancel.mutate()}>
          주행 취소
        </Button>
        {cancel.error && <Alert severity="error" sx={{ mt: 1 }}>{cancel.error.message}</Alert>}
      </CardContent>
    </Card>
  )
}

/** 조작 화면 위에 겹치는 주행 상태 (주행 중일 때만) — 남은 거리와 「취소」 를 손 닿는 곳에 */
export function NavOverlay({ nav }: { nav: NavState | null }) {
  const cancel = useNavCancel()
  const state = nav?.state ?? 'idle'
  if (state !== 'active' && state !== 'sending') return null
  const fb = nav?.feedback
  return (
    <Paper elevation={6} sx={{ px: 1.5, py: 1, borderRadius: 2, bgcolor: 'rgba(17,21,27,0.85)', backdropFilter: 'blur(4px)' }}>
      <Stack direction="row" spacing={1.5} sx={{ alignItems: 'center' }}>
        <Chip size="small" color="info" label={STATE[state].label} />
        <Typography variant="body2" sx={{ fontVariantNumeric: 'tabular-nums' }}>
          {fb ? `남은 ${fb.distance_remaining.toFixed(2)} m` : '경로 계산 중'}
          {fb && fb.recoveries > 0 && ` · 복구 ${fb.recoveries}회`}
        </Typography>
        <Button size="small" variant="contained" color="error" disabled={cancel.isPending} onClick={() => cancel.mutate()}>
          취소
        </Button>
      </Stack>
    </Paper>
  )
}
