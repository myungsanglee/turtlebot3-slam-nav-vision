import { AppBar, Chip, Stack, Toolbar, Typography } from '@mui/material'
import type { Status } from '../api/types'
import PowerControl from './PowerControl'

const MODE_LABEL: Record<string, string> = { IDLE: '대기', MAPPING: '지도 만들기', NAVIGATION: '주행' }

export default function StatusBar({ status, connected }: { status: Status | null; connected: boolean }) {
  const v = status?.velocity
  return (
    <AppBar position="static" color="transparent" elevation={0} sx={{ borderBottom: 1, borderColor: 'divider' }}>
      <Toolbar sx={{ gap: 2, flexWrap: 'wrap' }}>
        <Typography variant="h6" sx={{ fontWeight: 700, mr: 'auto' }}>
          TurtleBot3 콘솔
        </Typography>
        {v && (
          <Typography variant="body2" color="text.secondary" sx={{ fontVariantNumeric: 'tabular-nums' }}>
            속도 {v.linear.toFixed(2)} m/s · 회전 {v.angular.toFixed(2)} rad/s
          </Typography>
        )}
        <Stack direction="row" spacing={1} sx={{ alignItems: 'center' }}>
          <Chip size="small" label={`모드: ${MODE_LABEL[status?.mode ?? ''] ?? '—'}`} variant="outlined" />
          <Chip size="small" label={connected ? '서버 연결됨' : '서버 연결 끊김'} color={connected ? 'success' : 'error'} />
          {/* 대기·꺼짐 등 일부러 끈 상태에선 '응답 없음' 경고를 띄우지 않는다 (전원 칩이 상태를 보여 줌) */}
          {(!status?.power || ['running', 'starting', 'unconfigured', 'error'].includes(status.power.state)) && (
            <Chip
              size="small"
              label={status?.robot_connected ? '로봇 데이터 수신' : '로봇 응답 없음'}
              color={status?.robot_connected ? 'success' : 'warning'}
            />
          )}
          <PowerControl power={status?.power} mode={status?.mode} />
        </Stack>
      </Toolbar>
    </AppBar>
  )
}
