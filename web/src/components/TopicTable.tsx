import { Box, Card, CardContent, Chip, Table, TableBody, TableCell, TableHead, TableRow, Tooltip, Typography } from '@mui/material'
import type { TopicStatus } from '../api/types'

// 마지막 수신 후 경과 시간. 끊긴 뒤에도 서버가 마지막 수신 시각을 유지하므로 "3분 12초 전" 처럼 계속 늘어난다.
function ago(age: number | null) {
  if (age === null) return '기록 없음'
  if (age < 1) return '방금'
  if (age < 60) return `${Math.floor(age)}초 전`
  if (age < 3600) return `${Math.floor(age / 60)}분 ${Math.floor(age % 60)}초 전`
  return `${Math.floor(age / 3600)}시간 ${Math.floor((age % 3600) / 60)}분 전`
}

const clock = (epoch: number) => new Date(epoch * 1000).toLocaleString('ko-KR', { hour12: false })

function stateChip(t: TopicStatus) {
  if (t.state === 'off') return <Chip size="small" label="꺼짐" variant="outlined" />
  if (t.state === 'missing') return <Chip size="small" label="수신 없음" color="error" />
  if (t.rate < t.expected_hz * 0.7) return <Chip size="small" label="느림" color="warning" />
  return <Chip size="small" label="정상" color="success" />
}

export default function TopicTable({ topics }: { topics: TopicStatus[] }) {
  return (
    <Card>
      <CardContent>
        <Typography variant="subtitle1" sx={{ fontWeight: 600, mb: 1 }}>
          센서 수신 상태
        </Typography>
        <Box sx={{ overflowX: 'auto' }}>
        <Table size="small">
          <TableHead>
            <TableRow sx={{ '& th': { whiteSpace: 'nowrap' } }}>
              <TableCell>항목</TableCell>
              <TableCell align="right">주기</TableCell>
              <TableCell align="right">마지막 수신</TableCell>
              <TableCell align="right">상태</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {topics.map((t) => (
              <TableRow key={t.key}>
                <TableCell>
                  {t.label}
                  <Typography variant="caption" color="text.secondary" sx={{ display: { xs: 'none', sm: 'block' }, fontFamily: 'monospace' }}>
                    {t.topic}
                  </Typography>
                </TableCell>
                <TableCell align="right" sx={{ fontVariantNumeric: 'tabular-nums', whiteSpace: 'nowrap' }}>
                  {t.rate.toFixed(1)} / {t.expected_hz} Hz
                </TableCell>
                <TableCell align="right" sx={{ fontVariantNumeric: 'tabular-nums', whiteSpace: 'nowrap' }}>
                  <Tooltip title={t.last_received ? `마지막 수신 ${clock(t.last_received)}` : '서버가 켜진 뒤 한 번도 받지 못함'}>
                    <Typography variant="body2" component="span" color={t.state === 'missing' ? 'error' : 'inherit'}>
                      {ago(t.age)}
                    </Typography>
                  </Tooltip>
                </TableCell>
                <TableCell align="right">{stateChip(t)}</TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
        </Box>
      </CardContent>
    </Card>
  )
}
