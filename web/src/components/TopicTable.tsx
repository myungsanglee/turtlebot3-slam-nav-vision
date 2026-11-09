import { Box, Card, CardContent, Chip, Table, TableBody, TableCell, TableHead, TableRow, Typography } from '@mui/material'
import type { TopicStatus } from '../api/types'

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
                  {t.age === null ? '—' : t.age < 1 ? '방금' : `${t.age.toFixed(1)}초 전`}
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
