import { Card, CardContent, Chip, LinearProgress, Stack, Typography } from '@mui/material'
import type { Battery } from '../api/types'

// LiPo 는 잔량 % 가 전압에 비선형이라 판정은 셀당 전압으로 한다 (3.5V 충전 권장 / 3.3V 즉시 중단 — config/web_api.yaml)
const LEVEL = {
  ok: { label: '정상', color: 'success' },
  warn: { label: '충전 권장', color: 'warning' },
  critical: { label: '즉시 중단', color: 'error' },
} as const

export default function BatteryCard({ battery }: { battery: Battery | null }) {
  return (
    <Card>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'center', mb: 1 }}>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, mr: 'auto' }}>
            배터리
          </Typography>
          {battery && <Chip size="small" label={LEVEL[battery.level].label} color={LEVEL[battery.level].color} />}
        </Stack>
        {battery ? (
          <>
            <Typography variant="h3" sx={{ fontWeight: 700, fontVariantNumeric: 'tabular-nums' }}>
              {battery.voltage.toFixed(2)}
              <Typography component="span" variant="h6" color="text.secondary" sx={{ ml: 0.5 }}>
                V
              </Typography>
            </Typography>
            <Typography variant="body2" color="text.secondary" sx={{ mb: 1.5 }}>
              셀당 {battery.cell_voltage.toFixed(2)} V ({battery.cells}셀) · 완충 4.20 / 충전 권장 3.50 / 중단 3.30
            </Typography>
            <LinearProgress
              variant="determinate"
              value={Math.max(0, Math.min(100, battery.percentage))}
              color={LEVEL[battery.level].color}
              sx={{ height: 8, borderRadius: 4 }}
            />
            <Typography variant="caption" color="text.secondary">
              잔량 약 {battery.percentage.toFixed(0)}% (OpenCR 의 전압 직선 환산 — 참고용)
            </Typography>
          </>
        ) : (
          <Typography color="text.secondary">배터리 데이터 없음 (로봇 bringup 확인)</Typography>
        )}
      </CardContent>
    </Card>
  )
}
