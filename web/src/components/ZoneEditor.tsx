import { Alert, Button, Chip, Stack, ToggleButton, ToggleButtonGroup, Tooltip, Typography } from '@mui/material'
import { useState } from 'react'
import { useSaveZones } from '../api/zones'
import type { Point, Zone, ZoneKind } from '../api/types'

// ───────────────────────────────────────────────────────────────────────────
// 금지 구역·가상 벽 편집 — 거울·유리처럼 라이다·카메라가 못 보는 곳을 사람이 막는다 (docs/my_web_api.md 4.18)
//   가상 벽: 점 2개 이상을 이은 선 (거울·유리문 앞에 한 줄) / 금지 구역: 점 3개 이상 닫힌 영역
//   저장 지도로 주행 중일 때만 편집 — SLAM 지도는 켤 때마다 원점이 달라 구역을 둘 수 없다.
//   저장하면 서버가 Nav2 Keepout Filter 마스크를 바로 바꾼다 (주행 중에도 즉시 반영).
// ───────────────────────────────────────────────────────────────────────────

const NEED: Record<ZoneKind, number> = { wall: 2, area: 3 }
const LABEL: Record<ZoneKind, string> = { wall: '가상 벽', area: '금지 구역' }

export interface Draft {
  kind: ZoneKind
  points: Point[]
}

// 지도 위에 겹치는 편집 막대 (도구 = 'zone' 일 때)
export function ZoneDraftBar({ map, zones, draft, setDraft, onClose }: {
  map: string
  zones: Zone[]
  draft: Draft
  setDraft: (d: Draft) => void
  onClose: () => void
}) {
  const save = useSaveZones(map)
  const need = NEED[draft.kind]
  const done = () => {
    const n = zones.filter((z) => z.kind === draft.kind).length + 1
    const zone: Zone = { kind: draft.kind, points: draft.points, name: `${draft.kind === 'wall' ? '벽' : '구역'} ${n}` }
    save.mutate([...zones, zone], { onSuccess: () => setDraft({ ...draft, points: [] }) })
  }
  return (
    <Alert
      severity={save.error ? 'error' : 'warning'}
      icon={false}
      sx={{ position: 'absolute', top: 8, left: 8, right: 8, alignItems: 'center', '& .MuiAlert-message': { width: '100%' } }}
    >
      <Stack direction="row" sx={{ alignItems: 'center', gap: 1, flexWrap: 'wrap' }}>
        <ToggleButtonGroup
          size="small"
          exclusive
          value={draft.kind}
          onChange={(_, v: ZoneKind | null) => v && setDraft({ kind: v, points: draft.points })}
        >
          <ToggleButton value="wall">가상 벽</ToggleButton>
          <ToggleButton value="area">금지 구역</ToggleButton>
        </ToggleButtonGroup>
        <Typography variant="body2" sx={{ mr: 'auto' }}>
          점 {draft.points.length}개{draft.points.length < need && ` (${need}개 이상)`}
        </Typography>
        <Button size="small" disabled={draft.points.length === 0} onClick={() => setDraft({ ...draft, points: draft.points.slice(0, -1) })}>
          되돌리기
        </Button>
        <Button size="small" variant="contained" disabled={draft.points.length < need || save.isPending} onClick={done}>
          완료
        </Button>
        <Button size="small" color="inherit" onClick={onClose}>
          닫기
        </Button>
      </Stack>
      {save.error && <Typography variant="caption">{save.error.message}</Typography>}
    </Alert>
  )
}

// 지도 아래 목록 — 구역마다 칩, 삭제는 확인 후
export function ZoneList({ map, zones, applied }: { map: string; zones: Zone[]; applied: boolean }) {
  const save = useSaveZones(map)
  const [confirm, setConfirm] = useState<number | null>(null)
  return (
    <Stack direction="row" sx={{ alignItems: 'center', gap: 0.75, flexWrap: 'wrap', mt: 1 }}>
      <Tooltip title="거울·유리처럼 센서가 못 보는 곳 — Nav2 가 통과하지 않는다. 장애물 여유 폭이 붙지 않으니 실제 물체보다 10cm 쯤 앞에 그을 것">
        <Typography variant="caption" color="text.secondary">
          금지 구역 ({map}){zones.length > 0 && !applied && ' · 적용 대기'}:
        </Typography>
      </Tooltip>
      {zones.length === 0 && (
        <Typography variant="caption" color="text.secondary">
          없음 — 「금지 구역」 도구로 그리기
        </Typography>
      )}
      {zones.map((z, i) =>
        confirm === i ? (
          <Stack key={i} direction="row" sx={{ alignItems: 'center', gap: 0.5 }}>
            <Typography variant="caption">{z.name || LABEL[z.kind]} 삭제?</Typography>
            <Button
              size="small"
              color="error"
              disabled={save.isPending}
              onClick={() => save.mutate(zones.filter((_, k) => k !== i), { onSettled: () => setConfirm(null) })}
            >
              삭제
            </Button>
            <Button size="small" color="inherit" onClick={() => setConfirm(null)}>
              취소
            </Button>
          </Stack>
        ) : (
          <Chip
            key={i}
            size="small"
            variant="outlined"
            color="error"
            label={`${z.name || LABEL[z.kind]} · ${LABEL[z.kind]}`}
            onDelete={() => setConfirm(i)}
          />
        ),
      )}
      {save.error && (
        <Typography variant="caption" color="error">
          {save.error.message}
        </Typography>
      )}
    </Stack>
  )
}
