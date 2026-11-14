import MyLocationIcon from '@mui/icons-material/MyLocation'
import { Alert, Button, Chip, CircularProgress, Stack, Tooltip } from '@mui/material'
import { useRelocalize } from '../api/nav'
import type { LocalizationCandidate, LocalizationStatus, Pose } from '../api/types'

// ───────────────────────────────────────────────────────────────────────────
// 자동 위치 찾기 (저장 지도 주행) — docs/my_localization.md
//   서버(relocalizer)가 라이다 스캔을 지도 전체에 맞춰 보고 스스로 위치를 정한다.
//   확신이 없으면(비슷한 곳이 여럿) 후보를 보여 주고 사람이 고른다 → 고른 후보는 「초기 위치」 확인 창으로.
// ───────────────────────────────────────────────────────────────────────────

const STATE: Record<LocalizationStatus['state'], { label: string; color: 'default' | 'info' | 'success' | 'warning' | 'error' }> = {
  waiting: { label: '위치: 준비 중', color: 'default' },
  searching: { label: '위치 찾는 중', color: 'info' },
  localized: { label: '위치 확인됨', color: 'success' },
  ambiguous: { label: '위치 후보 여럿', color: 'warning' },
  failed: { label: '위치 모름', color: 'error' },
  lost: { label: '위치 잃음', color: 'error' },
}

const METHOD: Record<string, string> = {
  last_pose: '마지막 위치에서 시작',
  global: '지도 전체에서 찾음',
  manual: '직접 지정',
  recovered: '잃었다가 다시 찾음',
}

const deg = (r: number) => Math.round((r * 180) / Math.PI)

/** 지도 머리줄에 들어가는 상태 칩 + 「위치 찾기」 버튼 */
export function LocalizationControl({ loc }: { loc: LocalizationStatus | null }) {
  const reloc = useRelocalize()
  const st = loc ? STATE[loc.state] : STATE.waiting
  const searching = loc?.state === 'searching' || reloc.isPending
  const tip = loc
    ? [loc.message, loc.method && METHOD[loc.method], loc.track_score != null && `지금 맞는 정도 ${loc.track_score.toFixed(2)}`]
        .filter(Boolean)
        .join(' · ')
    : '자동 위치 찾기 노드 기다리는 중'
  return (
    <>
      <Tooltip title={reloc.error?.message || tip}>
        <Chip size="small" label={st.label} color={reloc.error ? 'error' : st.color} variant={loc?.state === 'localized' ? 'outlined' : 'filled'} />
      </Tooltip>
      <Tooltip title="라이다 스캔을 지도 전체에 맞춰 로봇 위치를 다시 찾는다 (로봇을 옮겨 놓았을 때)">
        <span>
          <Button
            size="small"
            variant="outlined"
            startIcon={searching ? <CircularProgress size={14} /> : <MyLocationIcon fontSize="small" />}
            disabled={searching}
            onClick={() => reloc.mutate()}
          >
            위치 찾기
          </Button>
        </span>
      </Tooltip>
    </>
  )
}

/** 확신 못 할 때 지도 위에 띄우는 안내 + 후보 버튼 (누르면 그 자리로 「초기 위치」 확인 창) */
export function LocalizationAlert({ loc, onPick }: { loc: LocalizationStatus | null; onPick: (p: Pose) => void }) {
  if (!loc || !['ambiguous', 'failed', 'lost'].includes(loc.state)) return null
  const cands: LocalizationCandidate[] = loc.candidates.slice(0, 3)
  return (
    <Alert severity={loc.state === 'ambiguous' ? 'warning' : 'error'} sx={{ mb: 1 }}>
      {loc.message}
      {cands.length > 0 && loc.state !== 'lost' && (
        <Stack direction="row" sx={{ mt: 1, gap: 1, flexWrap: 'wrap' }}>
          {cands.map((c, i) => (
            <Button key={i} size="small" variant="outlined" color="inherit" onClick={() => onPick({ x: c.x, y: c.y, yaw: c.yaw })}>
              후보 {i + 1} ({c.x.toFixed(2)}, {c.y.toFixed(2)}) {deg(c.yaw)}° · {c.score.toFixed(2)}
            </Button>
          ))}
        </Stack>
      )}
    </Alert>
  )
}
