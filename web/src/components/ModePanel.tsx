import DeleteOutlineIcon from '@mui/icons-material/DeleteOutlined'
import {
  Alert, Box, Button, Card, CardContent, Chip, Collapse, Dialog, DialogActions, DialogContent, DialogTitle, FormControlLabel,
  IconButton, LinearProgress, MenuItem, Stack, Switch, TextField, Typography,
} from '@mui/material'
import { useState } from 'react'
import { useDeleteMap, useMaps, useModeLog, useSaveMap, useSetMode } from '../api/mode'
import type { ModeState } from '../api/types'

// 운용 모드 — 터미널 없이 지도 만들기 → 저장 → 저장 지도로 주행 (docs/web_ui_design.md 4장, docs/my_web_api.md 4.14)
const MODE = {
  IDLE: { label: '대기', color: 'default' },
  MAPPING: { label: '지도 만들기', color: 'info' },
  NAVIGATION: { label: '주행', color: 'success' },
} as const

const PHASE: Record<ModeState['phase'], string> = { idle: '', starting: '시작 중', running: '', stopping: '정지 중', error: '오류' }

const today = () => {
  const d = new Date()
  const p = (n: number) => String(n).padStart(2, '0')
  return `map_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`
}

export default function ModePanel({ mode }: { mode: ModeState | null }) {
  const maps = useMaps()
  const setMode = useSetMode()
  const saveMap = useSaveMap()
  const deleteMap = useDeleteMap()
  const [vision, setVision] = useState(true)
  const [selected, setSelected] = useState('')
  const [name, setName] = useState(today)
  const [confirmDelete, setConfirmDelete] = useState<string | null>(null)
  const [showLog, setShowLog] = useState(false)
  const log = useModeLog(showLog || mode?.phase === 'error')

  const busy = mode?.phase === 'starting' || mode?.phase === 'stopping' || setMode.isPending
  const current = mode?.mode ?? 'IDLE'
  const mapList = maps.data ?? []
  const chosen = selected || mapList[0]?.name || ''
  const error = setMode.error ?? saveMap.error ?? deleteMap.error

  return (
    <Card>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'center', gap: 1, mb: 1 }}>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, mr: 'auto' }}>
            운용 모드
          </Typography>
          {mode?.phase && PHASE[mode.phase] && <Chip size="small" label={PHASE[mode.phase]} color={mode.phase === 'error' ? 'error' : 'warning'} />}
          <Chip label={MODE[current].label} color={MODE[current].color} />
        </Stack>
        {busy && <LinearProgress sx={{ mb: 1, borderRadius: 2 }} />}
        {mode?.message && (
          <Typography variant="body2" color={mode.phase === 'error' ? 'error' : 'text.secondary'} sx={{ mb: 1 }}>
            {mode.message}
          </Typography>
        )}
        {mode?.external && (
          <Alert severity="warning" sx={{ mb: 1 }}>
            서버 터미널 등 다른 곳에서 SLAM/Nav2 가 실행 중입니다. 웹에서 켜려면 그쪽을 먼저 끄세요.
          </Alert>
        )}

        {/* 모드 전환 */}
        <Stack spacing={1}>
          <Stack direction="row" spacing={1}>
            <Button fullWidth variant={current === 'MAPPING' ? 'contained' : 'outlined'} disabled={busy || current === 'MAPPING'} onClick={() => setMode.mutate({ mode: 'MAPPING', vision })}>
              지도 만들기 시작
            </Button>
            <Button fullWidth color="inherit" variant="outlined" disabled={busy || current === 'IDLE'} onClick={() => setMode.mutate({ mode: 'IDLE' })}>
              정지
            </Button>
          </Stack>
          <Stack direction="row" spacing={1}>
            <TextField select size="small" label="저장 지도" value={chosen} onChange={(e) => setSelected(e.target.value)} sx={{ flex: 1 }} disabled={!mapList.length}>
              {mapList.map((m) => (
                <MenuItem key={m.name} value={m.name}>
                  {m.name} ({m.size_m[0]}×{m.size_m[1]} m)
                </MenuItem>
              ))}
            </TextField>
            <Button
              variant={current === 'NAVIGATION' ? 'contained' : 'outlined'}
              color="success"
              disabled={busy || !chosen || (current === 'NAVIGATION' && mode?.map === chosen)}
              onClick={() => setMode.mutate({ mode: 'NAVIGATION', map: chosen, vision })}
            >
              이 지도로 주행
            </Button>
          </Stack>
          <FormControlLabel control={<Switch size="small" checked={vision} onChange={(e) => setVision(e.target.checked)} />} label="Vision AI(물체 검출) 함께 켜기" />
        </Stack>

        {/* 지도 저장 — 지도 만들기 중에만 */}
        {current === 'MAPPING' && mode?.phase === 'running' && (
          <Stack direction="row" spacing={1} sx={{ mt: 1.5 }}>
            <TextField size="small" label="지도 이름 (영문·숫자·_·-)" value={name} onChange={(e) => setName(e.target.value)} sx={{ flex: 1 }} />
            <Button variant="contained" disabled={saveMap.isPending || !name} onClick={() => saveMap.mutate(name, { onSuccess: () => setSelected(name) })}>
              {saveMap.isPending ? '저장 중…' : '지도 저장'}
            </Button>
          </Stack>
        )}
        {saveMap.isSuccess && <Typography variant="caption" color="success.main">「{saveMap.data?.name}」 저장됨</Typography>}

        {/* 저장 지도 목록 */}
        {mapList.length > 0 && (
          <Box sx={{ mt: 1.5, display: 'grid', gap: 1, gridTemplateColumns: 'repeat(auto-fill, minmax(120px, 1fr))' }}>
            {mapList.map((m) => (
              <Box key={m.name} sx={{ position: 'relative', border: 1, borderColor: m.name === chosen ? 'primary.main' : 'divider', borderRadius: 1, p: 0.5, cursor: 'pointer' }} onClick={() => setSelected(m.name)}>
                <Box component="img" src={`/api/maps/${m.name}.png?t=${m.saved}`} alt={m.name} sx={{ width: '100%', height: 80, objectFit: 'contain', imageRendering: 'pixelated', bgcolor: '#0b0d10' }} />
                <Typography variant="caption" sx={{ display: 'block', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                  {m.name}
                </Typography>
                <IconButton size="small" sx={{ position: 'absolute', top: 2, right: 2 }} onClick={(e) => (e.stopPropagation(), setConfirmDelete(m.name))} aria-label="지도 삭제">
                  <DeleteOutlineIcon fontSize="small" />
                </IconButton>
              </Box>
            ))}
          </Box>
        )}

        {error && (
          <Alert severity="error" sx={{ mt: 1 }} onClose={() => (setMode.reset(), saveMap.reset(), deleteMap.reset())}>
            {error.message}
          </Alert>
        )}

        <Button size="small" sx={{ mt: 1 }} onClick={() => setShowLog(!showLog)}>
          {showLog ? '로그 닫기' : '실행 로그 보기'}
        </Button>
        <Collapse in={showLog || mode?.phase === 'error'}>
          <Box component="pre" sx={{ m: 0, p: 1, maxHeight: 180, overflow: 'auto', fontSize: 11, bgcolor: '#0b0d10', borderRadius: 1, whiteSpace: 'pre-wrap' }}>
            {(log.data ?? []).join('\n') || '로그 없음'}
          </Box>
        </Collapse>
      </CardContent>

      <Dialog open={!!confirmDelete} onClose={() => setConfirmDelete(null)}>
        <DialogTitle>지도 삭제</DialogTitle>
        <DialogContent>「{confirmDelete}」 지도 파일을 지웁니다. 되돌릴 수 없습니다.</DialogContent>
        <DialogActions>
          <Button onClick={() => setConfirmDelete(null)}>취소</Button>
          <Button color="error" onClick={() => (deleteMap.mutate(confirmDelete!), setConfirmDelete(null), selected === confirmDelete && setSelected(''))}>
            삭제
          </Button>
        </DialogActions>
      </Dialog>
    </Card>
  )
}
