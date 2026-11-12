import CenterFocusStrongIcon from '@mui/icons-material/CenterFocusStrong'
import FitScreenIcon from '@mui/icons-material/FitScreen'
import { Alert, Box, Button, Card, CardContent, Chip, Stack, ToggleButton, ToggleButtonGroup, Tooltip, Typography } from '@mui/material'
import { lazy, Suspense, useRef, useState } from 'react'
import { useInitialPose, useNavGoal } from '../api/nav'
import { useRobotModel } from '../api/robot'
import type { Pose, RobotStream } from '../api/types'
import { useZones } from '../api/zones'
import MapView, { type Layers, type MapHandle, type Tool } from './MapView'
import { type Draft, ZoneDraftBar, ZoneList } from './ZoneEditor'

// three.js(~700KB)는 「3D」 를 누를 때만 내려받는다 — 2D 만 쓰는 폰에서 첫 로딩이 늘지 않게
const Map3D = lazy(() => import('./Map3D'))

const TOOL_HINT: Record<Tool, string> = {
  view: '끌어서 이동 · 휠로 확대',
  goal: '목표 위치를 누르고 바라볼 방향으로 끌었다 놓기 → 확인 후 이동',
  initial: '로봇의 실제 위치를 누르고 바라보는 방향으로 끌기 (저장 지도 주행 시 AMCL 초기화)',
  zone: '거울·유리 앞(10cm 쯤 떨어져)에 점을 차례로 누르기 → 「완료」. 가상 벽 = 선, 금지 구역 = 닫힌 영역',
}

const deg = (r: number) => Math.round((r * 180) / Math.PI)

export default function MapPanel({ robot }: { robot: RobotStream }) {
  const [view, setView] = useState<'2d' | '3d'>('2d')
  const [tool, setTool] = useState<Tool>('view')
  const [layers, setLayers] = useState<Layers>({ costmap: true, scan: true, depth: true, paths: true })
  const [follow, setFollow] = useState(false)
  const mapRef = useRef<MapHandle>(null)
  const [pending, setPending] = useState<Pose | null>(null)
  const goal = useNavGoal()
  const robotModel = useRobotModel(robot.connectionId)
  const initial = useInitialPose()
  // 금지 구역: 저장 지도로 주행 중일 때 그 지도의 구역 (SLAM 중엔 지도 원점이 매번 달라 편집 불가)
  const zoneMap = robot.mode?.mode === 'NAVIGATION' ? robot.mode.map : null
  const zones = useZones(zoneMap)
  const [draft, setDraft] = useState<Draft>({ kind: 'wall', points: [] })

  const nav = robot.status?.navigation
  // Nav2 가 꺼지면 costmap·footprint 가 더 오지 않는다 — 마지막 값이 옛 자리에 남지 않게 3초 넘게 안 오면 숨긴다
  const fresh = (age: number | null | undefined) => age != null && age < 3
  const costmap = fresh(nav?.costmap_age) ? robot.costmap : null
  const footprint = fresh(nav?.footprint_age) ? robot.footprint : []
  const active = robot.nav && (robot.nav.state === 'active' || robot.nav.state === 'sending')

  const confirm = () => {
    if (!pending) return
    const m = tool === 'initial' ? initial : goal
    m.mutate(pending, { onSettled: () => setPending(null) })
    setTool('view')
  }

  const layerValue = (Object.keys(layers) as (keyof Layers)[]).filter((k) => layers[k])

  return (
    <Card>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'center', gap: 1, mb: 1.5, flexWrap: 'wrap' }}>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, mr: 'auto' }}>
            지도
          </Typography>
          {nav?.display_frame === 'odom' && <Chip size="small" variant="outlined" label="출발점(odom) 기준" />}
          {nav && nav.display_frame === null && <Chip size="small" color="warning" label="로봇 위치 없음" />}
          <ToggleButtonGroup size="small" exclusive value={view} onChange={(_, v) => v && (setView(v), setTool('view'), setPending(null))}>
            <ToggleButton value="2d">2D</ToggleButton>
            <ToggleButton value="3d">3D</ToggleButton>
          </ToggleButtonGroup>
          <ToggleButtonGroup
            size="small"
            exclusive
            value={tool}
            onChange={(_, v) => v && (setTool(v), setPending(null), setDraft({ ...draft, points: [] }))}
          >
            <ToggleButton value="view">보기</ToggleButton>
            <ToggleButton value="goal" disabled={!nav?.nav2_ready || view === '3d'}>
              목표 지정
            </ToggleButton>
            <ToggleButton value="initial" disabled={!robot.map || !nav?.nav2_ready || view === '3d'}>
              초기 위치
            </ToggleButton>
            <ToggleButton value="zone" disabled={!zoneMap || view === '3d' || robot.pose?.frame !== nav?.fixed_frame}>
              금지 구역
            </ToggleButton>
          </ToggleButtonGroup>
          <ToggleButtonGroup
            size="small"
            value={layerValue}
            onChange={(_, v: string[]) => setLayers({ costmap: v.includes('costmap'), scan: v.includes('scan'), depth: v.includes('depth'), paths: v.includes('paths') })}
          >
            <ToggleButton value="costmap">장애물</ToggleButton>
            <ToggleButton value="scan">라이다</ToggleButton>
            <Tooltip title="카메라 depth 점 — 주황: 장애물로 피하는 높이 / 파랑: 바닥 / 보라: 로봇보다 높음">
              <ToggleButton value="depth">카메라</ToggleButton>
            </Tooltip>
            <ToggleButton value="paths">경로</ToggleButton>
          </ToggleButtonGroup>
          <Tooltip title="로봇 따라가기">
            <ToggleButton size="small" value="follow" selected={follow} onChange={() => setFollow(!follow)} disabled={!robot.pose}>
              <CenterFocusStrongIcon fontSize="small" />
            </ToggleButton>
          </Tooltip>
          <Tooltip title="지도 전체 보기">
            <span>
              <ToggleButton size="small" value="fit" onClick={() => (setFollow(false), mapRef.current?.fit())} disabled={!robot.map || view === '3d'}>
                <FitScreenIcon fontSize="small" />
              </ToggleButton>
            </span>
          </Tooltip>
        </Stack>

        <Box sx={{ position: 'relative' }}>
          {view === '3d' ? (
            <Box sx={{ height: { xs: 420, md: '68vh' }, bgcolor: '#0b0d10', borderRadius: 1 }}>
              <Suspense fallback={<Typography color="text.secondary" sx={{ p: 2 }}>3D 화면 불러오는 중…</Typography>}>
                <Map3D
                  frame={robot.pose?.frame ?? null}
                  mapFrame={nav?.fixed_frame ?? 'map'}
                  map={robot.map}
                  costmap={costmap}
                  pose={robot.pose}
                  scan={robot.scan}
                  depth={robot.depth}
                  globalPath={active ? robot.paths.global : []}
                  localPath={active ? robot.paths.local : []}
                  footprint={footprint}
                  robotModel={robotModel.data ?? null}
                  goal={active ? (robot.nav?.goal ?? null) : null}
                  zones={zoneMap ? (zones.data ?? []) : []}
                  layers={layers}
                  follow={follow}
                />
              </Suspense>
            </Box>
          ) : (
          <MapView
            frame={robot.pose?.frame ?? null}
            mapFrame={nav?.fixed_frame ?? 'map'}
            map={robot.map}
            costmap={costmap}
            pose={robot.pose}
            scan={robot.scan}
            depth={robot.depth}
            globalPath={active ? robot.paths.global : []}
            localPath={active ? robot.paths.local : []}
            footprint={footprint}
            robotModel={robotModel.data ?? null}
            goal={active ? (robot.nav?.goal ?? null) : null}
            pending={pending}
            tool={tool}
            layers={layers}
            follow={follow}
            ref={mapRef}
            zones={zoneMap ? (zones.data ?? []) : []}
            draft={tool === 'zone' ? draft : null}
            onPick={(q) => (tool === 'zone' ? setDraft({ ...draft, points: [...draft.points, [q.x, q.y]] }) : setPending(q))}
          />
          )}
          {tool === 'zone' && zoneMap && view === '2d' && (
            <ZoneDraftBar map={zoneMap} zones={zones.data ?? []} draft={draft} setDraft={setDraft} onClose={() => (setTool('view'), setDraft({ ...draft, points: [] }))} />
          )}
          {pending && (
            <Alert
              severity={tool === 'initial' ? 'info' : 'warning'}
              sx={{ position: 'absolute', top: 8, left: 8, right: 8 }}
              action={
                <Stack direction="row" spacing={1}>
                  <Button color="inherit" size="small" variant="outlined" onClick={confirm}>
                    {tool === 'initial' ? '설정' : '이동'}
                  </Button>
                  <Button color="inherit" size="small" onClick={() => setPending(null)}>
                    취소
                  </Button>
                </Stack>
              }
            >
              {tool === 'initial' ? '초기 위치' : '목표'} ({pending.x.toFixed(2)}, {pending.y.toFixed(2)}) m · {deg(pending.yaw)}°
              {tool === 'goal' && ' — 로봇이 실제로 움직인다'}
            </Alert>
          )}
        </Box>

        <Typography variant="caption" color="text.secondary" sx={{ display: 'block', mt: 1 }}>
          {view === '3d' ? '왼쪽 끌기: 회전 · 오른쪽 끌기: 이동 · 휠: 확대 (3D 는 보기 전용 — 목표 지정은 2D 에서)' : TOOL_HINT[tool]}
          {!nav?.nav2_ready && tool === 'view' && ' · 목표 지정은 Nav2 실행 중일 때 가능'}
          {!zoneMap && tool === 'view' && ' · 금지 구역은 「저장 지도로 주행」 중에 그린다'}
        </Typography>
        {zoneMap && <ZoneList map={zoneMap} zones={zones.data ?? []} applied={nav?.keepout?.map === zoneMap} />}
        {(goal.error || initial.error) && (
          <Alert severity="error" sx={{ mt: 1 }} onClose={() => (goal.reset(), initial.reset())}>
            {(goal.error ?? initial.error)?.message}
          </Alert>
        )}
      </CardContent>
    </Card>
  )
}
