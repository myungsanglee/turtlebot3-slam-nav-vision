import CenterFocusStrongIcon from '@mui/icons-material/CenterFocusStrong'
import FitScreenIcon from '@mui/icons-material/FitScreen'
import { Alert, Box, Button, Card, CardContent, Chip, Stack, ToggleButton, ToggleButtonGroup, Tooltip, Typography } from '@mui/material'
import { useRef, useState } from 'react'
import { useInitialPose, useNavGoal } from '../api/nav'
import type { Pose, RobotStream } from '../api/types'
import MapView, { type Layers, type MapHandle, type Tool } from './MapView'

const TOOL_HINT: Record<Tool, string> = {
  view: '끌어서 이동 · 휠로 확대',
  goal: '목표 위치를 누르고 바라볼 방향으로 끌었다 놓기 → 확인 후 이동',
  initial: '로봇의 실제 위치를 누르고 바라보는 방향으로 끌기 (저장 지도 주행 시 AMCL 초기화)',
}

const deg = (r: number) => Math.round((r * 180) / Math.PI)

export default function MapPanel({ robot }: { robot: RobotStream }) {
  const [tool, setTool] = useState<Tool>('view')
  const [layers, setLayers] = useState<Layers>({ costmap: true, scan: true, paths: true })
  const [follow, setFollow] = useState(false)
  const mapRef = useRef<MapHandle>(null)
  const [pending, setPending] = useState<Pose | null>(null)
  const goal = useNavGoal()
  const initial = useInitialPose()

  const nav = robot.status?.navigation
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
          <ToggleButtonGroup size="small" exclusive value={tool} onChange={(_, v) => v && (setTool(v), setPending(null))}>
            <ToggleButton value="view">보기</ToggleButton>
            <ToggleButton value="goal" disabled={!nav?.nav2_ready}>
              목표 지정
            </ToggleButton>
            <ToggleButton value="initial" disabled={!robot.map || !nav?.nav2_ready}>
              초기 위치
            </ToggleButton>
          </ToggleButtonGroup>
          <ToggleButtonGroup
            size="small"
            value={layerValue}
            onChange={(_, v: string[]) => setLayers({ costmap: v.includes('costmap'), scan: v.includes('scan'), paths: v.includes('paths') })}
          >
            <ToggleButton value="costmap">장애물</ToggleButton>
            <ToggleButton value="scan">라이다</ToggleButton>
            <ToggleButton value="paths">경로</ToggleButton>
          </ToggleButtonGroup>
          <Tooltip title="로봇 따라가기">
            <ToggleButton size="small" value="follow" selected={follow} onChange={() => setFollow(!follow)} disabled={!robot.pose}>
              <CenterFocusStrongIcon fontSize="small" />
            </ToggleButton>
          </Tooltip>
          <Tooltip title="지도 전체 보기">
            <span>
              <ToggleButton size="small" value="fit" onClick={() => (setFollow(false), mapRef.current?.fit())} disabled={!robot.map}>
                <FitScreenIcon fontSize="small" />
              </ToggleButton>
            </span>
          </Tooltip>
        </Stack>

        <Box sx={{ position: 'relative' }}>
          <MapView
            frame={robot.pose?.frame ?? null}
            mapFrame={nav?.fixed_frame ?? 'map'}
            map={robot.map}
            costmap={robot.costmap}
            pose={robot.pose}
            scan={robot.scan}
            globalPath={active ? robot.paths.global : []}
            localPath={active ? robot.paths.local : []}
            footprint={robot.footprint}
            goal={active ? (robot.nav?.goal ?? null) : null}
            pending={pending}
            tool={tool}
            layers={layers}
            follow={follow}
            ref={mapRef}
            onPick={setPending}
          />
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
          {TOOL_HINT[tool]}
          {!nav?.nav2_ready && tool === 'view' && ' · 목표 지정은 Nav2 실행 중일 때 가능'}
        </Typography>
        {(goal.error || initial.error) && (
          <Alert severity="error" sx={{ mt: 1 }} onClose={() => (goal.reset(), initial.reset())}>
            {(goal.error ?? initial.error)?.message}
          </Alert>
        )}
      </CardContent>
    </Card>
  )
}
