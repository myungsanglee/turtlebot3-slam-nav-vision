import CenterFocusStrongIcon from '@mui/icons-material/CenterFocusStrong'
import CloseIcon from '@mui/icons-material/Close'
import FitScreenIcon from '@mui/icons-material/FitScreen'
import SportsEsportsIcon from '@mui/icons-material/SportsEsports'
import SwapHorizIcon from '@mui/icons-material/SwapHoriz'
import VideocamIcon from '@mui/icons-material/Videocam'
import { Alert, Box, Button, Card, Chip, IconButton, Stack, ToggleButton, ToggleButtonGroup, Tooltip, Typography, type SxProps, type Theme } from '@mui/material'
import { lazy, Suspense, useRef, useState } from 'react'
import { useInitialPose, useNavGoal } from '../api/nav'
import { useRobotModel } from '../api/robot'
import type { Pose, RobotStream } from '../api/types'
import { useZones } from '../api/zones'
import CameraView from './CameraView'
import Joystick from './Joystick'
import { LocalizationAlert, LocalizationControl } from './LocalizationBar'
import MapView, { type Layers, type MapHandle, type Tool } from './MapView'
import { NavOverlay } from './NavPanel'
import { type Draft, ZoneDraftBar, ZoneList } from './ZoneEditor'

// ───────────────────────────────────────────────────────────────────────────
// 조작 화면 — 지도를 크게, 카메라는 모서리 작은 창(⇄ 로 맞바꿈), 조이스틱·주행 상태는 화면 위에 겹친다.
//   지도 만들기·조이스틱 이동 중 시선이 한 곳에 머물도록 (Formant·로보락 앱·Spot 의 "큰 화면 + 작은 창" 배치).
//   두 영역은 언제나 같은 컴포넌트를 유지하고 위치·크기만 바꾼다 → 맞바꿔도 영상 스트림·지도 확대 상태가 끊기지 않는다.
// ───────────────────────────────────────────────────────────────────────────

// three.js(~700KB)는 「3D」 를 누를 때만 내려받는다 — 2D 만 쓰는 폰에서 첫 로딩이 늘지 않게
const Map3D = lazy(() => import('./Map3D'))

const TOOL_HINT: Record<Tool, string> = {
  view: '끌어서 이동 · 휠로 확대',
  goal: '목표 위치를 누르고 바라볼 방향으로 끌었다 놓기 → 확인 후 이동',
  initial: '로봇의 실제 위치를 누르고 바라보는 방향으로 끌기 — 자동 위치 찾기가 실패했을 때만 (저장 지도 주행)',
  zone: '거울·유리 앞(10cm 쯤 떨어져)에 점을 차례로 누르기 → 「완료」. 가상 벽 = 선, 금지 구역 = 닫힌 영역',
}

// 작은 창 크기 (폭) — 겹치는 안내 막대가 작은 창을 가리지 않도록 오른쪽 여백도 여기서 계산
const PIP_W = { xs: '42%', sm: 260, md: 320 }
const BESIDE_PIP = { xs: 'calc(42% + 16px)', sm: 276, md: 336 }

const deg = (r: number) => Math.round((r * 180) / Math.PI)
const POWER_TEXT: Record<string, string> = { standby: '대기', off: 'Pi 꺼짐', unreachable: 'Pi 연결 안 됨', rebooting: '재부팅 중', stopping: '정지 중', starting: '시작 중' }

export default function MapPanel({ robot }: { robot: RobotStream }) {
  const [view, setView] = useState<'2d' | '3d'>('2d')
  const [tool, setTool] = useState<Tool>('view')
  const [layers, setLayers] = useState<Layers>({ costmap: true, scan: true, depth: true, paths: true })
  const [follow, setFollow] = useState(false)
  const [main, setMain] = useState<'map' | 'camera'>('map') // 큰 화면에 무엇을
  const [showCam, setShowCam] = useState(true) // 작은 창(카메라 또는 지도) 보이기
  const [showStick, setShowStick] = useState(true)
  const mapRef = useRef<MapHandle>(null)
  const [pending, setPending] = useState<Pose | null>(null)
  const goal = useNavGoal()
  const robotModel = useRobotModel(robot.connectionId)
  const initial = useInitialPose()
  // 금지 구역: 저장 지도로 주행 중일 때 그 지도의 구역 (SLAM 중엔 지도 원점이 매번 달라 편집 불가)
  const zoneMap = robot.mode?.mode === 'NAVIGATION' ? robot.mode.map : null
  const zones = useZones(zoneMap)
  const [draft, setDraft] = useState<Draft>({ kind: 'wall', points: [] })

  const status = robot.status
  const nav = status?.navigation
  // Nav2 가 꺼지면 costmap·footprint 가 더 오지 않는다 — 마지막 값이 옛 자리에 남지 않게 3초 넘게 안 오면 숨긴다
  const fresh = (age: number | null | undefined) => age != null && age < 3
  // 로봇 데이터가 끊기면(대기·Pi 꺼짐·통신 끊김) 로봇·라이다·카메라 점·장애물 지도·경로를 지운다 — 옛 값이 지금처럼 보이지 않게
  const robotLive = !!status?.robot_connected
  const costmap = robotLive && fresh(nav?.costmap_age) ? robot.costmap : null
  const footprint = robotLive && fresh(nav?.footprint_age) ? robot.footprint : []
  const pose = robotLive ? robot.pose : null
  const scan = robotLive ? robot.scan : []
  const depth = robotLive ? robot.depth : { points: [], band: robot.depth.band }
  const active = robotLive && robot.nav && (robot.nav.state === 'active' || robot.nav.state === 'sending')
  // 대기 중 기본 지도: 로봇 위치를 몰라도 지도는 보여 준다 (위치는 서버가 스캔 매칭으로 추정 — idle_localization)
  const defaultMap = robot.map?.source === 'default'
  const idleLoc = nav?.idle_localization
  const notice = !robotLive
    ? `로봇 데이터 없음 (${POWER_TEXT[status?.power?.state ?? ''] ?? '수신 끊김'}) — 로봇·센서 표시를 지웠습니다`
    : defaultMap && idleLoc?.state !== 'localized'
      ? idleLoc?.state === 'searching' ? '기본 지도에서 로봇 위치를 찾는 중…' : '기본 지도에서 로봇 위치를 확신하지 못함 — 「이 지도로 주행」 시 다시 찾습니다'
      : undefined
  // 자동 위치 찾기: 저장 지도 주행 중에만 (SLAM 모드는 slam_toolbox 가 위치를 스스로 안다)
  const loc = robot.mode?.mode === 'NAVIGATION' ? (robot.nav?.localization ?? null) : null
  const camMain = main === 'camera' && showCam
  const pipOn = showCam // 작은 창이 떠 있나 (카메라가 크면 지도가 작은 창)
  const overlayRight = pipOn ? BESIDE_PIP : 8
  const mapPicking = !camMain && view === '2d'

  const confirm = () => {
    if (!pending) return
    const m = tool === 'initial' ? initial : goal
    m.mutate(pending, { onSettled: () => setPending(null) })
    setTool('view')
  }
  const swap = () => {
    setMain(camMain ? 'map' : 'camera')
    setTool('view')
    setPending(null)
  }

  const layerValue = (Object.keys(layers) as (keyof Layers)[]).filter((k) => layers[k])
  const slot = (isMain: boolean): SxProps<Theme> =>
    isMain
      ? { position: 'absolute', inset: 0, zIndex: 0 }
      : {
          position: 'absolute', top: 8, right: 8, width: PIP_W, aspectRatio: '4 / 3', zIndex: 2, borderRadius: 1, overflow: 'hidden',
          boxShadow: 8, border: '1px solid', borderColor: 'divider', bgcolor: '#0b0d10',
        }

  const mapContent =
    view === '3d' ? (
      <Box sx={{ width: '100%', height: '100%', bgcolor: '#0b0d10' }}>
        <Suspense fallback={<Typography color="text.secondary" sx={{ p: 2 }}>3D 화면 불러오는 중…</Typography>}>
          <Map3D
            frame={pose?.frame ?? null}
            mapFrame={nav?.fixed_frame ?? 'map'}
            map={robot.map}
            costmap={costmap}
            pose={pose}
            scan={scan}
            depth={depth}
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
        fill
        compact={camMain}
        mapAlways={defaultMap}
        notice={notice}
        frame={pose?.frame ?? null}
        mapFrame={nav?.fixed_frame ?? 'map'}
        map={robot.map}
        costmap={costmap}
        pose={pose}
        scan={scan}
        depth={depth}
        globalPath={active ? robot.paths.global : []}
        localPath={active ? robot.paths.local : []}
        footprint={footprint}
        robotModel={robotModel.data ?? null}
        goal={active ? (robot.nav?.goal ?? null) : null}
        pending={pending}
        tool={camMain ? 'view' : tool}
        layers={layers}
        follow={(follow || camMain) && !!pose} // 작은 창일 땐 로봇을 따라가게 (주변이 항상 보이게)
        ref={mapRef}
        zones={zoneMap ? (zones.data ?? []) : []}
        draft={tool === 'zone' ? draft : null}
        onPick={(q) => (tool === 'zone' ? setDraft({ ...draft, points: [...draft.points, [q.x, q.y]] }) : setPending(q))}
      />
    )

  return (
    <Card sx={{ display: 'flex', flexDirection: 'column', height: { xs: 'calc(100svh - 76px)', lg: 'calc(100vh - 96px)' }, minHeight: 460 }}>
      {/* 도구 막대 */}
      {/* 폰: 한 줄 가로 스크롤 (지도 공간을 줄이지 않게) / 넓은 화면: 줄바꿈 */}
      <Stack
        direction="row"
        sx={{
          alignItems: 'center', gap: 1, px: 1.5, py: 1, borderBottom: 1, borderColor: 'divider',
          flexWrap: { xs: 'nowrap', md: 'wrap' }, overflowX: { xs: 'auto', md: 'visible' }, '& > *': { flexShrink: 0 },
        }}
      >
        <Typography variant="subtitle1" sx={{ fontWeight: 600, mr: 'auto' }}>
          조작
        </Typography>
        {defaultMap && (
          <Tooltip title="대기 중 표시하는 기본 지도 (운용 모드의 ★). 로봇 위치는 라이다를 지도에 맞춰 추정 — 화면 표시용">
            <Chip size="small" variant="outlined" label={`기본 지도: ${robot.map?.name}`} />
          </Tooltip>
        )}
        {defaultMap && robotLive && idleLoc?.state === 'localized' && (
          <Chip size="small" color="success" variant="outlined" label={`위치 추정 ${idleLoc.score?.toFixed(2) ?? ''}`} />
        )}
        {!robotLive && <Chip size="small" color="warning" label="로봇 데이터 없음" />}
        {robotLive && !defaultMap && nav?.display_frame === 'odom' && <Chip size="small" variant="outlined" label="출발점(odom) 기준" />}
        {robotLive && nav && nav.display_frame === null && <Chip size="small" color="warning" label="로봇 위치 없음" />}
        {robot.mode?.mode === 'NAVIGATION' && <LocalizationControl loc={loc} />}
        <ToggleButtonGroup
          size="small"
          exclusive
          value={tool}
          disabled={!mapPicking}
          onChange={(_, v) => v && (setTool(v), setPending(null), setDraft({ ...draft, points: [] }))}
        >
          <ToggleButton value="view">보기</ToggleButton>
          <ToggleButton value="goal" disabled={!nav?.nav2_ready}>
            목표 지정
          </ToggleButton>
          <ToggleButton value="initial" disabled={!robot.map || !nav?.nav2_ready}>
            초기 위치
          </ToggleButton>
          <ToggleButton value="zone" disabled={!zoneMap || robot.pose?.frame !== nav?.fixed_frame}>
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
            <ToggleButton value="depth">depth</ToggleButton>
          </Tooltip>
          <ToggleButton value="paths">경로</ToggleButton>
        </ToggleButtonGroup>
        <ToggleButtonGroup size="small" exclusive value={view} onChange={(_, v) => v && (setView(v), setTool('view'), setPending(null))}>
          <ToggleButton value="2d">2D</ToggleButton>
          <ToggleButton value="3d">3D</ToggleButton>
        </ToggleButtonGroup>
        <Tooltip title="로봇 따라가기">
          <ToggleButton size="small" value="follow" selected={follow} onChange={() => setFollow(!follow)} disabled={!pose}>
            <CenterFocusStrongIcon fontSize="small" />
          </ToggleButton>
        </Tooltip>
        <Tooltip title="지도 전체 보기">
          <span>
            <ToggleButton size="small" value="fit" onClick={() => (setFollow(false), mapRef.current?.fit())} disabled={!robot.map || view === '3d' || camMain}>
              <FitScreenIcon fontSize="small" />
            </ToggleButton>
          </span>
        </Tooltip>
        <Tooltip title={showCam ? '작은 창 숨기기' : '카메라 작은 창 보이기'}>
          <ToggleButton size="small" value="cam" selected={showCam} onChange={() => (setShowCam(!showCam), setMain('map'))}>
            <VideocamIcon fontSize="small" />
          </ToggleButton>
        </Tooltip>
        <Tooltip title={showStick ? '조이스틱 숨기기 (키보드 방향키는 계속 동작)' : '조이스틱 보이기'}>
          <ToggleButton size="small" value="stick" selected={showStick} onChange={() => setShowStick(!showStick)}>
            <SportsEsportsIcon fontSize="small" />
          </ToggleButton>
        </Tooltip>
      </Stack>

      <Box sx={{ px: 1, pt: loc && ['ambiguous', 'failed', 'lost'].includes(loc.state) ? 1 : 0 }}>
        <LocalizationAlert loc={loc} onPick={(p) => (setMain('map'), setView('2d'), setTool('initial'), setPending(p))} />
      </Box>

      {/* 큰 화면 + 작은 창 + 겹치는 조작 */}
      <Box sx={{ position: 'relative', flex: 1, minHeight: 0, overflow: 'hidden' }}>
        <Box sx={slot(!camMain)}>{mapContent}</Box>
        {showCam && (
          <Box sx={slot(camMain)}>
            <CameraView topics={status?.topics ?? []} connectionId={robot.connectionId} compact={!camMain} />
          </Box>
        )}
        {/* 작은 창 단추: 맞바꾸기 · 닫기 */}
        {pipOn && (
          <Stack direction="row" sx={{ position: 'absolute', top: 12, right: 12, zIndex: 3, gap: 0.5 }}>
            <Tooltip title={camMain ? '지도를 크게' : '카메라를 크게'}>
              <IconButton size="small" onClick={swap} sx={{ bgcolor: 'rgba(0,0,0,0.6)', '&:hover': { bgcolor: 'rgba(0,0,0,0.8)' } }}>
                <SwapHorizIcon fontSize="small" />
              </IconButton>
            </Tooltip>
            <Tooltip title="작은 창 닫기">
              <IconButton size="small" onClick={() => (setShowCam(false), setMain('map'))} sx={{ bgcolor: 'rgba(0,0,0,0.6)', '&:hover': { bgcolor: 'rgba(0,0,0,0.8)' } }}>
                <CloseIcon fontSize="small" />
              </IconButton>
            </Tooltip>
          </Stack>
        )}

        {tool === 'zone' && zoneMap && mapPicking && (
          <ZoneDraftBar
            map={zoneMap}
            zones={zones.data ?? []}
            draft={draft}
            setDraft={setDraft}
            right={overlayRight}
            onClose={() => (setTool('view'), setDraft({ ...draft, points: [] }))}
          />
        )}
        {pending && (
          <Alert
            severity={tool === 'initial' ? 'info' : 'warning'}
            sx={{ position: 'absolute', top: 8, left: 8, right: overlayRight, zIndex: 3 }}
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

        {/* 아래쪽: 주행 상태(왼쪽, 축척 막대 위) · 조이스틱(오른쪽) */}
        <Box sx={{ position: 'absolute', left: 8, bottom: 44, zIndex: 3, maxWidth: 'calc(100% - 200px)' }}>
          <NavOverlay nav={robot.nav} />
        </Box>
        {showStick && (
          <Box sx={{ position: 'absolute', right: 8, bottom: 8, zIndex: 3 }}>
            <Joystick send={robot.send} connected={robot.connected && !!status?.robot_connected} />
          </Box>
        )}
      </Box>

      {/* 안내·금지 구역 목록·오류 */}
      <Box sx={{ px: 1.5, py: 0.75, borderTop: 1, borderColor: 'divider' }}>
        <Typography variant="caption" color="text.secondary" sx={{ display: 'block' }}>
          {camMain
            ? '카메라를 크게 보는 중 — 지도 도구는 ⇄ 로 지도를 크게 했을 때'
            : view === '3d'
              ? '왼쪽 끌기: 회전 · 오른쪽 끌기: 이동 · 휠: 확대 (3D 는 보기 전용 — 목표 지정은 2D 에서)'
              : TOOL_HINT[tool]}
          {' · 조이스틱 또는 키보드 방향키(WASD)로 수동 이동'}
          {!nav?.nav2_ready && tool === 'view' && ' · 목표 지정은 Nav2 실행 중일 때 가능'}
        </Typography>
        {zoneMap && <ZoneList map={zoneMap} zones={zones.data ?? []} applied={nav?.keepout?.map === zoneMap} />}
        {(goal.error || initial.error) && (
          <Alert severity="error" sx={{ mt: 1 }} onClose={() => (goal.reset(), initial.reset())}>
            {(goal.error ?? initial.error)?.message}
          </Alert>
        )}
      </Box>
    </Card>
  )
}
