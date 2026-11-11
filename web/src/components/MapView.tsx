import { Box, Typography } from '@mui/material'
import type Konva from 'konva'
import { useEffect, useImperativeHandle, useLayoutEffect, useRef, useState, type Ref } from 'react'
import { Arrow, Group, Image as KImage, Layer, Line, Shape, Stage } from 'react-konva'
import type { GridMeta, Point, Pose } from '../api/types'

// ───────────────────────────────────────────────────────────────────────────
// 좌표 처리 원리
//   Stage(화면)는 위치 (x, y)와 배율 s(픽셀/미터)만 가진다.
//   그 안의 world 그룹을 scaleY = -1 로 뒤집어 "y 가 위로 커지는" 지도 좌표계를 만든다.
//   → 모든 레이어를 **지도 좌표(미터) 그대로** 그린다. 화면 좌표 계산 코드가 없다.
//   선 굵기·점 크기는 확대해도 같은 픽셀로 보이게 strokeScaleEnabled=false / 반지름 = 픽셀 / s.
// ───────────────────────────────────────────────────────────────────────────

export type Tool = 'view' | 'goal' | 'initial'

export interface Layers {
  costmap: boolean
  scan: boolean
  paths: boolean
}

interface Props {
  frame: string | null // 표시 좌표계 (로봇 위치의 frame). 'map' 이 아니면 지도 이미지를 숨긴다
  mapFrame: string // 지도 좌표계 이름 (보통 'map')
  map: GridMeta | null
  costmap: GridMeta | null
  pose: Pose | null
  scan: Point[]
  globalPath: Point[]
  localPath: Point[]
  footprint: Point[]
  goal: Pose | null // 진행 중인 목표
  pending: Pose | null // 사용자가 찍었지만 아직 보내지 않은 목표
  tool: Tool
  layers: Layers
  follow: boolean
  onPick: (p: Pose) => void
  ref?: Ref<MapHandle>
}

export interface MapHandle {
  fit: () => void // 지도 전체가 보이게 (버튼에서 호출)
}

const DEG = 180 / Math.PI

function useImage(src: string | null) {
  const [loaded, setLoaded] = useState<HTMLImageElement | null>(null)
  useEffect(() => {
    if (!src) return
    const im = new window.Image()
    im.onload = () => setLoaded(im) // 새 이미지가 다 받아진 뒤에 바꿔 끼워 깜빡임이 없다 (그동안 이전 이미지 유지)
    im.src = src
  }, [src])
  return src ? loaded : null
}

// OccupancyGrid 이미지를 지도 좌표에 깐다. 서버 PNG 는 위쪽 = 지도 +y 로 이미 뒤집혀 있으므로,
// 뒤집힌(world) 좌표계에서는 다시 scaleY=-1 로 그려 위아래를 맞춘다.
function Grid({ meta, img, opacity = 1 }: { meta: GridMeta; img: HTMLImageElement | null; opacity?: number }) {
  if (!img) return null
  const w = meta.width * meta.resolution
  const h = meta.height * meta.resolution
  return (
    <Group x={meta.origin.x} y={meta.origin.y} rotation={meta.origin.yaw * DEG}>
      <KImage image={img} x={0} y={h} width={w} height={h} scaleY={-1} opacity={opacity} listening={false} />
    </Group>
  )
}

function flat(points: Point[]) {
  return points.flat()
}

export default function MapView({ ref, ...p }: Props) {
  const wrapRef = useRef<HTMLDivElement>(null)
  const worldRef = useRef<Konva.Group>(null)
  const [size, setSize] = useState({ w: 600, h: 450 })
  const [view, setView] = useState({ x: 300, y: 225, s: 60 }) // Stage 위치·배율 (px/m)
  const [fitted, setFitted] = useState<'map' | 'robot' | null>(null)
  const [drag, setDrag] = useState<{ from: Point; to: Point } | null>(null)

  const mapImg = useImage(p.map ? `/api/map.png?v=${p.map.version}` : null)
  const costImg = useImage(p.costmap ? `/api/costmap.png?v=${p.costmap.version}` : null)

  // 컨테이너 크기 따라가기 (반응형)
  useLayoutEffect(() => {
    const el = wrapRef.current
    if (!el) return
    const ro = new ResizeObserver(() => {
      const w = el.clientWidth
      setSize({ w, h: Math.round(Math.max(340, Math.min(window.innerHeight * 0.68, w * 0.8))) })
    })
    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  // 지도 좌표 (wx, wy) 가 화면 가운데 오는 Stage 위치 (world 그룹이 y 를 뒤집으므로 y 는 +)
  const centered = (wx: number, wy: number, s: number) => ({ x: size.w / 2 - wx * s, y: size.h / 2 + wy * s, s })
  const fitView = () => {
    const m = p.map!
    const w = m.width * m.resolution
    const h = m.height * m.resolution
    return centered(m.origin.x + w / 2, m.origin.y + h / 2, Math.min(size.w / w, size.h / h) * 0.92)
  }
  useImperativeHandle(ref, () => ({ fit: () => p.map && setView(fitView()) }))

  // 무엇을 그리나 — 지도는 '지도 좌표계로 그릴 때'만 (SLAM·AMCL 동작 중).
  //   지도 없이 odom 으로 그릴 땐 지난 지도를 숨기고 로봇·라이다만 출발점 기준으로 실시간 표시.
  //   단 「초기 위치」 도구에선 (AMCL 이 아직 위치를 모르는 상태) 지도를 보여주고 odom 기준 레이어를 숨긴다.
  const initialTool = p.tool === 'initial'
  const showMap = !!p.map && (p.frame === p.mapFrame || initialTool)
  const showLive = !initialTool || p.frame === p.mapFrame
  const showCostmap = p.layers.costmap && !!p.costmap && p.costmap.frame === p.frame && showLive

  // 처음 화면 맞춤 (렌더 중 상태 조정 — 다음 렌더 전에 반영된다): 지도가 보이면 지도 전체, 아니면 로봇 중심
  if (showMap && fitted !== 'map') {
    setFitted('map')
    setView(fitView())
  } else if (!showMap && p.pose && fitted === null) {
    setFitted('robot')
    setView(centered(p.pose.x, p.pose.y, 80))
  }

  // 로봇 따라가기: 저장하지 않고 그릴 때마다 로봇 위치로 계산
  const stage = p.follow && p.pose ? centered(p.pose.x, p.pose.y, view.s) : view

  // 휠 확대/축소 — 마우스 아래 지점이 고정되도록
  const onWheel = (e: Konva.KonvaEventObject<WheelEvent>) => {
    e.evt.preventDefault()
    const ptr = e.target.getStage()!.getPointerPosition()!
    const factor = e.evt.deltaY < 0 ? 1.15 : 1 / 1.15
    const s = Math.max(5, Math.min(600, stage.s * factor))
    const wx = (ptr.x - stage.x) / stage.s
    const wy = (ptr.y - stage.y) / stage.s
    setView({ x: ptr.x - wx * s, y: ptr.y - wy * s, s })
  }

  // 화면 포인터 → 지도 좌표 (world 그룹의 로컬 좌표가 곧 지도 좌표)
  const worldPointer = (): Point | null => {
    const pos = worldRef.current?.getRelativePointerPosition()
    return pos ? [pos.x, pos.y] : null
  }

  // 목표/초기 위치 찍기: 누른 곳 = 위치, 끌어서 놓은 방향 = 로봇이 바라볼 방향 (RViz 2D Goal Pose 와 같은 조작)
  const picking = p.tool !== 'view'
  const onDown = () => {
    if (!picking) return
    const w = worldPointer()
    if (w) setDrag({ from: w, to: w })
  }
  const onMove = () => {
    if (!picking || !drag) return
    const w = worldPointer()
    if (w) setDrag({ ...drag, to: w })
  }
  const onUp = () => {
    if (!picking || !drag) return
    const [x0, y0] = drag.from
    const [x1, y1] = drag.to
    let yaw = Math.atan2(y1 - y0, x1 - x0)
    if (Math.hypot(x1 - x0, y1 - y0) < 0.05) {
      // 끌지 않고 탭만 했으면: 로봇에서 그 점을 바라보는 방향
      yaw = p.pose ? Math.atan2(y0 - p.pose.y, x0 - p.pose.x) : 0
    }
    setDrag(null)
    p.onPick({ x: x0, y: y0, yaw })
  }

  const px = 1 / stage.s // 1 화면 픽셀 = 몇 미터
  const arrow = (pose: Pose, len: number, color: string, key: string) => (
    <Arrow
      key={key}
      points={[pose.x, pose.y, pose.x + len * Math.cos(pose.yaw), pose.y + len * Math.sin(pose.yaw)]}
      stroke={color}
      fill={color}
      strokeWidth={3}
      strokeScaleEnabled={false}
      pointerLength={10 * px}
      pointerWidth={10 * px}
      listening={false}
    />
  )

  return (
    <Box ref={wrapRef} sx={{ position: 'relative', width: '100%', bgcolor: '#0b0d10', borderRadius: 1, overflow: 'hidden', touchAction: 'none' }}>
      <Stage
        width={size.w}
        height={size.h}
        x={stage.x}
        y={stage.y}
        scaleX={stage.s}
        scaleY={stage.s}
        draggable={!picking && !p.follow}
        onDragEnd={(e) => e.target === e.target.getStage() && setView({ ...view, x: e.target.x(), y: e.target.y() })}
        onWheel={onWheel}
        onPointerDown={onDown}
        onPointerMove={onMove}
        onPointerUp={onUp}
        style={{ cursor: picking ? 'crosshair' : p.follow ? 'default' : 'grab' }}
      >
        <Layer imageSmoothingEnabled={false}>
          <Group ref={worldRef} scaleY={-1}>
            {showMap && <Grid meta={p.map!} img={mapImg} />}
            {showCostmap && <Grid meta={p.costmap!} img={costImg} />}
            {p.layers.scan && showLive && p.scan.length > 0 && (
              // 점 180개를 컴포넌트 하나로 그린다 (점마다 컴포넌트를 만들면 5Hz 갱신마다 180개를 다시 맞춘다)
              <Shape
                listening={false}
                perfectDrawEnabled={false}
                fill="#ff5252"
                sceneFunc={(ctx, shape) => {
                  const r = 2.2 * px
                  ctx.beginPath()
                  for (const [x, y] of p.scan) {
                    ctx.moveTo(x + r, y)
                    ctx.arc(x, y, r, 0, Math.PI * 2)
                  }
                  ctx.fillShape(shape)
                }}
              />
            )}
            {p.layers.paths && p.globalPath.length > 1 && (
              <Line points={flat(p.globalPath)} stroke="#4caf50" strokeWidth={3} strokeScaleEnabled={false} listening={false} />
            )}
            {p.layers.paths && p.localPath.length > 1 && (
              <Line points={flat(p.localPath)} stroke="#ffeb3b" strokeWidth={2.5} strokeScaleEnabled={false} listening={false} />
            )}
            {showLive && p.footprint.length > 2 && (
              <Line points={flat(p.footprint)} closed stroke="#40c4ff" fill="rgba(64,196,255,0.18)" strokeWidth={2} strokeScaleEnabled={false} listening={false} />
            )}
            {showLive && p.pose && arrow(p.pose, 0.25, '#40c4ff', 'robot')}
            {p.goal && arrow(p.goal, 0.3, '#4caf50', 'goal')}
            {p.pending && arrow(p.pending, 0.3, p.tool === 'initial' ? '#ce93d8' : '#ffa726', 'pending')}
            {drag && (
              <Arrow
                points={[...drag.from, ...drag.to]}
                stroke="#ffa726"
                fill="#ffa726"
                strokeWidth={3}
                strokeScaleEnabled={false}
                pointerLength={10 * px}
                pointerWidth={10 * px}
                listening={false}
              />
            )}
          </Group>
        </Layer>
      </Stage>
      {!showMap && (
        <Typography variant="caption" color="text.secondary" sx={{ position: 'absolute', left: 10, bottom: 6, pointerEvents: 'none' }}>
          {p.pose
            ? '지도 없음 — 로봇·라이다를 출발점(odom) 기준으로 실시간 표시. 지도 만들기(SLAM)를 실행하면 지도가 나타난다'
            : '로봇 위치 없음 — 로봇 bringup 확인'}
        </Typography>
      )}
      {/* 축척 막대: 화면에서 1m 길이 */}
      <Box sx={{ position: 'absolute', right: 12, bottom: 8, pointerEvents: 'none', textAlign: 'center' }}>
        <Box sx={{ width: stage.s, height: 6, borderLeft: '2px solid #9aa4b2', borderRight: '2px solid #9aa4b2', borderBottom: '2px solid #9aa4b2' }} />
        <Typography variant="caption" color="text.secondary">
          1 m
        </Typography>
      </Box>
    </Box>
  )
}
