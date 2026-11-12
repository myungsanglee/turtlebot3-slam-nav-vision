import { Line, OrbitControls } from '@react-three/drei'
import { Canvas, useThree } from '@react-three/fiber'
import { useEffect, useMemo, useState } from 'react'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import type { OrbitControls as OrbitControlsImpl } from 'three/examples/jsm/controls/OrbitControls.js'
import { DEPTH_COLORS } from '../api/depth'
import type { DepthPoints, GridMeta, Point, Pose, RobotModelInfo, Zone } from '../api/types'

// ───────────────────────────────────────────────────────────────────────────
// 3D 화면 (보기 전용, RViz 의 Orbit 시점과 같은 역할) — 「3D」 탭을 누를 때만 내려받는다 (MapPanel 의 lazy import).
// 좌표: ROS 와 같은 z-up, 지도 좌표(미터) 그대로. 서버가 보낸 데이터를 변환 없이 쓴다.
//   바닥(z=0) = 지도·장애물 지도 이미지 / z=0.135 = 라이다 평면 (base_footprint 위 LDS 높이) / 로봇 = URDF CAD 메시
//   카메라 depth 점 = 서버가 보낸 실제 높이(z) 그대로 — 높이별 색 (바닥·장애물 높이·로봇보다 높음)
// ───────────────────────────────────────────────────────────────────────────
THREE.Object3D.DEFAULT_UP.set(0, 0, 1) // 카메라·컨트롤이 z 를 '위'로 쓰게

const SCAN_Z = 0.135 // base_footprint → base_scan 높이 (URDF: base_link 10mm + scan_joint 125mm)
const LINE_Z = 0.01
const ARROW_Z = 0.015 // 로봇 앞 방향 화살표 높이 (모든 바닥 레이어 위)

interface Props {
  frame: string | null
  mapFrame: string
  map: GridMeta | null
  costmap: GridMeta | null
  pose: Pose | null
  scan: Point[]
  depth: DepthPoints
  globalPath: Point[]
  localPath: Point[]
  footprint: Point[]
  robotModel: RobotModelInfo | null
  goal: Pose | null
  zones: Zone[]
  layers: { costmap: boolean; scan: boolean; depth: boolean; paths: boolean }
  follow: boolean
}

// 이미지 텍스처: 새 버전을 다 받은 뒤에 바꿔 끼우고 이전 것은 GPU 메모리에서 해제 (지도는 5초마다 바뀐다)
function useTextureUrl(url: string | null, nearest: boolean) {
  const [tex, setTex] = useState<THREE.Texture | null>(null)
  useEffect(() => {
    if (!url) return
    let alive = true
    new THREE.TextureLoader().load(url, (t) => {
      if (!alive) return t.dispose()
      t.colorSpace = THREE.SRGBColorSpace
      t.magFilter = nearest ? THREE.NearestFilter : THREE.LinearFilter
      setTex((old) => (old?.dispose(), t))
    })
    return () => {
      alive = false
    }
  }, [url, nearest])
  return url ? tex : null
}

function useStl(url: string | null) {
  const [geo, setGeo] = useState<THREE.BufferGeometry | null>(null)
  useEffect(() => {
    if (!url) return
    let alive = true
    new STLLoader().load(url, (g) => {
      if (!alive) return g.dispose()
      setGeo((old) => (old?.dispose(), g))
    })
    return () => {
      alive = false
    }
  }, [url])
  return url ? geo : null
}

// OccupancyGrid 이미지를 바닥에 깐다. 서버 PNG 의 위쪽 = +y 이고, three 의 평면 텍스처도 위쪽(v=1)이 +y 라 그대로 맞는다.
function GridPlane({ meta, url, z, opacity, nearest }: { meta: GridMeta; url: string; z: number; opacity: number; nearest: boolean }) {
  const tex = useTextureUrl(url, nearest)
  if (!tex) return null
  const w = meta.width * meta.resolution
  const h = meta.height * meta.resolution
  return (
    <group position={[meta.origin.x, meta.origin.y, z]} rotation={[0, 0, meta.origin.yaw]}>
      <mesh position={[w / 2, h / 2, 0]}>
        <planeGeometry args={[w, h]} />
        <meshBasicMaterial map={tex} transparent opacity={opacity} depthWrite={false} side={THREE.DoubleSide} />
      </mesh>
    </group>
  )
}

function ScanPoints({ points }: { points: Point[] }) {
  const geo = useMemo(() => {
    const g = new THREE.BufferGeometry()
    const arr = new Float32Array(points.length * 3)
    points.forEach(([x, y], i) => arr.set([x, y, SCAN_Z], i * 3))
    g.setAttribute('position', new THREE.BufferAttribute(arr, 3))
    return g
  }, [points])
  useEffect(() => () => geo.dispose(), [geo])
  return (
    <points geometry={geo}>
      <pointsMaterial color="#ff5252" size={5} sizeAttenuation={false} />
    </points>
  )
}

// 카메라 depth 점: 실제 3D 위치, 높이 구간별 색 (2D 와 같은 색)
function DepthCloud({ depth }: { depth: DepthPoints }) {
  const geo = useMemo(() => {
    const g = new THREE.BufferGeometry()
    const pos = new Float32Array(depth.points.length * 3)
    const col = new Float32Array(depth.points.length * 3)
    const [lo, hi] = depth.band
    const c = { floor: new THREE.Color(DEPTH_COLORS.floor), obstacle: new THREE.Color(DEPTH_COLORS.obstacle), above: new THREE.Color(DEPTH_COLORS.above) }
    depth.points.forEach(([x, y, z], i) => {
      pos.set([x, y, z], i * 3)
      const k = z < lo ? c.floor : z <= hi ? c.obstacle : c.above
      col.set([k.r, k.g, k.b], i * 3)
    })
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3))
    g.setAttribute('color', new THREE.BufferAttribute(col, 3))
    return g
  }, [depth])
  useEffect(() => () => geo.dispose(), [geo])
  return (
    <points geometry={geo}>
      <pointsMaterial vertexColors size={4} sizeAttenuation={false} />
    </points>
  )
}

// URDF 의 CAD 메시를 로봇 위치에. STL 원본 좌표 → (scale, rpy, xyz) → base_footprint → 지도 위 pose
function Robot({ pose, model, url }: { pose: Pose; model: RobotModelInfo; url: string }) {
  const geo = useStl(url)
  const t = model.transform!
  return (
    <group position={[pose.x, pose.y, 0]} rotation={[0, 0, pose.yaw]}>
      {geo && (
        <group position={t.xyz as [number, number, number]} rotation={new THREE.Euler(t.rpy[0], t.rpy[1], t.rpy[2], 'ZYX')}>
          <mesh geometry={geo} scale={t.scale as [number, number, number]}>
            <meshStandardMaterial color="#b9c3d0" roughness={0.65} metalness={0.15} />
          </mesh>
        </group>
      )}
      {/* 앞 방향 표시: 로봇 앞 바닥의 화살표 (+x). 장애물 지도(z=4mm)·경로(z=1cm)보다 위에 그린다.
          circleGeometry(r, 3) 은 첫 꼭짓점이 +x 를 향한 정삼각형이라 회전 없이 그대로 화살촉이 된다
          (중심이 x=0.25 면 촉 끝 0.28, 뒷변 0.235 → 선 끝 0.24 와 맞물림) */}
      <Line points={[[0.08, 0, ARROW_Z], [0.24, 0, ARROW_Z]]} color="#40c4ff" lineWidth={3} />
      <mesh position={[0.25, 0, ARROW_Z]}>
        <circleGeometry args={[0.03, 3]} />
        <meshBasicMaterial color="#40c4ff" />
      </mesh>
    </group>
  )
}

// 로봇 따라가기: 카메라와 회전 중심을 로봇이 움직인 만큼 같이 옮긴다 (보는 각도·거리는 유지)
function Follow({ pose, enabled }: { pose: Pose | null; enabled: boolean }) {
  const controls = useThree((s) => s.controls) as OrbitControlsImpl | null
  const camera = useThree((s) => s.camera)
  const invalidate = useThree((s) => s.invalidate)
  useEffect(() => {
    if (!enabled || !pose || !controls) return
    const d = new THREE.Vector3(pose.x - controls.target.x, pose.y - controls.target.y, 0)
    camera.position.add(d)
    controls.target.add(d)
    controls.update()
    invalidate()
  }, [enabled, pose, controls, camera, invalidate])
  return null
}

// 처음 열 때 볼 곳: 로봇(있으면) 또는 지도 가운데를 비스듬히 위에서
function initialTarget(p: Props): [number, number] {
  if (p.pose) return [p.pose.x, p.pose.y]
  if (p.map) return [p.map.origin.x + (p.map.width * p.map.resolution) / 2, p.map.origin.y + (p.map.height * p.map.resolution) / 2]
  return [0, 0]
}

export default function Map3D(p: Props) {
  const [target] = useState(() => initialTarget(p))
  const showMap = !!p.map && p.frame === p.mapFrame
  const showCostmap = p.layers.costmap && !!p.costmap && p.costmap.frame === p.frame
  const xyz = (pts: Point[], z = LINE_Z) => pts.map(([x, y]) => [x, y, z] as [number, number, number])

  return (
    <Canvas
      frameloop="demand" // 데이터가 바뀌거나 조작할 때만 다시 그린다 (폰 배터리·발열)
      camera={{ position: [target[0] - 0.55, target[1] - 0.75, 0.6], fov: 50, near: 0.01, far: 200, up: [0, 0, 1] }}
      style={{ width: '100%', height: '100%', borderRadius: 4 }}
    >
      <color attach="background" args={['#0b0d10']} />
      <hemisphereLight args={['#dfe7f2', '#20242b', 0.9]} />
      <directionalLight position={[2, -3, 5]} intensity={1.3} />
      <OrbitControls makeDefault target={[target[0], target[1], 0]} enableDamping={false} maxPolarAngle={Math.PI / 2 - 0.02} />
      <Follow pose={p.pose} enabled={p.follow} />

      {/* 1m 격자 (바닥 바로 아래) */}
      <gridHelper args={[30, 30, '#5a6577', '#3a414d']} rotation={[Math.PI / 2, 0, 0]} position={[target[0], target[1], -0.002]} />

      {showMap && <GridPlane meta={p.map!} url={`/api/map.png?v=${p.map!.version}`} z={0} opacity={1} nearest />}
      {showCostmap && <GridPlane meta={p.costmap!} url={`/api/costmap.png?v=${p.costmap!.version}`} z={0.004} opacity={0.85} nearest />}
      {p.layers.scan && p.scan.length > 0 && <ScanPoints points={p.scan} />}
      {p.layers.depth && p.depth.points.length > 0 && <DepthCloud depth={p.depth} />}
      {p.layers.paths && p.globalPath.length > 1 && <Line points={xyz(p.globalPath)} color="#4caf50" lineWidth={3} />}
      {p.layers.paths && p.localPath.length > 1 && <Line points={xyz(p.localPath, LINE_Z + 0.002)} color="#ffeb3b" lineWidth={2.5} />}
      {p.footprint.length > 2 && (
        <Line points={xyz([...p.footprint, p.footprint[0]], 0.006)} color="#40c4ff" lineWidth={1.5} dashed dashSize={0.03} gapSize={0.02} />
      )}
      {showMap &&
        p.zones.map((z, i) => (
          // 금지 구역(외곽선)·가상 벽 — 바닥 바로 위 빨강
          <Line key={`zone-${i}`} points={xyz(z.kind === 'area' ? [...z.points, z.points[0]] : z.points, 0.012)} color="#ff5252" lineWidth={z.kind === 'area' ? 2 : 5} />
        ))}
      {p.goal && (
        <Line
          points={[[p.goal.x, p.goal.y, 0.02], [p.goal.x + 0.3 * Math.cos(p.goal.yaw), p.goal.y + 0.3 * Math.sin(p.goal.yaw), 0.02]]}
          color="#4caf50"
          lineWidth={4}
        />
      )}
      {p.pose && p.robotModel?.available && (
        <Robot pose={p.pose} model={p.robotModel} url={`/api/robot/mesh.stl?v=${p.robotModel.version}`} />
      )}
    </Canvas>
  )
}
