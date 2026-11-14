// web-api 가 보내는 데이터 형식 (remote_pc/src/my_web_api/my_web_api/ros_node.py · app.py · nav.py 와 1:1)
// 좌표는 전부 지도(map) 좌표계, 미터·라디안. yaw 는 +x 에서 반시계가 +.

export type TopicState = 'ok' | 'missing' | 'off'

export interface TopicStatus {
  key: string
  topic: string
  label: string
  expected_hz: number
  optional: boolean
  rate: number
  age: number | null // 마지막 수신 후 경과 초 (끊겨도 유지). 한 번도 안 왔으면 null
  last_received: number | null // 마지막 수신 시각 (epoch 초)
  state: TopicState // ok=수신 중, missing=수신 없음(오류), off=선택 토픽이 꺼짐
}

export interface Battery {
  voltage: number
  percentage: number // OpenCR 펌웨어의 직선 환산(10.5V=0%, 12.3V=100%)을 0~100 으로 자른 값 (참고용)
  percentage_raw: number
  cells: number
  cell_voltage: number
  level: 'ok' | 'warn' | 'critical'
}

export interface NavigationStatus {
  fixed_frame: string
  display_frame: string | null // 화면 좌표계: map(SLAM·AMCL 동작 중) / odom(지도 없음 — 출발점 기준) / null
  localized: boolean // map → 로봇 TF 가 이어져 있음 (SLAM 또는 AMCL 실행 중)
  map_age: number | null
  costmap_age: number | null // 마지막 갱신 후 초 — 오래되면(Nav2 꺼짐) 화면에서 숨긴다
  footprint_age: number | null
  nav2_ready: boolean
  keepout: { map: string | null; zones: number } // Nav2 에 적용 중인 금지 구역
}

// 로봇 전원 상태 (서버 robot_power.py) — Pi 에 SSH 로 물어 본 bringup 상태 + 진행 중 동작
export type PowerState =
  | 'unconfigured' | 'unreachable' | 'off' | 'rebooting'
  | 'standby' | 'starting' | 'running' | 'stopping' | 'error' | 'calibrating'
export type PowerAction = 'wake' | 'standby' | 'restart' | 'reboot' | 'poweroff' | 'gyro_reset'

export interface Power {
  state: PowerState
  bringup: string | null
  uptime: number | null
  throttled: string | null
  busy: PowerAction | null
  last: { action: PowerAction; at: number; ok: boolean; message: string } | null
}

// 자이로 드리프트 (서버 gyro_watch.py) — 바퀴가 멈춘 채 odom 방향이 도는 정도
export interface GyroStatus {
  state: 'unknown' | 'moving' | 'settling' | 'ok' | 'drift'
  rate_deg_min: number | null // 정지 중 방향 변화율 (정상 ≈0, 틀어지면 13~15)
  still_sec: number | null
  wheels: boolean // 바퀴 정보(/joint_states) 수신 여부
  auto: boolean // 자동 재보정 켜짐
  auto_min_deg_min: number
  last_reset: { time: number; auto: boolean; ok: boolean; reason: string | null } | null
}

export interface Status {
  time: number
  mode: string
  power: Power
  gyro?: GyroStatus
  robot_connected: boolean
  battery: Battery | null
  velocity: { linear: number; angular: number } | null
  navigation: NavigationStatus
  topics: TopicStatus[]
}

export interface Pose {
  x: number
  y: number
  yaw: number
  frame?: string // 서버가 보낸 로봇 위치에만 있음 (표시 좌표계)
}

export type Point = [number, number]
export type Point3 = [number, number, number] // x, y (지도 좌표, m) + z (바닥 기준 높이, m)

// 카메라 depth 점 (서버 depth_points.py). band = Nav2 가 장애물로 찍는 높이 범위 [하한, 상한] (색 구분용)
export interface DepthPoints {
  points: Point3[]
  band: [number, number]
}

// OccupancyGrid 메타. 이미지는 /api/map.png?v=version · /api/costmap.png?v=version
export interface GridMeta {
  width: number
  height: number
  resolution: number
  origin: Pose // 격자 (0,0) 칸의 모서리 위치 (map 좌표)
  version: number
  frame?: string
}

export type NavStateName = 'idle' | 'sending' | 'active' | 'succeeded' | 'aborted' | 'canceled' | 'rejected' | 'error'

// GET /api/robot — URDF 의 base_link CAD 메시 (robot_model.py)
export interface RobotModelInfo {
  available: boolean
  reason?: string
  source?: string // URDF 의 메시 경로 (package://…)
  file?: string // STL 파일 이름
  triangles?: number
  version?: string // URDF·메시가 바뀔 때만 바뀜 → 이미지·메시 주소에 붙여 캐시
  top_view?: GridMeta & { z_min: number; z_max: number } // 위에서 본 이미지 (base_footprint 좌표)
  transform?: { scale: number[]; xyz: number[]; rpy: number[] } // STL → base_footprint (3D 화면용)
}

export interface NavFeedback {
  distance_remaining: number
  navigation_time: number
  estimated_time_remaining: number
  recoveries: number
}

// 자동 위치 찾기 (my_localization relocalizer → /relocalization/status). 저장 지도 주행 모드에서만 온다.
export type LocalizationStateName = 'waiting' | 'searching' | 'localized' | 'ambiguous' | 'failed' | 'lost'
export interface LocalizationCandidate {
  x: number
  y: number
  yaw: number
  score: number // 0~1 (우도장 × 빔 일관성)
}
export interface LocalizationStatus {
  state: LocalizationStateName
  method: 'last_pose' | 'global' | 'manual' | 'recovered' | null
  score: number | null // 위치를 정할 때의 점수
  track_score: number | null // 지금 위치에서 스캔이 지도와 맞는 정도 (1초마다)
  pose: Pose | null
  candidates: LocalizationCandidate[]
  message: string
  time: number
}

export interface NavState {
  state: NavStateName
  goal: Pose | null
  feedback: NavFeedback | null
  message: string
  localization: LocalizationStatus | null
  version: number
}

// 운용 모드 (mode_manager.py)
export type ModeName = 'IDLE' | 'MAPPING' | 'NAVIGATION'
export interface ModeState {
  mode: ModeName
  phase: 'idle' | 'starting' | 'running' | 'stopping' | 'error'
  map: string | null
  vision: boolean
  since: number
  message: string
  external: boolean // 웹이 아닌 곳(서버 터미널)에서 SLAM/Nav2 가 떠 있음
  version: number
}

// 금지 구역·가상 벽 (저장 지도별, 지도 좌표 m) — 서버 zones.py
export type ZoneKind = 'area' | 'wall'
export interface Zone {
  kind: ZoneKind // area: 3점 이상 닫힌 영역 / wall: 2점 이상 이은 선
  points: Point[]
  name: string
}

export interface SavedMap {
  name: string
  width: number
  height: number
  resolution: number
  size_m: [number, number]
  saved: number // epoch 초
  zones: number // 금지 구역 개수
}

export interface RobotStream {
  connected: boolean
  connectionId: number
  status: Status | null
  pose: Pose | null
  map: GridMeta | null
  costmap: GridMeta | null
  scan: Point[]
  scanFrame: string | null
  depth: DepthPoints
  paths: { global: Point[]; local: Point[] }
  footprint: Point[]
  nav: NavState | null
  mode: ModeState | null
  send: (msg: object) => void // 브라우저 → 서버 (조이스틱)
}
