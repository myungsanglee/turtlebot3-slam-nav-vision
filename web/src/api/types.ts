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
  age: number | null // 마지막 수신 후 경과 초. 한 번도 안 왔으면 null
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
  localized: boolean // map → 로봇 TF 가 이어져 있음 (SLAM 또는 AMCL 실행 중)
  map_age: number | null
  nav2_ready: boolean
}

export interface Status {
  time: number
  mode: string
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
}

export type Point = [number, number]

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

export interface NavFeedback {
  distance_remaining: number
  navigation_time: number
  estimated_time_remaining: number
  recoveries: number
}

export interface NavState {
  state: NavStateName
  goal: Pose | null
  feedback: NavFeedback | null
  message: string
  version: number
}

export interface RobotStream {
  connected: boolean
  connectionId: number
  status: Status | null
  pose: Pose | null
  map: GridMeta | null
  costmap: GridMeta | null
  scan: Point[]
  paths: { global: Point[]; local: Point[] }
  footprint: Point[]
  nav: NavState | null
}
