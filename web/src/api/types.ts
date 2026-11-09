// web-api 의 GET /api/status · WebSocket 'status' 메시지 형식 (remote_pc/src/my_web_api/my_web_api/ros_node.py snapshot())

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

export interface Status {
  time: number
  mode: string
  robot_connected: boolean
  battery: Battery | null
  velocity: { linear: number; angular: number } | null
  topics: TopicStatus[]
}
