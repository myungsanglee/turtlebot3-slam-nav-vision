import { useCallback, useEffect, useReducer, useRef } from 'react'
import type { RobotStream } from './types'

// WebSocket /ws 하나로 모든 실시간 데이터를 받는다. 서버는 바뀐 것만 보낸다 (app.py ws()).
// 끊기면 2초 뒤 다시 연결. connectionId 는 재연결마다 바뀐다 → MJPEG 같은 긴 연결을 다시 열 때 쓴다.

const initial: RobotStream = {
  connected: false,
  connectionId: 0,
  status: null,
  pose: null,
  map: null,
  costmap: null,
  scan: [],
  scanFrame: null,
  depth: { points: [], band: [0.04, 0.25] },
  paths: { global: [], local: [] },
  footprint: [],
  nav: null,
  mode: null,
  send: () => {},
}

type Action = { kind: 'open' } | { kind: 'close' } | { kind: 'message'; msg: any } // eslint-disable-line @typescript-eslint/no-explicit-any

function reducer(s: RobotStream, a: Action): RobotStream {
  if (a.kind === 'open') return { ...s, connected: true, connectionId: s.connectionId + 1 }
  if (a.kind === 'close') return { ...s, connected: false }
  const { type, ...m } = a.msg
  switch (type) {
    case 'status':
      return { ...s, status: m }
    case 'pose':
      return { ...s, pose: m.pose }
    case 'map':
      return { ...s, map: m }
    case 'costmap':
      return { ...s, costmap: m }
    case 'scan':
      return { ...s, scan: m.points, scanFrame: m.frame }
    case 'depth_points':
      return { ...s, depth: { points: m.points, band: m.band } }
    case 'path':
      return { ...s, paths: { ...s.paths, [m.name]: m.points } }
    case 'footprint':
      return { ...s, footprint: m.points }
    case 'nav':
      return { ...s, nav: m }
    case 'mode':
      return { ...s, mode: m }
    default:
      return s
  }
}

export function useRobot(): RobotStream {
  const [state, dispatch] = useReducer(reducer, initial)
  const wsRef = useRef<WebSocket | null>(null)

  useEffect(() => {
    let ws: WebSocket | null = null
    let closed = false
    let retry: number | undefined

    const connect = () => {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      ws = new WebSocket(`${proto}://${location.host}/ws`)
      wsRef.current = ws
      ws.onopen = () => dispatch({ kind: 'open' })
      ws.onmessage = (ev) => dispatch({ kind: 'message', msg: JSON.parse(ev.data) })
      ws.onclose = () => {
        dispatch({ kind: 'close' })
        if (!closed) retry = window.setTimeout(connect, 2000)
      }
    }
    connect()
    return () => {
      closed = true
      window.clearTimeout(retry)
      ws?.close()
    }
  }, [])

  // 열려 있을 때만 보낸다 (끊긴 동안의 조이스틱 명령은 버린다 — 재연결 후 오래된 명령이 나가면 위험)
  const send = useCallback((msg: object) => {
    const ws = wsRef.current
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg))
  }, [])

  return { ...state, send }
}
