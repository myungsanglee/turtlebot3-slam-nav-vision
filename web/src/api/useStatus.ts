import { useEffect, useRef, useState } from 'react'
import type { Status } from './types'

// WebSocket /ws 로 상태를 받는다. 끊기면 2초 뒤 다시 연결 (web-api 재시작·네트워크 순단 대비).
// connectionId 는 다시 연결될 때마다 바뀐다 → 영상(MJPEG) 같은 긴 연결을 새로 열 때 쓴다.
export function useStatus() {
  const [status, setStatus] = useState<Status | null>(null)
  const [connected, setConnected] = useState(false)
  const [connectionId, setConnectionId] = useState(0)
  const retry = useRef<number | undefined>(undefined)

  useEffect(() => {
    let ws: WebSocket | null = null
    let closed = false

    const connect = () => {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      ws = new WebSocket(`${proto}://${location.host}/ws`)
      ws.onopen = () => {
        setConnected(true)
        setConnectionId((n) => n + 1)
      }
      ws.onmessage = (ev) => {
        const msg = JSON.parse(ev.data)
        if (msg.type === 'status') setStatus(msg as Status)
      }
      ws.onclose = () => {
        setConnected(false)
        if (!closed) retry.current = window.setTimeout(connect, 2000)
      }
    }
    connect()
    return () => {
      closed = true
      window.clearTimeout(retry.current)
      ws?.close()
    }
  }, [])

  return { status, connected, connectionId }
}
