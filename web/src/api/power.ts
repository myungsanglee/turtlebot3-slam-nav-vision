import { useMutation } from '@tanstack/react-query'
import type { PowerAction } from './types'

// 로봇 전원 동작 (대기·켜기·ROS 재시작·Pi 재부팅·끄기). 상태는 WebSocket status.power 로 온다.
export const usePowerAction = () =>
  useMutation({
    mutationFn: async (action: PowerAction) => {
      const res = await fetch('/api/power', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ action }),
      })
      const data = await res.json().catch(() => ({}))
      if (!res.ok) throw new Error(data.message ?? data.detail ?? `요청 실패 (${res.status})`)
      return data
    },
  })
