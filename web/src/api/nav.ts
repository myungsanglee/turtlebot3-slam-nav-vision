import { useMutation } from '@tanstack/react-query'
import type { Pose } from './types'

// 주행 요청 (REST). 실패하면 서버가 준 한글 메시지를 Error 로 던진다 → 화면이 그대로 보여준다.
async function post(path: string, body?: unknown): Promise<{ ok: boolean; message: string }> {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  })
  const data = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(data.message ?? data.detail ?? `요청 실패 (${res.status})`)
  return data
}

export const useNavGoal = () => useMutation({ mutationFn: (p: Pose) => post('/api/nav/goal', p) })
export const useNavCancel = () => useMutation({ mutationFn: () => post('/api/nav/cancel') })
export const useInitialPose = () => useMutation({ mutationFn: (p: Pose) => post('/api/nav/initial_pose', p) })
