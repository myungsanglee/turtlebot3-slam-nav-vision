import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import type { ModeName, SavedMap } from './types'

// 모드 전환·지도 저장/삭제 (REST). 실패하면 서버의 한글 메시지를 Error 로.
async function call(method: string, path: string, body?: unknown) {
  const res = await fetch(path, {
    method,
    headers: { 'content-type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  })
  const data = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(data.message ?? data.detail ?? `요청 실패 (${res.status})`)
  return data
}

export const useMaps = () =>
  useQuery({ queryKey: ['maps'], queryFn: (): Promise<SavedMap[]> => call('GET', '/api/maps') })

export const useSetMode = () =>
  useMutation({ mutationFn: (r: { mode: ModeName; map?: string; vision?: boolean }) => call('POST', '/api/mode', r) })

export function useSaveMap() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (name: string) => call('POST', '/api/maps', { name }),
    onSuccess: () => qc.invalidateQueries({ queryKey: ['maps'] }),
  })
}

export function useDeleteMap() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (name: string) => call('DELETE', `/api/maps/${encodeURIComponent(name)}`),
    onSuccess: () => qc.invalidateQueries({ queryKey: ['maps'] }),
  })
}

export const useModeLog = (enabled: boolean) =>
  useQuery({
    queryKey: ['mode-log'],
    queryFn: async (): Promise<string[]> => (await call('GET', '/api/mode/log?lines=40')).lines,
    enabled,
    refetchInterval: enabled ? 2000 : false,
  })
