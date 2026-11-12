import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query'
import type { Zone } from './types'

// 금지 구역 (저장 지도별). 저장하면 서버가 그 지도로 주행 중일 때 Nav2 에 바로 반영한다.
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

const url = (map: string) => `/api/maps/${encodeURIComponent(map)}/zones`

export const useZones = (map: string | null) =>
  useQuery({
    queryKey: ['zones', map],
    queryFn: async (): Promise<Zone[]> => (await call('GET', url(map!))).zones,
    enabled: !!map,
  })

export function useSaveZones(map: string | null) {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: async (zones: Zone[]): Promise<{ message: string; applied: boolean }> => call('PUT', url(map!), { zones }),
    onSuccess: () => {
      qc.invalidateQueries({ queryKey: ['zones', map] })
      qc.invalidateQueries({ queryKey: ['maps'] })
    },
  })
}
