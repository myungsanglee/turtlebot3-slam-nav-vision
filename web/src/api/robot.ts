import { useQuery } from '@tanstack/react-query'
import type { RobotModelInfo } from './types'

// 로봇 모양 정보는 한 번만 받는다. 서버 재시작(웹소켓 재연결) 때 connectionId 가 바뀌면 다시 받는다.
export function useRobotModel(connectionId: number) {
  return useQuery({
    queryKey: ['robot-model', connectionId],
    queryFn: async (): Promise<RobotModelInfo> => (await fetch('/api/robot')).json(),
    // 서버가 URDF 를 받기 전이면 available=false — 2초 뒤 다시 확인
    refetchInterval: (q) => (q.state.data?.available ? false : 2000),
  })
}
