import type { DepthPoints, Point3 } from './types'

// 카메라 depth 점의 높이별 색 (2D·3D 공통). 바닥 = '여기까지 비어 있다', 장애물 높이 = Nav2 가 피하는 것
export const DEPTH_COLORS = { floor: '#4f6a86', obstacle: '#ffa726', above: '#9575cd' }

export function splitDepth(d: DepthPoints) {
  const floor: Point3[] = []
  const obstacle: Point3[] = []
  const above: Point3[] = []
  for (const q of d.points) (q[2] < d.band[0] ? floor : q[2] <= d.band[1] ? obstacle : above).push(q)
  return { floor, obstacle, above }
}
