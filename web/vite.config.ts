import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// 개발 서버(npm run dev, :5173)는 API·WebSocket 을 web-api(:8080)로 넘긴다 → 코드에선 항상 같은 출처 경로(/api, /ws)만 쓴다.
// 배포는 npm run build → dist/ 를 web-api 가 직접 제공 (docs/my_web_api.md).
export default defineConfig({
  plugins: [react()],
  // 지도 렌더러(Konva)·MUI 포함 ~810KB(gzip 250KB). Tailscale 내부 콘솔이라 첫 로딩 1회 비용으로 감수 → 경고 기준만 올림
  build: { chunkSizeWarningLimit: 1000 },
  server: {
    host: true,   // Tailscale 로 맥·폰에서 접속
    proxy: {
      '/api': 'http://localhost:8080',
      '/ws': { target: 'ws://localhost:8080', ws: true },
    },
  },
})
