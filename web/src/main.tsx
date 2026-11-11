import { CssBaseline, ThemeProvider, createTheme } from '@mui/material'
import { QueryClient, QueryClientProvider } from '@tanstack/react-query'
import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'

const theme = createTheme({
  palette: { mode: 'dark', background: { default: '#0f1216', paper: '#171b21' } },
  shape: { borderRadius: 10 },
  typography: { fontFamily: '"Pretendard", "Noto Sans KR", system-ui, sans-serif' },
})

// REST 요청(주행 목표·취소 등)은 TanStack Query 의 mutation 으로 — 진행 중·오류 상태를 선언적으로 다룬다
const queryClient = new QueryClient()

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <QueryClientProvider client={queryClient}>
      <ThemeProvider theme={theme}>
        <CssBaseline />
        <App />
      </ThemeProvider>
    </QueryClientProvider>
  </StrictMode>,
)
