import { CssBaseline, ThemeProvider, createTheme } from '@mui/material'
import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'

const theme = createTheme({
  palette: { mode: 'dark', background: { default: '#0f1216', paper: '#171b21' } },
  shape: { borderRadius: 10 },
  typography: { fontFamily: '"Pretendard", "Noto Sans KR", system-ui, sans-serif' },
})

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <App />
    </ThemeProvider>
  </StrictMode>,
)
