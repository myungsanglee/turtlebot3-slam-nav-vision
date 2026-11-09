import { Alert, Box, Container, Stack } from '@mui/material'
import { useStatus } from './api/useStatus'
import BatteryCard from './components/BatteryCard'
import CameraPanel from './components/CameraPanel'
import StatusBar from './components/StatusBar'
import TopicTable from './components/TopicTable'

// P1 화면: 연결 상태 · 카메라 · 배터리 · 센서 수신 상태 (docs/web_ui_design.md 8장)
export default function App() {
  const { status, connected, connectionId } = useStatus()
  const topics = status?.topics ?? []

  return (
    <Box sx={{ minHeight: '100vh' }}>
      <StatusBar status={status} connected={connected} />
      <Container maxWidth="xl" sx={{ py: 3 }}>
        {!connected && (
          <Alert severity="error" sx={{ mb: 2 }}>
            웹 서버(web-api)에 연결할 수 없습니다. 2초마다 다시 시도합니다.
          </Alert>
        )}
        <Box sx={{ display: 'grid', gap: 2, gridTemplateColumns: { xs: 'minmax(0, 1fr)', md: 'minmax(0, 3fr) minmax(0, 2fr)' }, alignItems: 'start' }}>
          <CameraPanel topics={topics} connectionId={connectionId} />
          <Stack spacing={2}>
            <BatteryCard battery={status?.battery ?? null} />
            <TopicTable topics={topics} />
          </Stack>
        </Box>
      </Container>
    </Box>
  )
}
