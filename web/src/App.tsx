import { Alert, Box, Container, Stack } from '@mui/material'
import { useRobot } from './api/useRobot'
import BatteryCard from './components/BatteryCard'
import CameraPanel from './components/CameraPanel'
import Joystick from './components/Joystick'
import MapPanel from './components/MapPanel'
import ModePanel from './components/ModePanel'
import NavPanel from './components/NavPanel'
import StatusBar from './components/StatusBar'
import TopicTable from './components/TopicTable'

// 화면: 지도(2D/3D) + 클릭 주행 · 운용 모드(지도 만들기·저장·주행) · 카메라 · 조이스틱 · 주행 상태 · 배터리 · 센서 상태
export default function App() {
  const robot = useRobot()
  const { status, connected, connectionId } = robot
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
        <Box sx={{ display: 'grid', gap: 2, gridTemplateColumns: { xs: 'minmax(0, 1fr)', lg: 'minmax(0, 3fr) minmax(0, 2fr)' }, alignItems: 'start' }}>
          <Stack spacing={2}>
            <MapPanel robot={robot} />
            <TopicTable topics={topics} />
          </Stack>
          <Stack spacing={2}>
            <ModePanel mode={robot.mode} />
            <CameraPanel topics={topics} connectionId={connectionId} />
            <Joystick send={robot.send} connected={connected && !!status?.robot_connected} />
            <NavPanel nav={robot.nav} navigation={status?.navigation} />
            <BatteryCard battery={status?.battery ?? null} />
          </Stack>
        </Box>
      </Container>
    </Box>
  )
}
