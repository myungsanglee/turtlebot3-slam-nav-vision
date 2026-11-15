import { Alert, Box, Stack } from '@mui/material'
import { useRobot } from './api/useRobot'
import BatteryCard from './components/BatteryCard'
import MapPanel from './components/MapPanel'
import ModePanel from './components/ModePanel'
import NavPanel from './components/NavPanel'
import StatusBar from './components/StatusBar'
import TopicTable from './components/TopicTable'

// 화면 (docs/my_web_api.md 4.22):
//   왼쪽(폰: 위) = 조작 화면 — 지도 크게 + 카메라 작은 창(맞바꾸기) + 겹치는 조이스틱·주행 상태. 화면 높이에 꼭 맞춰 스크롤 없이 조작.
//   오른쪽(폰: 아래) = 운용 모드(지도 만들기·저장·주행) · 주행 상세 · 배터리 · 센서 상태 — 따로 스크롤.
export default function App() {
  const robot = useRobot()
  const { status, connected } = robot
  const topics = status?.topics ?? []

  return (
    <Box sx={{ minHeight: '100vh' }}>
      <StatusBar status={status} connected={connected} />
      <Box sx={{ p: { xs: 1, md: 2 } }}>
        {!connected && (
          <Alert severity="error" sx={{ mb: 1 }}>
            웹 서버(web-api)에 연결할 수 없습니다. 2초마다 다시 시도합니다.
          </Alert>
        )}
        <Box sx={{ display: 'grid', gap: 2, gridTemplateColumns: { xs: 'minmax(0, 1fr)', lg: 'minmax(0, 1fr) 380px' }, alignItems: 'start' }}>
          <MapPanel robot={robot} />
          <Stack
            spacing={2}
            sx={{
              // 넓은 화면: 조작 화면 높이에 맞춰 옆 패널만 따로 스크롤
              maxHeight: { lg: 'calc(100vh - 96px)' }, overflowY: { lg: 'auto' }, position: { lg: 'sticky' }, top: { lg: 80 }, pr: { lg: 0.5 },
              '& > *': { flexShrink: 0 }, // 카드가 눌려 잘리지 않게 (패널이 스크롤된다)
            }}
          >
            <ModePanel mode={robot.mode} />
            <NavPanel nav={robot.nav} navigation={status?.navigation} />
            <BatteryCard battery={status?.battery ?? null} />
            <TopicTable topics={topics} />
          </Stack>
        </Box>
      </Box>
    </Box>
  )
}
