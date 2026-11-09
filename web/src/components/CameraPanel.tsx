import { Box, Card, CardContent, Chip, Stack, ToggleButton, ToggleButtonGroup, Typography } from '@mui/material'
import { useState } from 'react'
import type { TopicStatus } from '../api/types'

type Source = 'camera' | 'vision'

// 영상은 web-api 의 MJPEG(/api/camera.mjpg, /api/vision.mjpg)를 <img> 로 그대로 띄운다.
// 로봇이 이미 JPEG 로 보내므로 서버도 브라우저도 재인코딩·디코딩 코드가 없다.
export default function CameraPanel({ topics, connectionId }: { topics: TopicStatus[]; connectionId: number }) {
  const [source, setSource] = useState<Source>('camera')
  const topic = topics.find((t) => t.key === (source === 'camera' ? 'color' : 'vision'))
  const live = topic?.state === 'ok'

  const message = !topic
    ? '서버 연결 대기 중'
    : source === 'vision' && topic.state === 'off'
      ? 'Vision 이 꺼져 있음 — 지도 만들기·주행 모드에서 켜진다'
      : '카메라 영상이 들어오지 않음 — 로봇 상태를 확인'

  return (
    <Card>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'center', mb: 1.5, gap: 1, flexWrap: 'wrap' }}>
          <Typography variant="subtitle1" sx={{ fontWeight: 600, mr: 'auto' }}>
            카메라
          </Typography>
          {live && <Chip size="small" label={`${topic!.rate.toFixed(1)} fps`} variant="outlined" />}
          <ToggleButtonGroup size="small" exclusive value={source} onChange={(_, v) => v && setSource(v)}>
            <ToggleButton value="camera">원본</ToggleButton>
            <ToggleButton value="vision">검출 결과</ToggleButton>
          </ToggleButtonGroup>
        </Stack>
        <Box
          sx={{
            position: 'relative', aspectRatio: '4 / 3', bgcolor: 'black', borderRadius: 1, overflow: 'hidden',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
          }}
        >
          {live ? (
            // key 가 바뀌면 <img> 를 새로 만들어 스트림을 다시 연다 (서버 재연결·영상 복구 시)
            <img
              key={`${source}-${connectionId}`}
              src={`/api/${source}.mjpg?c=${connectionId}`}
              alt={source === 'camera' ? '카메라 영상' : 'Vision 검출 영상'}
              style={{ width: '100%', height: '100%', objectFit: 'contain' }}
            />
          ) : (
            <Typography color="text.secondary" sx={{ px: 3, textAlign: 'center' }}>
              {message}
            </Typography>
          )}
        </Box>
      </CardContent>
    </Card>
  )
}
