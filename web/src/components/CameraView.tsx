import { Box, Chip, Stack, ToggleButton, ToggleButtonGroup, Typography } from '@mui/material'
import { useState } from 'react'
import type { TopicStatus } from '../api/types'

type Source = 'camera' | 'vision'

// 카메라 영상 — 부모 영역을 꽉 채운다 (조작 화면의 큰 화면·작은 창 어디든).
// 영상은 web-api 의 MJPEG(/api/camera.mjpg, /api/vision.mjpg)를 <img> 로 그대로 띄운다.
// 로봇이 이미 JPEG 로 보내므로 서버도 브라우저도 재인코딩·디코딩 코드가 없다.
export default function CameraView({ topics, connectionId, compact = false }: { topics: TopicStatus[]; connectionId: number; compact?: boolean }) {
  const [source, setSource] = useState<Source>('camera')
  const topic = topics.find((t) => t.key === (source === 'camera' ? 'color' : 'vision'))
  const live = topic?.state === 'ok'

  const message = !topic
    ? '서버 연결 대기 중'
    : source === 'vision' && topic.state === 'off'
      ? 'Vision 이 꺼져 있음 — 지도 만들기·주행 모드에서 켜진다'
      : '카메라 영상이 들어오지 않음 — 로봇 상태를 확인'

  return (
    <Box sx={{ position: 'relative', width: '100%', height: '100%', bgcolor: 'black', display: 'flex', alignItems: 'center', justifyContent: 'center', overflow: 'hidden' }}>
      {live ? (
        // key 가 바뀌면 <img> 를 새로 만들어 스트림을 다시 연다 (서버 재연결·영상 복구 시)
        <img
          key={`${source}-${connectionId}`}
          src={`/api/${source}.mjpg?c=${connectionId}`}
          alt={source === 'camera' ? '카메라 영상' : 'Vision 검출 영상'}
          style={{ width: '100%', height: '100%', objectFit: 'contain', display: 'block' }}
        />
      ) : (
        <Typography variant={compact ? 'caption' : 'body2'} color="text.secondary" sx={{ px: 2, textAlign: 'center' }}>
          {message}
        </Typography>
      )}
      {/* 영상 위 작은 조절: 원본 / 검출 결과, fps */}
      <Stack direction="row" spacing={0.5} sx={{ position: 'absolute', left: 6, bottom: 6, alignItems: 'center' }} onClick={(e) => e.stopPropagation()}>
        <ToggleButtonGroup
          size="small"
          exclusive
          value={source}
          onChange={(_, v) => v && setSource(v)}
          sx={{ bgcolor: 'rgba(0,0,0,0.55)', '& .MuiToggleButton-root': { py: 0.1, px: compact ? 0.6 : 1, fontSize: compact ? 11 : 12 } }}
        >
          <ToggleButton value="camera">원본</ToggleButton>
          <ToggleButton value="vision">검출</ToggleButton>
        </ToggleButtonGroup>
        {live && !compact && <Chip size="small" label={`${topic!.rate.toFixed(1)} fps`} sx={{ bgcolor: 'rgba(0,0,0,0.55)' }} />}
      </Stack>
    </Box>
  )
}
