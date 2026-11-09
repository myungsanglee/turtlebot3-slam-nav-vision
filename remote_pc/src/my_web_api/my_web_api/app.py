"""FastAPI 앱: HTTP/WebSocket API + React 정적 파일.

API (docs/web_ui_design.md 5장 중 P1 범위):
  GET  /api/status        로봇 연결·배터리·속도·토픽별 수신 상태
  GET  /api/camera.mjpg   카메라 컬러 영상 (MJPEG)
  GET  /api/vision.mjpg   Vision 검출 결과 영상 (MJPEG, Vision 이 켜져 있을 때)
  WS   /ws                status 를 주기적으로 push  {"type": "status", ...}
  GET  /docs              자동 생성 API 문서 (FastAPI)
"""
from __future__ import annotations

import asyncio
import os
import threading

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from .monitor import FrameStore

BOUNDARY = 'frame'


def create_app(node, static_dir: str = '', stop: threading.Event | None = None) -> FastAPI:
    """stop: 서버 종료 요청 시 set 되는 이벤트. 끝나지 않는 응답(MJPEG·WebSocket)이 스스로 끝나게 해
    uvicorn 이 강제 취소할 일이 없게 한다."""
    stop = stop or threading.Event()
    app = FastAPI(title='TurtleBot3 Web API', version='0.1.0',
                  description='브라우저용 로봇 API — 브라우저는 ROS 를 직접 다루지 않는다.')

    @app.get('/api/status')
    def status():
        return node.snapshot()

    async def mjpeg(store: FrameStore, request: Request):
        """multipart/x-mixed-replace: JPEG 를 경계 문자열로 이어 붙여 계속 보내면 <img> 가 동영상처럼 갱신한다.
        새 프레임이 있을 때만 보낸다 (버전 비교). 로봇이 이미 JPEG 로 압축해 보내므로 재인코딩 없음."""
        last = -1
        while not stop.is_set() and not await request.is_disconnected():
            version, data = store.get()
            if data is not None and version != last:
                last = version
                yield (f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n'
                       f'Content-Length: {len(data)}\r\n\r\n').encode() + data + b'\r\n'
            else:
                await asyncio.sleep(0.02)

    def stream(key: str, request: Request) -> StreamingResponse:
        return StreamingResponse(mjpeg(node.frames[key], request),
                                 media_type=f'multipart/x-mixed-replace; boundary={BOUNDARY}',
                                 headers={'Cache-Control': 'no-cache, no-store'})

    @app.get('/api/camera.mjpg')
    async def camera(request: Request):
        return stream('color', request)

    @app.get('/api/vision.mjpg')
    async def vision(request: Request):
        return stream('vision', request)

    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        await websocket.accept()
        try:
            while not stop.is_set():
                await websocket.send_json({'type': 'status', **node.snapshot()})
                await asyncio.sleep(node.status_period)
            await websocket.close(code=1001)   # 1001 = going away (서버 종료)
        except (WebSocketDisconnect, RuntimeError):
            pass   # 브라우저가 닫힘

    # 정적 파일(React 빌드)은 API 라우트 뒤에 '/' 로 붙인다 — 먼저 붙이면 /api 를 가로챈다
    if static_dir and os.path.isfile(os.path.join(static_dir, 'index.html')):
        app.mount('/', StaticFiles(directory=static_dir, html=True), name='web')

    return app
