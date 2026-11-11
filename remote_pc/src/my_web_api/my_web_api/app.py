"""FastAPI 앱: HTTP/WebSocket API + React 정적 파일.

API (docs/web_ui_design.md 5장, docs/my_web_api.md 5장):
  GET  /api/status            로봇 연결·배터리·속도·Nav 준비 상태·토픽별 수신 상태
  GET  /api/camera.mjpg       카메라 컬러 영상 (MJPEG)
  GET  /api/vision.mjpg       Vision 검출 결과 영상 (MJPEG, Vision 이 켜져 있을 때)
  GET  /api/map  · /api/map.png          지도 메타(해상도·원점·크기·버전) / 이미지
  GET  /api/costmap · /api/costmap.png   로컬 장애물 지도 메타 / 이미지 (원점은 map 좌표로 변환됨)
  GET  /api/nav               주행 상태
  POST /api/nav/goal          {x, y, yaw}  지도 좌표로 이동 (Nav2 NavigateToPose)
  POST /api/nav/cancel        이동 취소
  POST /api/nav/initial_pose  {x, y, yaw}  AMCL 초기 위치 (저장 지도 주행 모드)
  WS   /ws                    status(2Hz)·pose(10Hz)·scan·path·footprint·map·costmap·nav 를 바뀔 때 push
  GET  /docs                  자동 생성 API 문서
"""
from __future__ import annotations

import asyncio
import os
import threading

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .monitor import FrameStore

BOUNDARY = 'frame'
WS_TICK_SEC = 0.1          # WebSocket 루프 주기 (pose 10Hz)
# 실패 이유 → HTTP 상태 코드
REASON_STATUS = {'unavailable': 503, 'rejected': 409, 'no_goal': 409, 'error': 500, 'timeout': 504}


class Pose2D(BaseModel):
    """지도(map) 좌표계의 위치와 방향. yaw 는 라디안, +x 축에서 반시계 방향이 +."""
    x: float = Field(description='m')
    y: float = Field(description='m')
    yaw: float = Field(0.0, description='rad')


def create_app(node, static_dir: str = '', stop: threading.Event | None = None) -> FastAPI:
    """stop: 서버 종료 요청 시 set 되는 이벤트. 끝나지 않는 응답(MJPEG·WebSocket)이 스스로 끝나게 해
    uvicorn 이 강제 취소할 일이 없게 한다."""
    stop = stop or threading.Event()
    app = FastAPI(title='TurtleBot3 Web API', version='0.2.0',
                  description='브라우저용 로봇 API — 브라우저는 ROS 를 직접 다루지 않는다.')

    # ---------------- 상태·영상 (P1) ----------------
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

    # ---------------- 지도 (P2) ----------------
    def grid_meta(store, what):
        meta = store.meta()
        if meta is None:
            raise HTTPException(404, f'{what} 없음 (SLAM/Nav2 가 실행 중인지 확인)')
        return meta

    def grid_png(store, what):
        png = store.png()
        if png is None:
            raise HTTPException(404, f'{what} 없음')
        # 주소에 ?v=버전 을 붙여 요청하므로 같은 버전은 캐시해도 된다
        return Response(png, media_type='image/png', headers={'Cache-Control': 'max-age=3600'})

    @app.get('/api/map')
    def map_meta():
        return grid_meta(node.map, '지도')

    @app.get('/api/map.png')
    def map_png():
        return grid_png(node.map, '지도')

    @app.get('/api/costmap')
    def costmap_meta():
        return grid_meta(node.costmap, '장애물 지도')

    @app.get('/api/costmap.png')
    def costmap_png():
        return grid_png(node.costmap, '장애물 지도')

    # ---------------- 주행 (P2) ----------------
    async def nav_call(kind: str, **kw):
        """요청을 ROS 스레드 큐에 넣고 결과를 기다린다 (nav.py 의 스레드 규칙)."""
        fut = node.nav.submit(kind, **kw)
        try:
            result = await asyncio.wait_for(asyncio.wrap_future(fut), timeout=5.0)
        except asyncio.TimeoutError:
            result = {'ok': False, 'reason': 'timeout', 'message': 'Nav2 응답 없음 (5초)'}
        if result.get('ok'):
            return result
        return JSONResponse(result, status_code=REASON_STATUS.get(result.get('reason'), 500))

    @app.get('/api/nav')
    def nav_state():
        return node.nav.snapshot()

    @app.post('/api/nav/goal')
    async def nav_goal(goal: Pose2D):
        return await nav_call('goal', x=goal.x, y=goal.y, yaw=goal.yaw)

    @app.post('/api/nav/cancel')
    async def nav_cancel():
        return await nav_call('cancel')

    @app.post('/api/nav/initial_pose')
    async def nav_initial_pose(pose: Pose2D):
        return await nav_call('initial_pose', x=pose.x, y=pose.y, yaw=pose.yaw)

    # ---------------- WebSocket ----------------
    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        """연결마다 '마지막으로 보낸 버전'을 기억하고, 바뀐 것만 보낸다.
        pose 는 TF 조회값이라 버전이 없어 값이 바뀔 때만, status 는 status_period 마다."""
        await websocket.accept()
        sent: dict[str, object] = {}
        status_every = max(1, round(node.status_period / WS_TICK_SEC))
        tick = 0

        def changed(key, version):
            if sent.get(key) == version:
                return False
            sent[key] = version
            return True

        try:
            while not stop.is_set():
                if tick % status_every == 0:
                    await websocket.send_json({'type': 'status', **node.snapshot()})
                pose = node.robot_pose()
                if changed('pose', None if pose is None else tuple(pose.values())):
                    await websocket.send_json({'type': 'pose', 'pose': pose})
                for key, store in (('map', node.map), ('costmap', node.costmap)):
                    meta = store.meta()
                    if meta is not None and changed(key, meta['version']):
                        await websocket.send_json({'type': key, **meta})
                version, scan = node.scan_pts.get()
                if scan is not None and changed('scan', version):
                    await websocket.send_json({'type': 'scan', **scan})
                for name, store in node.paths.items():
                    version, pts = store.get()
                    if pts is not None and changed(f'path_{name}', version):
                        await websocket.send_json({'type': 'path', 'name': name, 'points': pts})
                version, pts = node.footprint.get()
                if pts is not None and changed('footprint', version):
                    await websocket.send_json({'type': 'footprint', 'points': pts})
                nav = node.nav.snapshot()
                if changed('nav', nav['version']):
                    await websocket.send_json({'type': 'nav', **nav})
                tick += 1
                await asyncio.sleep(WS_TICK_SEC)
            await websocket.close(code=1001)   # 1001 = going away (서버 종료)
        except (WebSocketDisconnect, RuntimeError):
            pass   # 브라우저가 닫힘

    # 정적 파일(React 빌드)은 API 라우트 뒤에 '/' 로 붙인다 — 먼저 붙이면 /api 를 가로챈다
    if static_dir and os.path.isfile(os.path.join(static_dir, 'index.html')):
        app.mount('/', StaticFiles(directory=static_dir, html=True), name='web')

    return app
