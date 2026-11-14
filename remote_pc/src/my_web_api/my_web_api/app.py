"""FastAPI 앱: HTTP/WebSocket API + React 정적 파일.

API (docs/web_ui_design.md 5장, docs/my_web_api.md 5장):
  GET  /api/status            로봇 연결·배터리·속도·Nav 준비 상태·토픽별 수신 상태
  GET  /api/camera.mjpg       카메라 컬러 영상 (MJPEG)
  GET  /api/vision.mjpg       Vision 검출 결과 영상 (MJPEG, Vision 이 켜져 있을 때)
  GET  /api/map  · /api/map.png          지도 메타(해상도·원점·크기·버전) / 이미지
  GET  /api/costmap · /api/costmap.png   로컬 장애물 지도 메타 / 이미지 (원점은 map 좌표로 변환됨)
  GET  /api/robot · /api/robot/top.png · /api/robot/mesh.stl
                              로봇 모양 (URDF 의 base_link CAD 메시): 정보 / 위에서 본 이미지 / 3D 용 STL
  GET  /api/nav               주행 상태
  POST /api/nav/goal          {x, y, yaw}  지도 좌표로 이동 (Nav2 NavigateToPose)
  POST /api/nav/cancel        이동 취소
  POST /api/nav/initial_pose  {x, y, yaw}  AMCL 초기 위치 (저장 지도 주행 모드)
  POST /api/nav/relocalize    자동 위치 찾기 — 지도 전체 검색 (my_localization, 결과는 nav.localization)
  GET  /api/mode · POST /api/mode {mode, map?, vision?}   운용 모드 (IDLE / MAPPING / NAVIGATION)
  GET  /api/mode/log          모드 launch 로그 꼬리 (문제 확인용)
  GET  /api/maps · POST /api/maps {name} · DELETE /api/maps/{name} · GET /api/maps/{name}.png
  GET|PUT /api/maps/{name}/zones   금지 구역·가상 벽 (저장 지도별, 주행 중이면 Nav2 에 즉시 반영)
                              저장 지도 목록 / 지금 지도 저장 / 삭제 / 미리보기
  WS   /ws                    status(2Hz)·pose(10Hz)·scan·depth_points·path·footprint·map·costmap·nav·mode 를 바뀔 때 push,
                              브라우저 → 서버: {"type": "teleop", "x", "y", "scale"} (조이스틱, 0.1초마다) / {"type": "teleop_release"}
  GET  /api/power · POST /api/power {action}   로봇 전원: wake(켜기) / standby(대기) / restart(ROS 재시작) /
                              reboot(Pi 재부팅) / poweroff(Pi 끄기) — 대기·재시작·끄기 전에 웹에서 띄운 SLAM·Nav2 를 먼저 끈다
                              / gyro_reset(자이로 재보정 — 정지 상태에서만, 지도 만들기 중 불가. 상태의 gyro 에 드리프트 °/분)
  GET  /docs                  자동 생성 API 문서
"""
from __future__ import annotations

import asyncio
import gzip
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
REASON_STATUS = {'unavailable': 503, 'rejected': 409, 'no_goal': 409, 'error': 500, 'timeout': 504,
                 'busy': 409, 'external': 409, 'bad_request': 400, 'not_found': 404, 'no_map': 409}


class ModeRequest(BaseModel):
    mode: str = Field(description='IDLE | MAPPING | NAVIGATION')
    map: str | None = Field(None, description='NAVIGATION 일 때 저장 지도 이름')
    vision: bool = Field(True, description='Vision AI(검출) 도 켤지')


class ZonesPut(BaseModel):
    zones: list = Field(description='[{kind: area|wall, points: [[x, y], ...], name}] — 지도 좌표(m)')


class PowerRequest(BaseModel):
    action: str = Field(description='wake | standby | restart | reboot | poweroff | gyro_reset')


class MapSave(BaseModel):
    name: str = Field(description='영문·숫자·_·- 1~40자')


def result(r: dict):
    """{'ok': bool, 'reason': …} → 성공은 200, 실패는 이유별 HTTP 코드."""
    if r.get('ok'):
        return r
    return JSONResponse(r, status_code=REASON_STATUS.get(r.get('reason'), 500))


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

    # ---------------- 로봇 모양 ----------------
    @app.get('/api/robot')
    def robot():
        return node.robot_model.info

    @app.get('/api/robot/top.png')
    def robot_top():
        if node.robot_model.top_png is None:
            raise HTTPException(404, node.robot_model.info.get('reason', '로봇 모델 없음'))
        # 주소에 ?v=버전 (URDF·메시가 바뀔 때만 바뀜) 을 붙여 요청하므로 오래 캐시해도 된다
        return Response(node.robot_model.top_png, media_type='image/png', headers={'Cache-Control': 'max-age=86400'})

    @app.get('/api/robot/mesh.stl')
    def robot_mesh(request: Request):
        gz = node.robot_model.mesh_gz
        if gz is None:
            raise HTTPException(404, node.robot_model.info.get('reason', '로봇 모델 없음'))
        headers = {'Cache-Control': 'max-age=86400'}
        if 'gzip' in request.headers.get('accept-encoding', ''):
            # 미리 압축해 둔 바이트를 그대로 — 브라우저가 풀어서 STL 로 받는다 (서버 전체 gzip 은 MJPEG 스트림을 망쳐서 안 씀)
            return Response(gz, media_type='model/stl', headers={**headers, 'Content-Encoding': 'gzip'})
        return Response(gzip.decompress(gz), media_type='model/stl', headers=headers)

    # ---------------- 주행 (P2) ----------------
    async def nav_call(kind: str, **kw):
        """요청을 ROS 스레드 큐에 넣고 결과를 기다린다 (nav.py 의 스레드 규칙)."""
        fut = node.nav.submit(kind, **kw)
        try:
            r = await asyncio.wait_for(asyncio.wrap_future(fut), timeout=5.0)
        except asyncio.TimeoutError:
            r = {'ok': False, 'reason': 'timeout', 'message': 'Nav2 응답 없음 (5초)'}
        return result(r)

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

    @app.post('/api/nav/relocalize')
    async def nav_relocalize():
        return await nav_call('relocalize')

    # ---------------- 로봇 전원 ----------------
    @app.get('/api/power')
    def power_state():
        return node.power.snapshot()

    @app.post('/api/power')
    def power_set(req: PowerRequest):
        return result(node.power.request(req.action))

    # ---------------- 모드·지도 (P3) ----------------
    @app.get('/api/mode')
    def mode_state():
        return node.mode.snapshot()

    @app.post('/api/mode')
    def mode_set(req: ModeRequest):
        return result(node.mode.request(req.mode.upper(), req.map, req.vision))

    @app.get('/api/mode/log')
    def mode_log(lines: int = 40):
        return {'lines': node.mode.log_tail(max(1, min(60, lines)))}

    @app.get('/api/maps')
    def maps_list():
        return [{k: v for k, v in m.items() if k != 'yaml'} for m in node.maps.list()]

    @app.post('/api/maps')
    def maps_save(req: MapSave):
        # 동기 함수 → FastAPI 가 스레드 풀에서 실행 (map_saver 가 수 초 걸려도 다른 요청을 막지 않는다)
        if node.map.meta() is None:
            return result({'ok': False, 'reason': 'no_map', 'message': '저장할 지도 없음 — 지도 만들기 모드에서'})
        try:
            r = node.maps.save(req.name)
        except ValueError as e:
            return result({'ok': False, 'reason': 'bad_request', 'message': str(e)})
        return r if r['ok'] else result({**r, 'reason': 'error'})

    @app.delete('/api/maps/{name}')
    def maps_delete(name: str):
        snap = node.mode.snapshot()
        if snap['mode'] == 'NAVIGATION' and snap['map'] == name:
            return result({'ok': False, 'reason': 'busy', 'message': '지금 주행에 쓰는 지도는 지울 수 없음'})
        try:
            ok = node.maps.delete(name)
        except ValueError as e:
            return result({'ok': False, 'reason': 'bad_request', 'message': str(e)})
        return result({'ok': True, 'message': '삭제됨'} if ok else {'ok': False, 'reason': 'not_found', 'message': '없음'})

    @app.get('/api/maps/{name}/zones')
    def zones_get(name: str):
        try:
            return {'map': name, 'zones': node.maps.get_zones(name)}
        except ValueError as e:
            return result({'ok': False, 'reason': 'bad_request', 'message': str(e)})

    @app.put('/api/maps/{name}/zones')
    def zones_put(name: str, req: ZonesPut):
        try:
            zones = node.maps.set_zones(name, req.zones)
        except ValueError as e:
            return result({'ok': False, 'reason': 'bad_request', 'message': str(e)})
        except FileNotFoundError:
            return result({'ok': False, 'reason': 'not_found', 'message': f'지도 {name} 없음'})
        node.sync_keepout()   # 지금 주행 중인 지도면 바로 Nav2 에 (아니면 그 지도로 주행할 때 적용)
        applied = node.keepout['map'] == name
        return {'ok': True, 'zones': zones, 'applied': applied,
                'message': f'금지 구역 {len(zones)}개 저장' + (' · 주행에 바로 적용' if applied else ' · 이 지도로 주행할 때 적용')}

    @app.get('/api/maps/{name}.png')
    def maps_thumb(name: str):
        try:
            return Response(node.maps.thumbnail(name), media_type='image/png', headers={'Cache-Control': 'no-cache'})
        except (ValueError, FileNotFoundError, AttributeError):
            raise HTTPException(404, '지도 없음')

    # ---------------- WebSocket ----------------
    @app.websocket('/ws')
    async def ws(websocket: WebSocket):
        """연결마다 '마지막으로 보낸 버전'을 기억하고, 바뀐 것만 보낸다.
        pose 는 TF 조회값이라 버전이 없어 값이 바뀔 때만, status 는 status_period 마다."""
        await websocket.accept()
        sent: dict[str, object] = {}

        async def reader():
            """브라우저 → 서버: 조이스틱. 연결이 끊기면 finally 에서 정지."""
            while True:
                msg = await websocket.receive_json()
                if msg.get('type') == 'teleop':
                    node.teleop.command(msg.get('x', 0.0), msg.get('y', 0.0), msg.get('scale', 1.0))
                elif msg.get('type') == 'teleop_release':
                    node.teleop.release()

        reader_task = asyncio.create_task(reader())
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
                version, dp = node.depth_pts.get()
                if dp is not None and changed('depth_points', version):
                    await websocket.send_json({'type': 'depth_points', **dp})
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
                mode = node.mode.snapshot()
                if changed('mode', (mode['version'], mode['external'])):
                    await websocket.send_json({'type': 'mode', **mode})
                if reader_task.done():
                    break                       # 읽기 쪽이 끊김 → 연결 종료
                tick += 1
                await asyncio.sleep(WS_TICK_SEC)
            if not reader_task.done():
                await websocket.close(code=1001)   # 1001 = going away (서버 종료)
        except (WebSocketDisconnect, RuntimeError):
            pass   # 브라우저가 닫힘
        finally:
            reader_task.cancel()
            node.teleop.release()              # 탭을 닫거나 연결이 끊기면 조이스틱 즉시 정지

    # 정적 파일(React 빌드)은 API 라우트 뒤에 '/' 로 붙인다 — 먼저 붙이면 /api 를 가로챈다
    if static_dir and os.path.isfile(os.path.join(static_dir, 'index.html')):
        app.mount('/', StaticFiles(directory=static_dir, html=True), name='web')

    return app
