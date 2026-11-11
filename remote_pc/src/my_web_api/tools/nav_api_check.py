"""웹 콘솔 주행 API 점검: REST 로 목표·취소·초기 위치를 보내고 WebSocket 으로 nav 상태 변화를 기록한다.
⚠️ 진짜 Nav2 가 떠 있으면 로봇이 실제로 움직인다 — tools/fake_nav.py 와 함께 쓸 것.
"""
import asyncio, json, sys, urllib.request, websockets
def post(path, body=None):
    req = urllib.request.Request('http://localhost:8080' + path, data=json.dumps(body or {}).encode(),
                                 headers={'content-type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(req) as r: return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e: return e.code, json.loads(e.read())
async def watch(ws, sec):
    states = []
    end = asyncio.get_event_loop().time() + sec
    while asyncio.get_event_loop().time() < end:
        try: m = json.loads(await asyncio.wait_for(ws.recv(), 0.5))
        except asyncio.TimeoutError: continue
        if m['type'] == 'nav':
            fb = m.get('feedback') or {}
            s = m['state'] + (f"({fb.get('distance_remaining')})" if fb else '')
            if not states or states[-1].split('(')[0] != m['state'] or len(states) < 3: states.append(s)
    return states
async def main():
    async with websockets.connect('ws://localhost:8080/ws') as ws:
        await watch(ws, 0.5)
        print('1) 목표 → 도착 :', post('/api/nav/goal', {'x': 1.0, 'y': 0.5, 'yaw': 1.57}), await watch(ws, 3.0))
        print('2) 목표 → 취소 :', post('/api/nav/goal', {'x': 1.0, 'y': 0.5, 'yaw': 0}), end=' ')
        await asyncio.sleep(0.6); print(post('/api/nav/cancel'), await watch(ws, 1.5))
        print('3) 거절        :', post('/api/nav/goal', {'x': 999, 'y': 0, 'yaw': 0}), await watch(ws, 1.0))
        print('4) 진행 중 아님 취소:', post('/api/nav/cancel'))
        print('5) 초기 위치    :', post('/api/nav/initial_pose', {'x': 0.1, 'y': 0.2, 'yaw': 0.3}))
asyncio.run(main())
