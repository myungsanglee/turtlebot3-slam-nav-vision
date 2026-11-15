"""기본 지도(maps.py)와 대기 중 위치 추정(idle_locator.py)."""
import math

import cv2
import numpy as np
from my_localization.scan_matcher import MapModel, Scan, raycast

from my_web_api.geometry import compose
from my_web_api.idle_locator import IdleLocator, inverse
from my_web_api.maps import MapStore

RES = 0.05
ANGLES = np.radians(np.arange(360.0))


def write_map(d, name, img, origin=(-1.0, -0.5)):
    cv2.imwrite(str(d / f'{name}.pgm'), img)
    (d / f'{name}.yaml').write_text(
        f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{origin[0]}, {origin[1]}, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def room_img():
    """map_saver 와 같은 색: 254 빈칸, 0 벽, 205 미지. 비대칭 가구 포함. 이미지 위쪽 = 지도 +y."""
    img = np.full((64, 84), 205, np.uint8)
    img[2:-2, 2:-2] = 0
    img[4:-4, 4:-4] = 254
    img[10:18, 30:34] = 0
    img[40:44, 8:30] = 0
    return img


def test_default_map_set_get_delete(tmp_path):
    write_map(tmp_path, 'a', room_img())
    write_map(tmp_path, 'b', room_img())
    st = MapStore(str(tmp_path))
    assert st.get_default() is None
    st.set_default('a')
    assert st.get_default() == 'a'
    assert [m['default'] for m in st.list() if m['name'] == 'a'] == [True]
    st.delete('a')                       # 기본 지도를 지우면 설정도 사라진다
    assert st.get_default() is None and not (tmp_path / '.default_map').exists()
    st.set_default('b')
    st.set_default(None)
    assert st.get_default() is None


def test_load_grid_matches_map_server_convention(tmp_path):
    img = room_img()
    write_map(tmp_path, 'a', img)
    grid, res, origin = MapStore(str(tmp_path)).load_grid('a')
    assert res == RES and origin == (-1.0, -0.5, 0.0)
    flipped = img[::-1]                  # 행 0 = 아래 (OccupancyGrid)
    assert (grid[flipped == 254] == 0).all() and (grid[flipped == 0] == 100).all() and (grid[flipped == 205] == -1).all()


def test_idle_locator_finds_and_tracks(tmp_path):
    write_map(tmp_path, 'a', room_img())
    grid, res, origin = MapStore(str(tmp_path)).load_grid('a')
    m = MapModel(grid, res, origin[:2])
    rng = np.random.default_rng(0)
    true_map_base = (0.9, 1.0, 0.7)
    odom_base = (0.3, -0.2, 0.4)          # odom 은 아무 데서나 시작했다
    sensor = (-0.1, 0.0, 0.0)

    def scan_at(p):
        sx, sy = p[0] + math.cos(p[2]) * sensor[0], p[1] + math.sin(p[2]) * sensor[0]
        r = raycast(m, grid, sx, sy, p[2], ANGLES) + rng.normal(0, 0.01, 360)
        return Scan(r, ANGLES, sensor)

    loc = IdleLocator()
    loc.set_map('k', grid, res, origin[:2])
    loc.update(scan_at(true_map_base), odom_base)
    assert loc.state == 'localized'
    est = compose(loc.map_from_odom(), odom_base)   # 추정 map←odom 으로 옮긴 로봇 = 실제 위치
    assert math.hypot(est[0] - true_map_base[0], est[1] - true_map_base[1]) < 0.05
    # 로봇이 odom 으로 20cm 전진 → 예측 주변 확인으로 따라간다
    step = (0.2, 0.0, 0.0)
    loc.update(scan_at(compose(true_map_base, step)), compose(odom_base, step))
    est2 = compose(loc.map_from_odom(), compose(odom_base, step))
    tgt = compose(true_map_base, step)
    assert loc.state == 'localized' and math.hypot(est2[0] - tgt[0], est2[1] - tgt[1]) < 0.05
    loc.lose()
    assert loc.map_from_odom() is None and loc.state == 'searching'


def test_inverse_roundtrip():
    p = (0.4, -1.2, 2.1)
    q = compose(p, inverse(p))
    assert abs(q[0]) < 1e-9 and abs(q[1]) < 1e-9 and abs(q[2]) < 1e-9
