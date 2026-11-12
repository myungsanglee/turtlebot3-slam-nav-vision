import os

import numpy as np
import pytest

from my_web_api import zones as z
from my_web_api.maps import MapStore


def test_validate_rules():
    ok = z.validate([{'kind': 'area', 'points': [[0, 0], [1, 0], [1, 1]]}, {'kind': 'wall', 'points': [[0, 0], [2, 0]]}])
    assert [q['kind'] for q in ok] == ['area', 'wall']
    for bad in ([{'kind': 'area', 'points': [[0, 0], [1, 0]]}],          # area 는 3점 이상
                [{'kind': 'wall', 'points': [[0, 0]]}],                  # wall 은 2점 이상
                [{'kind': 'x', 'points': [[0, 0], [1, 1]]}],
                [{'kind': 'wall', 'points': [[0, 0], [float('nan'), 1]]}],
                'zones'):
        with pytest.raises(ValueError):
            z.validate(bad)


def test_rasterize_area_in_grid_coordinates():
    # 지도 10x10 칸, 5cm, 원점 (-0.25, -0.25) → 지도 (0,0) 은 칸 (5,5). 0~0.1m 사각형 = 칸 5~7 근처
    zones = z.validate([{'kind': 'area', 'points': [[0, 0], [0.1, 0], [0.1, 0.1], [0, 0.1]]}])
    m = z.rasterize(zones, 10, 10, 0.05, -0.25, -0.25)
    assert m.dtype == np.int8 and m.shape == (10, 10)
    rows, cols = np.nonzero(m)
    assert set(np.unique(m)) == {0, 100}
    assert rows.min() == 5 and cols.min() == 5 and rows.max() <= 7 and cols.max() <= 7
    assert m[0, 0] == 0 and m[9, 9] == 0


def test_rasterize_wall_is_continuous_line():
    zones = z.validate([{'kind': 'wall', 'points': [[-0.2, 0.0], [0.2, 0.0]]}])
    m = z.rasterize(zones, 10, 10, 0.05, -0.25, -0.25)
    row = 5                                         # y = 0 → 행 5
    assert all(m[row, c] == 100 for c in range(1, 10))   # x −0.2~0.2 → 열 1~9 끊김 없음
    assert m[0].sum() == 0 and m[9].sum() == 0


def test_mapstore_zones_roundtrip_and_cleanup(tmp_path):
    d = tmp_path
    (d / 'room.pgm').write_bytes(b'P5\n4 3\n255\n' + bytes([254] * 12))
    (d / 'room.yaml').write_text('image: room.pgm\nresolution: 0.050000\norigin: [-1.0, -2.0, 0]\n')
    s = MapStore(str(d))
    assert s.geometry('room') == {'width': 4, 'height': 3, 'resolution': 0.05, 'origin_x': -1.0, 'origin_y': -2.0}
    s.set_zones('room', [{'kind': 'wall', 'points': [[0, 0], [1, 1]]}])
    assert s.get_zones('room')[0]['kind'] == 'wall' and s.list()[0]['zones'] == 1
    with pytest.raises(FileNotFoundError):
        s.set_zones('none', [])
    s.delete('room')
    assert not os.path.exists(d / 'room.zones.json')
