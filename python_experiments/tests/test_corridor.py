# -*- coding: utf-8 -*-
"""회랑(corridor) 인력 비용 — 기존 설계처럼 공용 랙으로 배관을 뭉치게.

[실행]
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_corridor.py -v
"""
from __future__ import annotations

import pytest

from routing3d_py.cost import RouteParams, CostModel
from routing3d_py.occupancy import DenseOccupancyMap
from routing3d_py.scene import RouteTask
from routing3d_py.multi_route import route_sequential


def _occ(size_mm: float = 3000.0, cell: float = 100.0) -> DenseOccupancyMap:
    return DenseOccupancyMap.from_world_bounds(
        (0.0, 0.0, 0.0), (size_mm, size_mm, cell), cell_mm=cell)


# ----------------------------------------------------------------- 검증

def test_w_corridor_negative_rejected():
    with pytest.raises(ValueError):
        RouteParams(w_corridor=-1.0)


# ----------------------------------------------------------------- cell_penalty

def test_cell_penalty_off_corridor_charged():
    occ = _occ()
    p = RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0, w_corridor=100.0)
    cm = CostModel(occ, p, corridor={(5, 5, 0)})
    assert cm.cell_penalty((5, 5, 0)) == 0.0      # 회랑 안 → 면제
    assert cm.cell_penalty((1, 1, 0)) == 100.0     # 회랑 밖 → 가산


def test_cell_penalty_rack_level_exempt():
    occ = _occ()
    p = RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0, w_corridor=100.0,
                    rack_levels=(0,))
    cm = CostModel(occ, p, corridor=None)
    assert cm.cell_penalty((9, 9, 0)) == 0.0       # z=0 은 rack → 면제
    # rack 아닌 z 에서 회랑도 없으면 가산(여기선 nz=1 라 z=0 뿐 → 개념 확인용)


def test_cell_penalty_disabled_when_zero():
    occ = _occ()
    p = RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0, w_corridor=0.0)
    cm = CostModel(occ, p, corridor={(5, 5, 0)})
    assert cm.cell_penalty((1, 1, 0)) == 0.0       # 비활성 → 가산 없음


# ----------------------------------------------- 통합: 배관이 공용 회랑으로 뭉침

def _path_cells(res):
    return set(res.result.path) if res.result.success else set()


def test_corridor_bundles_second_pipe_near_first():
    """w_corridor 켜면 두 번째 배관이 첫 배관 곁으로 붙는다(번들링)."""
    occ = _occ(size_mm=3000.0, cell=100.0)
    # A: y=500(row5) 수평, B: y=1500(row15) 수평 — 10셀 떨어진 평행 배관.
    a = RouteTask((100, 500, 50), (2800, 500, 50), utility="u", utility_group="g",
                  start_name="A", end_name="e", end_instance_guid=None)
    b = RouteTask((100, 1500, 50), (2800, 1500, 50), utility="u", utility_group="g",
                  start_name="B", end_name="e", end_instance_guid=None)

    # 회전비용 0 + 강한 회랑 인력 → B 가 A 곁으로 올라붙는다.
    params = RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0, w_corridor=1000.0)

    base = route_sequential(occ, [a, b], RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0),
                            priority="original")
    bundled = route_sequential(occ, [a, b], params, priority="original", corridor_radius=1)

    assert base.success_count == 2
    assert bundled.success_count == 2

    a_cells = _path_cells(base.pipes[0])
    a_near = {(i + di, j + dj, k) for (i, j, k) in a_cells
              for di in (-2, -1, 0, 1, 2) for dj in (-2, -1, 0, 1, 2)}

    def near_frac(res):
        cells = _path_cells(res)
        if not cells:
            return 0.0
        return sum(1 for c in cells if c in a_near) / len(cells)

    base_b_near = near_frac(base.pipes[1])
    bundled_b_near = near_frac(bundled.pipes[1])
    # 번들링 시 B 가 A 근처를 훨씬 많이 지난다.
    assert bundled_b_near > base_b_near
    assert bundled_b_near > 0.5


def test_corridor_increases_length_toward_design():
    """회랑 인력은 총 길이를 늘린다(기존 설계의 우회·랙 따라가기에 가까워짐)."""
    occ = _occ(size_mm=3000.0, cell=100.0)
    a = RouteTask((100, 500, 50), (2800, 500, 50), utility="u", utility_group="g",
                  start_name="A", end_name="e", end_instance_guid=None)
    b = RouteTask((100, 1500, 50), (2800, 1500, 50), utility="u", utility_group="g",
                  start_name="B", end_name="e", end_instance_guid=None)
    base = route_sequential(occ, [a, b], RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0),
                            priority="original")
    bundled = route_sequential(occ, [a, b],
                               RouteParams(cell_mm=100.0, w_turn=0.0, w_clear=0.0, w_corridor=1000.0),
                               priority="original", corridor_radius=1)
    assert bundled.total_length_mm >= base.total_length_mm
