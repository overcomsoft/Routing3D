# -*- coding: utf-8 -*-
"""관경(diameter) 반영 테스트 — 호칭경 파싱 / 관경 우선순위 / 관경 기반 이격.

[실행]
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_diameter.py -v
"""
from __future__ import annotations

import pytest

from routing3d_py.scene import RouteTask, parse_pipe_size_mm
from routing3d_py.occupancy import DenseOccupancyMap
from routing3d_py.multi_route import order_tasks, route_sequential, _task_radius


def _mk(dia: float, end_x: float = 100.0, name: str = "t") -> RouteTask:
    """관경 dia, 길이 end_x 인 작업(나머지 필드는 더미)."""
    return RouteTask(
        start_mm=(0.0, 0.0, 0.0), end_mm=(end_x, 0.0, 0.0),
        utility="u", utility_group="g",
        start_name=name, end_name="e", end_instance_guid=None,
        diameter_mm=dia,
    )


def _occ(size_mm: float = 4000.0, cell: float = 100.0) -> DenseOccupancyMap:
    return DenseOccupancyMap.from_world_bounds(
        (0.0, 0.0, 0.0), (size_mm, size_mm, cell), cell_mm=cell)


# ----------------------------------------------------------------- 호칭경 파싱

@pytest.mark.parametrize("s, mm", [
    ("40A", 40.0), ("150A", 150.0), ("100A", 100.0),
    ("1/2B", 12.7), ("1B", 25.4), ("1/4B", 6.35), ("3/8B", 9.525),
    ("1-1/4B", 31.75), ("1/4BX1/2B", 6.35),
    ("65", 65.0), ("", 0.0), (None, 0.0), ("garbage", 0.0),
])
def test_parse_pipe_size_mm(s, mm):
    assert parse_pipe_size_mm(s) == pytest.approx(mm)


# ----------------------------------------------------------------- 관경 우선순위

def test_order_diameter_largest_first():
    occ = _occ()
    tasks = [_mk(25.0, name="small"), _mk(150.0, name="big"), _mk(80.0, name="mid")]
    ordered = order_tasks(occ, tasks, "diameter")
    assert [t.start_name for t in ordered] == ["big", "mid", "small"]


def test_order_diameter_tiebreak_longest():
    occ = _occ()
    tasks = [_mk(50.0, end_x=3000.0, name="long"), _mk(50.0, end_x=1000.0, name="short")]
    ordered = order_tasks(occ, tasks, "diameter")
    assert [t.start_name for t in ordered] == ["long", "short"]


# ----------------------------------------------------------------- 관경→이격 반경

def test_task_radius_not_size_aware():
    assert _task_radius(_mk(300.0), 100.0, base_radius=1,
                        size_aware=False, clearance_mm=0.0) == 1


def test_task_radius_scales_with_diameter():
    thin = _task_radius(_mk(50.0), 100.0, base_radius=0, size_aware=True, clearance_mm=0.0)
    thick = _task_radius(_mk(600.0), 100.0, base_radius=0, size_aware=True, clearance_mm=0.0)
    assert thin == 1          # ceil(25/100)
    assert thick == 3         # ceil(300/100)
    assert thick > thin


def test_task_radius_unknown_falls_back():
    assert _task_radius(_mk(0.0), 100.0, base_radius=2,
                        size_aware=True, clearance_mm=0.0) == 2


def test_task_radius_clearance_adds():
    # 반경 25mm + 이격 200mm = 225 → ceil(225/100)=3
    assert _task_radius(_mk(50.0), 100.0, base_radius=0,
                        size_aware=True, clearance_mm=200.0) == 3


# ------------------------------------------------- 통합: 굵은 배관이 더 넓게 막음

def test_size_aware_blocks_more_cells():
    occ = _occ()
    tasks = [RouteTask(start_mm=(50.0, 2000.0, 50.0), end_mm=(3800.0, 2000.0, 50.0),
                       utility="u", utility_group="g", start_name="big",
                       end_name="e", end_instance_guid=None, diameter_mm=600.0)]
    base = route_sequential(occ, tasks, priority="original", size_aware_radius=False)
    aware = route_sequential(occ, tasks, priority="original", size_aware_radius=True)
    assert base.success_count == 1
    assert aware.success_count == 1
    assert aware.occupancy.count_blocked() > base.occupancy.count_blocked()
