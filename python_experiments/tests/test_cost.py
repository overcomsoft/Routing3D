"""비용함수 단위 테스트 — Phase 1 Step 1.3
================================================================================

[실행 명령어]  (프로젝트 루트 또는 python_experiments/ 에서)
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_cost.py -v

[검증 범위]
  - RouteParams 검증(음수 가중치 거부)
  - clearance_map: 장애물까지 거리(셀)·상한·경계
  - CostModel.move_cost: 회전 페널티, 클리어런스 근접 페널티, 단 분리
  - astar_weighted:
      · 가중치 0 → 기본 astar 와 길이 동일
      · w_turn>0 → 회전 최소화(L자 1회전)
      · w_clear>0 → 장애물에서 더 떨어져 우회
      · admissibility(최적성): 알려진 최적 비용과 일치
================================================================================
"""

import pytest

from routing3d_py.astar import astar, astar_weighted
from routing3d_py.cost import CostModel, RouteParams, clearance_map
from routing3d_py.occupancy import DenseOccupancyMap


# ----------------------------------------------------------------- RouteParams

def test_routeparams_defaults():
    p = RouteParams()
    assert p.cell_mm == 50.0
    assert p.w_turn == 500.0
    assert p.w_clear == 10.0
    assert p.clearance_radius == 2


@pytest.mark.parametrize("kwargs", [
    {"cell_mm": 0},
    {"w_turn": -1},
    {"w_clear": -5},
    {"clearance_radius": -1},
    {"clearance_connectivity": 18},
])
def test_routeparams_invalid(kwargs):
    with pytest.raises(ValueError):
        RouteParams(**kwargs)


# ----------------------------------------------------------------- clearance_map

def test_clearance_map_single_obstacle():
    m = DenseOccupancyMap(shape=(9, 9, 1), cell_mm=50)
    m.block_cell((4, 4, 0))
    dist = clearance_map(m, max_radius=2, connectivity=6)
    assert dist[4, 4, 0] == 0          # 장애물 자신
    assert dist[5, 4, 0] == 1          # 면 인접
    assert dist[6, 4, 0] == 2          # 2칸
    assert dist[8, 4, 0] == 2          # 멀어도 상한 2
    assert dist[4, 4, 0] >= 0


def test_clearance_map_zero_radius():
    m = DenseOccupancyMap(shape=(5, 5, 1))
    m.block_cell((2, 2, 0))
    dist = clearance_map(m, max_radius=0)
    assert dist[2, 2, 0] == 0
    assert dist[0, 0, 0] == 0          # 상한 0 → 모두 0


def test_clearance_map_empty():
    m = DenseOccupancyMap(shape=(5, 5, 1))
    dist = clearance_map(m, max_radius=3)
    # 장애물이 없으면 모든 셀이 상한값.
    assert (dist == 3).all()


# ----------------------------------------------------------------- CostModel

def test_move_cost_no_turn():
    m = DenseOccupancyMap(shape=(5, 5, 5))
    model = CostModel(m, RouteParams(w_turn=500, w_clear=0))
    # 같은 방향(+x)으로 직진 → 회전 페널티 없음
    c = model.move_cost((2, 0, 0), prev_off=(1, 0, 0), move_off=(1, 0, 0))
    assert c == pytest.approx(50.0)


def test_move_cost_turn_penalty():
    m = DenseOccupancyMap(shape=(5, 5, 5))
    model = CostModel(m, RouteParams(w_turn=500, w_clear=0))
    # +x 로 오다가 +y 로 꺾음 → 회전 페널티
    c = model.move_cost((1, 1, 0), prev_off=(1, 0, 0), move_off=(0, 1, 0))
    assert c == pytest.approx(50.0 + 500.0)


def test_move_cost_first_move_no_turn():
    m = DenseOccupancyMap(shape=(5, 5, 5))
    model = CostModel(m, RouteParams(w_turn=500, w_clear=0))
    # 시작(prev_off=None) → 회전 페널티 없음
    c = model.move_cost((1, 0, 0), prev_off=None, move_off=(1, 0, 0))
    assert c == pytest.approx(50.0)


def test_move_cost_clearance_penalty():
    m = DenseOccupancyMap(shape=(9, 9, 1), cell_mm=50)
    m.block_cell((4, 4, 0))
    model = CostModel(m, RouteParams(w_turn=0, w_clear=10, clearance_radius=2))
    # (5,4,0) 은 장애물에 거리 1 → 페널티 = 10*(2-1)=10
    c_near = model.move_cost((5, 4, 0), prev_off=(1, 0, 0), move_off=(1, 0, 0))
    assert c_near == pytest.approx(50.0 + 10.0)
    # 멀리 떨어진 셀(거리 >=2) → 페널티 0
    c_far = model.move_cost((0, 0, 0), prev_off=(1, 0, 0), move_off=(1, 0, 0))
    assert c_far == pytest.approx(50.0)


def test_move_cost_tier_penalty():
    m = DenseOccupancyMap(shape=(5, 5, 5))
    model = CostModel(m, RouteParams(w_turn=0, w_clear=0, w_tier={3: 200.0}))
    c = model.move_cost((1, 1, 3), prev_off=(1, 0, 0), move_off=(1, 0, 0))
    assert c == pytest.approx(50.0 + 200.0)


# ----------------------------------------------------------------- astar_weighted

def test_weighted_uniform_matches_basic():
    """가중치 0 이면 기본 astar 와 길이가 같아야 한다."""
    m = DenseOccupancyMap(shape=(8, 8, 1), cell_mm=50)
    params = RouteParams(w_turn=0, w_clear=0)
    base = astar(m, (0, 0, 0), (7, 5, 0))
    weighted = astar_weighted(m, (0, 0, 0), (7, 5, 0), params)
    assert weighted.success
    assert weighted.length_mm == pytest.approx(base.length_mm)


def test_weighted_minimizes_turns():
    """w_turn>0 이면 (0,0)→(3,3) 최단 경로 중 회전이 최소(L자 1회전)여야 한다."""
    m = DenseOccupancyMap(shape=(4, 4, 1), cell_mm=50)
    res = astar_weighted(m, (0, 0, 0), (3, 3, 0), RouteParams(w_turn=500, w_clear=0))
    assert res.success
    assert res.length_mm == pytest.approx(6 * 50)   # 맨해튼 최단 유지
    assert res.turns == 1                            # 회전 최소
    # 총 비용 = 길이 + 회전 1회 페널티
    assert res.cost_mm == pytest.approx(6 * 50 + 500)


def test_weighted_keeps_clearance():
    """w_clear>0 이면 장애물에서 더 떨어져 우회해야 한다."""
    # 가운데 단일 장애물. 직선 경로는 장애물을 통과하므로 우회 필요.
    m = DenseOccupancyMap(shape=(11, 11, 1), cell_mm=50)
    m.block_cell((5, 5, 0))
    start, goal = (0, 5, 0), (10, 5, 0)
    clear = clearance_map(m, max_radius=3, connectivity=6)

    base = astar(m, start, goal)
    weighted = astar_weighted(m, start, goal,
                              RouteParams(w_turn=0, w_clear=50, clearance_radius=3))
    assert base.success and weighted.success
    # 경로상 최소 클리어런스(장애물과의 최소 거리)가 가중 경로에서 더 커야 함.
    min_clear_base = min(int(clear[c]) for c in base.path)
    min_clear_weighted = min(int(clear[c]) for c in weighted.path)
    assert min_clear_weighted > min_clear_base


def test_weighted_unreachable():
    m = DenseOccupancyMap(shape=(7, 5, 1), cell_mm=50)
    for y in range(5):
        m.block_cell((3, y, 0))
    res = astar_weighted(m, (0, 0, 0), (6, 0, 0), RouteParams())
    assert not res.success
    assert res.path is None


def test_weighted_start_equals_goal():
    m = DenseOccupancyMap(shape=(5, 5, 5))
    res = astar_weighted(m, (2, 2, 2), (2, 2, 2), RouteParams())
    assert res.success
    assert res.path == [(2, 2, 2)]
    assert res.cost_mm == 0


def test_weighted_optimal_cost_known():
    """알려진 최적 비용과 일치(admissible 휴리스틱 → 최적)."""
    # 빈 5x1x1: (0,0,0)→(4,0,0) 직진. 회전 0, 클리어런스 없음.
    m = DenseOccupancyMap(shape=(5, 1, 1), cell_mm=50)
    res = astar_weighted(m, (0, 0, 0), (4, 0, 0), RouteParams(w_turn=500))
    assert res.cost_mm == pytest.approx(4 * 50)   # 회전 0 → 페널티 없음
    assert res.turns == 0
