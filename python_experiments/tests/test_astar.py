"""직교 A* 단위 테스트 — Phase 1 Step 1.2
================================================================================

[실행 명령어]  (프로젝트 루트 또는 python_experiments/ 에서)
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_astar.py -v

[검증 범위]
  - 빈 공간 직선: 길이 = 맨해튼 거리, 회전 0
  - 장애물 우회(L자/벽): 경로가 장애물을 피하고 직선보다 길다
  - 도달 불가: 벽으로 막히면 실패
  - 경계: start==goal, start/goal 점유, 격자 밖
  - 백엔드 무관: 세 백엔드에서 동일 결과
================================================================================
"""

import pytest

from routing3d_py.astar import astar, astar_world, count_turns, manhattan
from routing3d_py.occupancy import (
    AABB,
    BitPackedOccupancyMap,
    DenseOccupancyMap,
    SparseOccupancyMap,
)

ALL_BACKENDS = [DenseOccupancyMap, SparseOccupancyMap, BitPackedOccupancyMap]


@pytest.fixture(params=ALL_BACKENDS, ids=lambda c: c.__name__)
def backend(request):
    return request.param


# ----------------------------------------------------------------- 보조 함수

def test_manhattan():
    assert manhattan((0, 0, 0), (3, 0, 0)) == 3
    assert manhattan((1, 2, 3), (4, 0, 5)) == 3 + 2 + 2


def test_count_turns_straight():
    path = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
    assert count_turns(path) == 0


def test_count_turns_one_bend():
    path = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (2, 1, 0), (2, 2, 0)]
    assert count_turns(path) == 1


# ----------------------------------------------------------------- 기본 탐색

def test_straight_line_empty(backend):
    m = backend(shape=(10, 1, 1), cell_mm=50)
    res = astar(m, (0, 0, 0), (9, 0, 0))
    assert res.success
    assert res.path[0] == (0, 0, 0)
    assert res.path[-1] == (9, 0, 0)
    assert len(res.path) == 10          # 0..9
    assert res.length_mm == pytest.approx(9 * 50)
    assert res.turns == 0


def test_length_equals_manhattan_empty(backend):
    m = backend(shape=(8, 8, 8), cell_mm=50)
    start, goal = (0, 0, 0), (7, 5, 3)
    res = astar(m, start, goal)
    assert res.success
    # 빈 공간 최단 길이 = 맨해튼 거리 * cell
    assert res.length_mm == pytest.approx(manhattan(start, goal) * 50)
    # 경로 셀은 모두 인접(단일 step)
    for a, b in zip(res.path, res.path[1:]):
        assert manhattan(a, b) == 1


def test_start_equals_goal(backend):
    m = backend(shape=(5, 5, 5))
    res = astar(m, (2, 2, 2), (2, 2, 2))
    assert res.success
    assert res.path == [(2, 2, 2)]
    assert res.length_mm == 0
    assert res.turns == 0


def test_detour_around_wall(backend):
    # 2D 평면(z=0)에 y축을 가로지르는 벽을 세우고 한 칸만 통로를 둔다.
    m = backend(shape=(7, 7, 1), cell_mm=50)
    # x=3 열을 모두 막되 (3,6,0) 한 칸만 열어둔다 → 우회 강제.
    for y in range(6):
        m.block_cell((3, y, 0))
    res = astar(m, (0, 0, 0), (6, 0, 0))
    assert res.success
    # 직선이라면 6칸(=300mm)이지만 우회하므로 더 길어야 한다.
    assert res.length_mm > 6 * 50
    # 경로는 막힌 셀을 지나지 않는다.
    for c in res.path:
        assert not m.is_blocked(c)
    assert res.turns >= 2  # 위로 올라갔다 내려오므로 최소 2회 회전


def test_unreachable_blocked_wall(backend):
    # x=3 평면을 z 전체에 걸쳐 완전히 막아 시작/목표를 분리.
    m = backend(shape=(7, 5, 3), cell_mm=50)
    for y in range(5):
        for z in range(3):
            m.block_cell((3, y, z))
    res = astar(m, (0, 0, 0), (6, 0, 0))
    assert not res.success
    assert res.path is None
    assert res.expanded_nodes > 0


def test_start_blocked_fails(backend):
    m = backend(shape=(5, 5, 5))
    m.block_cell((0, 0, 0))
    res = astar(m, (0, 0, 0), (4, 4, 4))
    assert not res.success
    assert res.path is None


def test_goal_blocked_fails(backend):
    m = backend(shape=(5, 5, 5))
    m.block_cell((4, 4, 4))
    res = astar(m, (0, 0, 0), (4, 4, 4))
    assert not res.success


def test_out_of_bounds_fails(backend):
    m = backend(shape=(5, 5, 5))
    res = astar(m, (0, 0, 0), (10, 0, 0))   # goal 격자 밖
    assert not res.success


def test_collect_visited(backend):
    m = backend(shape=(6, 6, 1), cell_mm=50)
    res = astar(m, (0, 0, 0), (5, 5, 0), collect_visited=True)
    assert res.success
    assert res.visited is not None
    assert len(res.visited) == res.expanded_nodes
    assert (0, 0, 0) in res.visited


def test_max_expansions_aborts(backend):
    m = backend(shape=(50, 50, 1), cell_mm=50)
    res = astar(m, (0, 0, 0), (49, 49, 0), max_expansions=5)
    assert not res.success
    assert res.expanded_nodes <= 5


def test_astar_world_uses_world_coords():
    m = DenseOccupancyMap(shape=(10, 1, 1), origin=(1000, 0, 0), cell_mm=50)
    # 월드 1025mm → 셀 0, 1475mm → 셀 9
    res = astar_world(m, (1025, 25, 25), (1475, 25, 25))
    assert res.success
    assert res.path[0] == (0, 0, 0)
    assert res.path[-1] == (9, 0, 0)


def test_backends_agree_on_path_length():
    """세 백엔드에서 같은 장면의 경로 길이·회전이 동일해야 한다."""
    results = []
    for cls in ALL_BACKENDS:
        m = cls(shape=(7, 7, 1), cell_mm=50)
        for y in range(6):
            m.block_cell((3, y, 0))
        r = astar(m, (0, 0, 0), (6, 0, 0))
        results.append((r.success, r.length_mm, r.turns))
    assert len(set(results)) == 1, f"backend mismatch: {results}"


def test_path_avoids_inflated_obstacle():
    # 장애물을 1셀 팽창시키면 경로가 더 멀리 우회해야 한다(클리어런스 확인).
    base = DenseOccupancyMap(shape=(9, 9, 1), cell_mm=50)
    base.block_cell((4, 4, 0))
    inflated = base.inflate(1, connectivity=6)
    res = astar(inflated, (0, 4, 0), (8, 4, 0))
    assert res.success
    # 팽창된 십자 장애물(중심+상하좌우)을 피해가야 함.
    for c in res.path:
        assert not inflated.is_blocked(c)
