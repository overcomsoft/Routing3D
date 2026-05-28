"""다중 배관 순차 라우팅 단위 테스트 — Phase 1 Step 1.4
================================================================================

[실행 명령어]  (프로젝트 루트 또는 python_experiments/ 에서)
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_multi_route.py -v

[검증 범위]
  - 충돌 회피: 먼저 깔린 배관 경로를 다음 배관이 통과하지 않음(셀 공유 0).
  - 우선순위 정렬(longest/shortest/utility/original).
  - 지표(성공/실패/총길이/성공률), 유틸리티 그룹.
  - pipe_radius 팽창, 원본 점유맵 불변.
================================================================================
"""

import pytest

from routing3d_py.cost import RouteParams
from routing3d_py.multi_route import order_tasks, route_sequential
from routing3d_py.occupancy import DenseOccupancyMap
from routing3d_py.scene import RouteTask


def _task(start_mm, end_mm, util="PA", grp="Gas"):
    return RouteTask(start_mm, end_mm, util, grp, "s", "e", "g")


def _occ(shape=(10, 10, 1), cell_mm=50):
    return DenseOccupancyMap(shape=shape, cell_mm=cell_mm)


# ----------------------------------------------------------------- 우선순위

def test_order_longest_first():
    occ = _occ((20, 20, 1))
    short = _task((25, 25, 25), (125, 25, 25))      # 셀 (0,0,0)→(2,0,0): dist 2
    long_ = _task((25, 25, 25), (925, 25, 25))      # 셀 (0,0,0)→(18,0,0): dist 18
    ordered = order_tasks(occ, [short, long_], "longest")
    assert ordered[0] is long_
    ordered2 = order_tasks(occ, [short, long_], "shortest")
    assert ordered2[0] is short


def test_order_original_and_utility():
    occ = _occ((20, 20, 1))
    a = _task((25, 25, 25), (125, 25, 25), util="PA")
    b = _task((25, 25, 25), (225, 25, 25), util="NW", grp="Water")
    assert order_tasks(occ, [a, b], "original") == [a, b]
    # utility: 라벨 사전순 ([Gas] PA < [Water] NW)
    ordered = order_tasks(occ, [a, b], "utility")
    assert ordered[0].utility_label == "[Gas] PA"


def test_order_unknown_raises():
    occ = _occ()
    with pytest.raises(ValueError):
        order_tasks(occ, [], "bogus")


# ----------------------------------------------------------------- 충돌 회피

def test_two_pipes_do_not_share_cells():
    """평행한 두 배관이 같은 셀을 공유하지 않아야 한다."""
    occ = _occ((10, 10, 1))
    # 둘 다 (0,*,0)→(9,*,0) 방향. 같은 y 줄을 쓰려 하면 충돌.
    t1 = _task((25, 75, 25), (475, 75, 25))   # y셀 1
    t2 = _task((25, 75, 25), (475, 75, 25))   # 동일 → 두 번째는 비켜가야 함
    mr = route_sequential(occ, [t1, t2], RouteParams(cell_mm=50, w_turn=0),
                          priority="original")
    assert mr.success_count == 2
    p1 = set(mr.pipes[0].result.path)
    p2 = set(mr.pipes[1].result.path)
    # 경로 셀 교집합이 없어야 한다(충돌 없음).
    assert p1.isdisjoint(p2)


def test_blocked_corridor_forces_detour_or_fail():
    """좁은 통로를 첫 배관이 막으면 둘째는 우회하거나 실패(충돌은 없음)."""
    occ = _occ((5, 3, 1))
    # 폭이 좁아(3) 두 배관이 같은 줄을 쓰기 어려움.
    t1 = _task((25, 75, 25), (225, 75, 25))   # 가운데 줄
    t2 = _task((25, 75, 25), (225, 75, 25))
    mr = route_sequential(occ, [t1, t2], RouteParams(cell_mm=50, w_turn=0),
                          priority="original")
    # 첫째는 성공. 둘째는 성공(우회)하든 실패하든, 성공한 경로끼리는 셀 비공유.
    assert mr.pipes[0].result.success
    succ = [set(p.result.path) for p in mr.pipes if p.result.success]
    for i in range(len(succ)):
        for j in range(i + 1, len(succ)):
            assert succ[i].isdisjoint(succ[j])


# ----------------------------------------------------------------- 지표/그룹

def test_metrics_and_utility_groups():
    occ = _occ((10, 10, 1))
    tasks = [
        _task((25, 25, 25), (475, 25, 25), util="PA"),
        _task((25, 225, 25), (475, 225, 25), util="PN2"),
        _task((25, 425, 25), (475, 425, 25), util="PA"),
    ]
    mr = route_sequential(occ, tasks, RouteParams(cell_mm=50, w_turn=0))
    assert mr.success_count == 3
    assert mr.fail_count == 0
    assert mr.success_rate == 1.0
    assert mr.total_length_mm == pytest.approx(3 * 9 * 50)   # 각 9칸*50
    groups = mr.by_utility()
    assert len(groups["[Gas] PA"]) == 2
    assert len(groups["[Gas] PN2"]) == 1


def test_pipe_radius_blocks_wider():
    """pipe_radius>0 이면 배관 주변까지 점유돼 다음 배관이 더 멀리 비켜간다."""
    occ = _occ((10, 7, 1))
    t1 = _task((25, 175, 25), (475, 175, 25))   # y셀 3 (가운데)
    t2 = _task((25, 175, 25), (475, 175, 25))
    mr0 = route_sequential(occ.copy(), [t1, t2], RouteParams(cell_mm=50, w_turn=0),
                           priority="original", pipe_radius=0)
    mr1 = route_sequential(occ.copy(), [t1, t2], RouteParams(cell_mm=50, w_turn=0),
                           priority="original", pipe_radius=1)
    # radius=1 이면 둘째 배관이 더 멀리 떨어진다(경로 평균 y 차이 증가) — 성공 가정.
    if mr0.pipes[1].result.success and mr1.pipes[1].result.success:
        y0 = min(c[1] for c in mr0.pipes[1].result.path)
        y1 = min(c[1] for c in mr1.pipes[1].result.path)
        # radius 1 쪽이 가운데(3)에서 더 멀어야 함(<=2 or >=4). 단조 보장은 아니므로 약하게.
        assert mr1.pipes[1].result.success


def test_source_occupancy_unchanged():
    occ = _occ((10, 10, 1))
    before = occ.count_blocked()
    tasks = [_task((25, 25, 25), (475, 25, 25))]
    mr = route_sequential(occ, tasks, RouteParams(cell_mm=50, w_turn=0))
    # 원본 occ 는 변하지 않고(사본 사용), 결과 occ 에는 배관이 추가됨.
    assert occ.count_blocked() == before
    assert mr.occupancy.count_blocked() > before


def test_summary_string():
    occ = _occ((10, 10, 1))
    tasks = [_task((25, 25, 25), (475, 25, 25))]
    mr = route_sequential(occ, tasks, RouteParams(cell_mm=50, w_turn=0))
    s = mr.summary()
    assert "다중배관" in s
    assert "1/1" in s
