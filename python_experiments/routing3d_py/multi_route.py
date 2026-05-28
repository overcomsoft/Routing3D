"""다중 배관 순차 라우팅 (Multi-Pipe Sequential Routing) — Phase 1 Step 1.4
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서)
  # 프로젝트 6 전체 배관을 충돌 없이 순차 라우팅 + 유틸리티별 렌더
  .\\.venv\\Scripts\\python.exe -m routing3d_py.scene --project 6 --multi --cell-mm 100 ^
      --screenshot python_experiments/out/multi.png

  # 단위 테스트
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_multi_route.py -v

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
여러 배관(start→end 작업)을 '한 개씩 차례로' 라우팅한다. 핵심은 **이미 깔린 배관을
다음 배관의 장애물로 추가**하여 배관끼리 같은 셀을 점유하지 않게(충돌 없이) 만드는
것이다. 이것이 다중 배관 라우팅의 베이스라인(greedy sequential) 전략이다.

[알고리즘 — 순차 라우팅(sequential / rip-up 없음)]
--------------------------------------------------------------------------------
  1) 우선순위 규칙으로 작업 순서를 정한다(기본: 긴 것 먼저).
  2) 장애물 점유맵의 '작업용 사본'을 만든다(원본 보존).
  3) 작업을 순서대로:
       a) start/end 가 점유면 가까운 빈 셀로 스냅(snap_to_free).
       b) 비용함수 A*(astar_weighted)로 경로 탐색.
       c) 성공하면 경로 셀(+pipe_radius 팽창)을 작업용 점유맵에 '점유'로 추가
          → 이후 배관이 이 경로를 피한다.
       d) 실패하면 기록만(다음 배관에 영향 없음).
  4) (성공률 / 총 길이 / 실패 수)를 측정한다.

[이 단계에서 하지 않는 것]
  - rip-up & reroute, CBS(충돌 기반 탐색) 등 전역 최적화 → Phase 3.
  - 따라서 혼잡한 출발부(메인장비 면에 PoC 밀집)에서는 후순위 배관이 막혀 실패할 수
    있다. 그 실패율을 '측정'하는 것이 본 단계의 목적이며, 해소는 Phase 3 과제다.

[우선순위 규칙(priority)]
  - "longest"  : 시작-끝 맨해튼 거리가 긴 배관 먼저(기본). 어려운 것 먼저.
  - "shortest" : 짧은 것 먼저.
  - "utility"  : 유틸리티 라벨로 묶은 뒤(그룹 내) 긴 것 먼저.
  - "original" : 입력 순서 유지.
  (계획서의 '직경 큰 순'은 직경 데이터가 확보되면 추가. 현재는 거리 기준.)
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass

from .astar import AStarResult, astar_weighted, manhattan
from .cost import RouteParams
from .occupancy import NEIGHBORS_6, Cell, OccupancyMap
from .scene import RouteTask


@dataclass
class PipeResult:
    """배관 1개의 라우팅 결과.

    필드:
        task        : 라우팅 작업(start→end + utility).
        result      : A* 결과(AStarResult).
        order_index : 실제 라우팅된 순서(0부터).
    """

    task: RouteTask
    result: AStarResult
    order_index: int


@dataclass
class MultiRouteResult:
    """다중 배관 순차 라우팅 결과 묶음.

    필드:
        pipes     : PipeResult 리스트(라우팅 순서).
        occupancy : 최종 작업용 점유맵(장애물 + 모든 성공 배관).
        priority  : 사용한 우선순위 규칙 이름.
    """

    pipes: list[PipeResult]
    occupancy: OccupancyMap
    priority: str

    @property
    def success_count(self) -> int:
        return sum(1 for p in self.pipes if p.result.success)

    @property
    def fail_count(self) -> int:
        return len(self.pipes) - self.success_count

    @property
    def total_length_mm(self) -> float:
        return sum(p.result.length_mm for p in self.pipes if p.result.success)

    @property
    def success_rate(self) -> float:
        return self.success_count / len(self.pipes) if self.pipes else 0.0

    def by_utility(self) -> dict[str, list[PipeResult]]:
        """결과를 유틸리티 라벨별로 묶는다."""
        groups: dict[str, list[PipeResult]] = {}
        for p in self.pipes:
            groups.setdefault(p.task.utility_label, []).append(p)
        return groups

    def summary(self) -> str:
        """사람이 읽기 좋은 한 줄 요약."""
        return (
            f"[다중배관/{self.priority}] {self.success_count}/{len(self.pipes)} 성공 "
            f"({self.success_rate:.0%}), 실패 {self.fail_count}, "
            f"총 길이 {self.total_length_mm:,.0f} mm"
        )


# ------------------------------------------------------------------ 우선순위

def order_tasks(occ: OccupancyMap, tasks: list[RouteTask], priority: str) -> list[RouteTask]:
    """우선순위 규칙에 따라 작업 순서를 정렬해 반환한다.

    매개변수:
        occ      : 셀 변환용 점유맵(거리 계산에 to_cell 사용).
        tasks    : 원본 작업 리스트.
        priority : "longest" | "shortest" | "utility" | "original".
    반환값:
        정렬된 작업 리스트(원본 변경 없음).
    """
    def dist(t: RouteTask) -> int:
        return manhattan(occ.to_cell(t.start_mm), occ.to_cell(t.end_mm))

    if priority == "original":
        return list(tasks)
    if priority == "shortest":
        return sorted(tasks, key=dist)
    if priority == "longest":
        return sorted(tasks, key=dist, reverse=True)
    if priority == "utility":
        # 유틸리티 라벨로 그룹(이름 순), 그룹 내 긴 것 먼저.
        return sorted(tasks, key=lambda t: (t.utility_label, -dist(t)))
    raise ValueError(f"unknown priority: {priority!r}")


# ------------------------------------------------------------------ 순차 라우팅

def route_sequential(
    occ: OccupancyMap,
    tasks: list[RouteTask],
    params: RouteParams | None = None,
    *,
    priority: str = "longest",
    pipe_radius: int = 0,
    snap_to_free: int = 2,
    max_expansions: int | None = None,
) -> MultiRouteResult:
    """배관들을 순차적으로(충돌 없이) 라우팅한다.

    매개변수:
        occ            : 장애물 점유맵(원본). 내부에서 사본을 만들어 사용(원본 불변).
        tasks          : 라우팅 작업 리스트.
        params         : RouteParams. None 이면 기본(cell_mm=occ.cell_mm).
        priority       : 우선순위 규칙(order_tasks 참조).
        pipe_radius    : 깔린 배관을 점유로 추가할 때 팽창 반경(셀). 0=경로 셀만.
                         >0 이면 배관 굵기/이격을 흉내내 더 넓게 막는다.
        snap_to_free   : start/end 가 점유면 빈 셀 탐색 반경(셀).
        max_expansions : 배관당 A* 확장 상한(폭주 방지).
    반환값:
        MultiRouteResult.

    지역 변수:
        work    : 작업용 점유맵 사본(장애물 + 누적된 배관).
        ordered : 우선순위 정렬된 작업.
    """
    params = params or RouteParams(cell_mm=occ.cell_mm)
    work = occ.copy()
    ordered = order_tasks(occ, tasks, priority)

    pipes: list[PipeResult] = []
    for idx, task in enumerate(ordered):
        s = _snap(work, work.to_cell(task.start_mm), snap_to_free)
        g = _snap(work, work.to_cell(task.end_mm), snap_to_free)
        res = astar_weighted(work, s, g, params, max_expansions=max_expansions)
        pipes.append(PipeResult(task=task, result=res, order_index=idx))
        if res.success and res.path:
            _mark_pipe(work, res.path, pipe_radius)

    return MultiRouteResult(pipes=pipes, occupancy=work, priority=priority)


def _mark_pipe(occ: OccupancyMap, path: list[Cell], radius: int) -> None:
    """경로 셀(+반경 radius 이웃)을 점유로 표시한다(다음 배관이 피하도록).

    radius>0 이면 6-이웃을 radius 단계 확장하며 함께 막는다(배관 굵기/이격 흉내).
    """
    for cell in path:
        occ.block_cell(cell)
    if radius <= 0:
        return
    frontier = set(path)
    for _ in range(radius):
        nxt: set[Cell] = set()
        for (i, j, k) in frontier:
            for di, dj, dk in NEIGHBORS_6:
                c = (i + di, j + dj, k + dk)
                if occ.in_bounds(c) and not occ.is_blocked(c):
                    occ.block_cell(c)
                    nxt.add(c)
        frontier = nxt


def _snap(occ: OccupancyMap, cell: Cell, radius: int) -> Cell:
    """cell 이 점유면 반경 radius 내 가장 가까운 빈 셀 반환(없으면 원래 cell)."""
    if not occ.is_blocked(cell):
        return cell
    if radius <= 0:
        return cell
    ci, cj, ck = cell
    best = None
    best_d = None
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            for dk in range(-radius, radius + 1):
                c = (ci + di, cj + dj, ck + dk)
                if not occ.is_blocked(c):
                    d = abs(di) + abs(dj) + abs(dk)
                    if best_d is None or d < best_d:
                        best_d, best = d, c
    return best if best is not None else cell
