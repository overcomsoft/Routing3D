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
    if priority == "diameter":
        # 관경 큰 것 먼저(메인 랙 공간 선점), 같은 관경이면 긴 것 먼저.
        #   실제 설계 관행: 굵은 배관이 주 경로를 먼저 차지하고 가는 배관이 분기.
        return sorted(tasks, key=lambda t: (-getattr(t, "diameter_mm", 0.0), -dist(t)))
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
    size_aware_radius: bool = False,
    clearance_mm: float = 0.0,
) -> MultiRouteResult:
    """배관들을 순차적으로(충돌 없이) 라우팅한다.

    매개변수:
        occ              : 장애물 점유맵(원본). 내부에서 사본을 만들어 사용(원본 불변).
        tasks            : 라우팅 작업 리스트.
        params           : RouteParams. None 이면 기본(cell_mm=occ.cell_mm).
        priority         : 우선순위 규칙(order_tasks 참조. "diameter"=굵은 것 먼저).
        pipe_radius      : 깔린 배관을 점유로 추가할 때 팽창 반경(셀). 0=경로 셀만.
                           >0 이면 배관 굵기/이격을 흉내내 더 넓게 막는다.
        snap_to_free     : start/end 가 점유면 빈 셀 탐색 반경(셀).
        max_expansions   : 배관당 A* 확장 상한(폭주 방지).
        size_aware_radius: True 면 배관 관경(diameter_mm) 비례 반경으로 막는다
                           (굵은 배관=넓게). 관경 미상이면 pipe_radius.
        clearance_mm     : size_aware 시 반경에 더할 이격(mm).
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
            r = _task_radius(task, occ.cell_mm, pipe_radius, size_aware_radius, clearance_mm)
            _mark_pipe(work, res.path, r)

    return MultiRouteResult(pipes=pipes, occupancy=work, priority=priority)


# ------------------------------------------------------------------ rip-up & reroute

def route_ripup(
    occ: OccupancyMap,
    tasks: list[RouteTask],
    params: RouteParams | None = None,
    *,
    priority: str = "longest",
    pipe_radius: int = 0,
    snap_to_free: int = 2,
    max_expansions: int | None = None,
    max_rounds: int = 10,
    max_ripup: int = 4,
    size_aware_radius: bool = False,
    clearance_mm: float = 0.0,
) -> MultiRouteResult:
    """순차 라우팅 후 **rip-up & reroute** 로 실패 배관을 해소한다(Phase 3 Step 3.8).

    [알고리즘 — 결정적 · 단조증가(채택 시 항상 성공 +1)]
      1) route_sequential 과 동일한 베이스라인을 깐다(placed = 성공 배관 경로 맵).
      2) 라운드 반복(최대 max_rounds):
         실패 배관 f(우선순위 순)마다:
           a) '장애물만' 점유맵에서 f 의 이상(ideal) 경로를 구한다. 실패하면 f 는
              장애물만으로도 불가 → 영구 실패로 두고 건너뛴다.
           b) 이상 경로가 가로지르는 기존 배관 = blocker 집합. 없거나 max_ripup 초과면 건너뛴다.
           c) blocker 들을 뜯어내고(placed 에서 제거) f 를 배치한 뒤, 뜯어낸 blocker 를
              (결정적 순서로) 모두 재라우팅한다.
           d) **f 성공 + 모든 blocker 재배치 성공** 일 때만 채택(무손실 → 성공 수 +1).
              하나라도 실패하면 그 시도를 통째로 버린다(원상 복귀).
         한 라운드에서 채택이 하나도 없으면 종료.

    blocker 를 전부 되살릴 수 있을 때만 받아들이므로 성공 수는 절대 줄지 않고 단조 증가하며,
    채택마다 +1 이라 라운드/시도는 유한하다(초기 실패 수로 상한). 진짜 혼잡/불가 배관은
    그대로 실패로 남는다(올바른 동작).

    매개변수(route_sequential 과 동일 + ):
        max_rounds : 전체 rip-up 라운드 상한.
        max_ripup  : 한 번에 뜯어낼 blocker 수 상한(너무 큰 교란 방지).
    반환값:
        MultiRouteResult. pipes 는 우선순위 정렬 순서(order_index = 그 순서).

    지역 변수:
        static  : 장애물만의 기준 점유맵(불변).
        placed  : 정렬 인덱스 → 배관 경로(현재 깔린 것).
        results : 정렬 인덱스 → 그 배관의 최종 AStarResult.
    """
    params = params or RouteParams(cell_mm=occ.cell_mm)
    ordered = order_tasks(occ, tasks, priority)
    n = len(ordered)
    static = occ.copy()  # 장애물만(불변 기준).

    # 정렬 인덱스별 점유 반경(관경 인지 시 배관마다 다름). _build_work·인라인 마킹이 공유.
    radii = {
        idx: _task_radius(ordered[idx], occ.cell_mm, pipe_radius, size_aware_radius, clearance_mm)
        for idx in range(n)
    }

    placed: dict[int, list[Cell]] = {}
    results: dict[int, AStarResult] = {}

    def _route(w: OccupancyMap, task: RouteTask) -> AStarResult:
        s = _snap(w, w.to_cell(task.start_mm), snap_to_free)
        g = _snap(w, w.to_cell(task.end_mm), snap_to_free)
        return astar_weighted(w, s, g, params, max_expansions=max_expansions)

    def _build_work(paths: dict[int, list[Cell]]) -> OccupancyMap:
        w = static.copy()
        for idx, p in paths.items():
            _mark_pipe(w, p, radii[idx])
        return w

    # 1) 베이스라인 순차 라우팅(route_sequential 과 동일).
    work = static.copy()
    for idx, task in enumerate(ordered):
        res = _route(work, task)
        results[idx] = res
        if res.success and res.path:
            placed[idx] = res.path
            _mark_pipe(work, res.path, radii[idx])

    # 2) rip-up 라운드.
    for _round in range(max_rounds):
        failed = [idx for idx in range(n) if idx not in placed]
        if not failed:
            break
        changed = False
        for f in failed:
            task_f = ordered[f]
            ideal = _route(static, task_f)  # 장애물만의 이상 경로.
            if not (ideal.success and ideal.path):
                continue  # 장애물만으로도 불가 → 영구 실패.
            cellset = set(ideal.path)
            blockers = sorted(b for b, p in placed.items() if any(c in cellset for c in p))
            if not blockers or len(blockers) > max_ripup:
                continue

            trial = dict(placed)
            for b in blockers:
                del trial[b]
            wt = _build_work(trial)

            rf = _route(wt, task_f)  # 뜯어낸 자리에 f 배치.
            if not (rf.success and rf.path):
                continue
            trial[f] = rf.path
            _mark_pipe(wt, rf.path, radii[f])

            reres: dict[int, AStarResult] = {}
            all_ok = True
            for b in blockers:  # 뜯어낸 배관 재라우팅(결정적 순서).
                rb = _route(wt, ordered[b])
                reres[b] = rb
                if rb.success and rb.path:
                    trial[b] = rb.path
                    _mark_pipe(wt, rb.path, radii[b])
                else:
                    all_ok = False

            if all_ok:  # 무손실일 때만 채택(성공 수 +1).
                placed = trial
                results[f] = rf
                for b in blockers:
                    results[b] = reres[b]
                changed = True
        if not changed:
            break

    pipes = [PipeResult(task=ordered[idx], result=results[idx], order_index=idx) for idx in range(n)]
    return MultiRouteResult(pipes=pipes, occupancy=_build_work(placed), priority=priority)


def _task_radius(
    task: RouteTask, cell_mm: float, base_radius: int,
    size_aware: bool, clearance_mm: float,
) -> int:
    """배관 1개를 점유로 표시할 팽창 반경(셀)을 정한다.

    size_aware=False 거나 관경 미상 → base_radius(고정) 그대로.
    size_aware=True → 관경 비례: ceil((반경 + 이격) / cell), base_radius 와 max.
      굵은 배관일수록 더 넓게 막아 실제 이격을 흉내(가는 배관은 좁게).
    """
    if not size_aware or getattr(task, "diameter_mm", 0.0) <= 0:
        return base_radius
    import math
    cells = math.ceil((task.diameter_mm / 2.0 + clearance_mm) / cell_mm)
    return max(base_radius, cells)


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
