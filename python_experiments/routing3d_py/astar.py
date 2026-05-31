"""직교 A* 탐색 (Orthogonal A*) — Phase 1 Step 1.2
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서. 미설치 시 python_experiments/ 에서)
  # ① DB 영역에서 start→goal(mm) 경로 탐색 후 지표 출력
  .\\.venv\\Scripts\\python.exe -m routing3d_py.astar ^
      --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 ^
      --start 196000 9000 15600 --goal 204000 11000 15600

  # ② 경로를 점유맵과 함께 3D 렌더(스크린샷). --show 면 인터랙티브 창.
  .\\.venv\\Scripts\\python.exe -m routing3d_py.astar ^
      --region 195000 8000 14000 205000 12000 16000 ^
      --start 196000 9000 15600 --goal 204000 11000 15600 ^
      --screenshot python_experiments/out/route.png

  # ③ 단위 테스트
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_astar.py -v

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
점유맵(OccupancyMap, 어느 백엔드든) 위에서 시작 셀 → 목표 셀까지의 최단 직교 경로를
A* 알고리즘으로 찾는다. 이동은 6방향(±X,±Y,±Z) 직교만 허용(대각선 금지).

[Step 1.2 범위]
  - 비용은 균일 이동비용(셀 1칸 = cell_mm)만 사용. turn penalty·클리어런스·단 분리
    등 비용함수 튜닝은 Step 1.3(cost.py)에서 확장한다.
  - 휴리스틱은 맨해튼 거리(셀 수) × cell_mm. 직교 격자에서 admissible & consistent.

[A* 알고리즘 흐름도]
--------------------------------------------------------------------------------
  open(우선순위 큐): f = g + h 가 작은 것부터 꺼냄
        │
        ▼ pop current
  current == goal ? ──예──▶ came_from 역추적으로 경로 복원 → 반환
        │ 아니오
        ▼ 6-이웃 nb 마다
   is_blocked(nb)? ──예──▶ 건너뜀 (장애물/격자 밖)
        │ 아니오
        ▼ g_new = g[current] + step_cost
   g_new < g[nb]? ──예──▶ g[nb]=g_new, came_from[nb]=current, open 에 push
        │
        └ open 이 비면 → 경로 없음(None)

  자료구조
    open_heap : heapq, 원소 (f, counter, cell). counter 로 안정적 tie-break.
    g         : dict[cell -> 누적 비용(mm)]. 미방문은 +∞ 취급.
    came_from : dict[cell -> 직전 cell]. 경로 복원용.
    closed    : set[cell]. 이미 확정(expand)된 셀. 중복 pop 무시.
================================================================================
"""

from __future__ import annotations

import heapq
import itertools
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from .occupancy import NEIGHBORS_6, Cell, OccupancyMap

if TYPE_CHECKING:
    from .cost import RouteParams


@dataclass
class AStarResult:
    """A* 탐색 결과 묶음.

    필드:
        success        : 경로를 찾았는지 여부.
        path           : 셀 리스트 [start, ..., goal]. 실패 시 None.
        length_mm      : 경로 '기하' 길이(mm) = (셀 수 - 1) * cell_mm. 실패 시 0.
        turns          : 경로의 방향 전환(직각 회전) 횟수.
        expanded_nodes : A* 가 확장(expand)한 노드(상태) 수(탐색 비용 지표).
        visited        : 탐색 중 확장된 모든 셀(collect_visited=True 일 때). 시각화용.
        elapsed_ms     : 탐색 소요 시간(ms, 참고용).
        cost_mm        : 경로 '총 비용'(mm) = 기하 길이 + turn/클리어런스/단 페널티 합.
                         가중치 없는 astar 에서는 length_mm 과 같다.
    """

    success: bool
    path: list[Cell] | None
    length_mm: float
    turns: int
    expanded_nodes: int
    visited: list[Cell] | None
    elapsed_ms: float
    cost_mm: float = 0.0

    def summary(self) -> str:
        """사람이 읽기 좋은 한 줄 요약."""
        if not self.success:
            return (
                f"[A*] 실패(경로 없음) | 확장 {self.expanded_nodes} 노드, "
                f"{self.elapsed_ms:.1f} ms"
            )
        return (
            f"[A*] 성공 | 셀 {len(self.path)}개, 길이 {self.length_mm:.0f} mm, "
            f"비용 {self.cost_mm:.0f} mm, 회전 {self.turns}회, "
            f"확장 {self.expanded_nodes} 노드, {self.elapsed_ms:.1f} ms"
        )


def manhattan(a: Cell, b: Cell) -> int:
    """두 셀의 맨해튼 거리(셀 수). |di|+|dj|+|dk|."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def count_turns(path: list[Cell]) -> int:
    """경로의 방향 전환 횟수를 센다.

    인접 셀 간 이동 방향이 직전 방향과 달라질 때마다 1회 회전으로 카운트한다.
    매개변수:
        path : 셀 리스트.
    반환값:
        회전 횟수(셀이 2개 이하면 0).
    """
    if len(path) < 3:
        return 0
    turns = 0
    prev = (path[1][0] - path[0][0], path[1][1] - path[0][1], path[1][2] - path[0][2])
    for i in range(2, len(path)):
        cur = (
            path[i][0] - path[i - 1][0],
            path[i][1] - path[i - 1][1],
            path[i][2] - path[i - 1][2],
        )
        if cur != prev:
            turns += 1
        prev = cur
    return turns


def astar(
    occ: OccupancyMap,
    start: Cell,
    goal: Cell,
    *,
    step_cost: float | None = None,
    collect_visited: bool = False,
    max_expansions: int | None = None,
) -> AStarResult:
    """점유맵 위에서 start → goal 직교 최단 경로를 A* 로 찾는다.

    매개변수:
        occ             : OccupancyMap(Dense/Sparse/BitPacked 무관). is_blocked 만 사용.
        start, goal     : 시작/목표 셀 인덱스 (i,j,k).
        step_cost       : 셀 1칸 이동 비용(mm). 기본 None → occ.cell_mm.
        collect_visited : True 면 확장한 셀을 모두 모아 결과에 담는다(시각화/디버그).
        max_expansions  : 확장 셀 수 상한(폭주 방지). None 이면 무제한.
    반환값:
        AStarResult.

    실패(success=False) 조건:
        - start 또는 goal 이 점유/격자 밖.
        - 목표까지 경로가 없음(open 소진).
        - max_expansions 초과.

    지역 변수:
        h          : 휴리스틱 함수(맨해튼 거리 × step_cost).
        open_heap  : (f, counter, cell) 최소 힙.
        g          : 셀별 최소 누적 비용.
        came_from  : 경로 복원용 직전 셀 맵.
        closed     : 확정된 셀 집합.
    """
    t0 = time.perf_counter()
    sc = float(occ.cell_mm if step_cost is None else step_cost)
    visited: list[Cell] | None = [] if collect_visited else None

    # 시작/목표가 통과 불가면 즉시 실패.
    if occ.is_blocked(start) or occ.is_blocked(goal):
        return AStarResult(False, None, 0.0, 0, 0, visited,
                           (time.perf_counter() - t0) * 1000.0)

    # 같은 셀이면 경로는 그 셀 하나.
    if start == goal:
        if collect_visited:
            visited.append(start)
        return AStarResult(True, [start], 0.0, 0, 1, visited,
                           (time.perf_counter() - t0) * 1000.0)

    def h(cell: Cell) -> float:
        return manhattan(cell, goal) * sc

    counter = itertools.count()
    open_heap: list[tuple[float, int, Cell]] = [(h(start), next(counter), start)]
    g: dict[Cell, float] = {start: 0.0}
    came_from: dict[Cell, Cell] = {}
    closed: set[Cell] = set()
    expanded = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue  # 더 나은 경로로 이미 확정된 셀(낡은 힙 항목)
        closed.add(current)
        expanded += 1
        if visited is not None:
            visited.append(current)

        if current == goal:
            path = _reconstruct(came_from, current)
            length = (len(path) - 1) * sc
            return AStarResult(
                True, path, length, count_turns(path),
                expanded, visited, (time.perf_counter() - t0) * 1000.0,
                cost_mm=length,  # 균일 비용 → 비용 = 기하 길이
            )

        if max_expansions is not None and expanded >= max_expansions:
            break

        ci, cj, ck = current
        g_current = g[current]
        for di, dj, dk in NEIGHBORS_6:
            nb = (ci + di, cj + dj, ck + dk)
            if nb in closed or occ.is_blocked(nb):
                continue
            tentative = g_current + sc
            if tentative < g.get(nb, float("inf")):
                g[nb] = tentative
                came_from[nb] = current
                heapq.heappush(open_heap, (tentative + h(nb), next(counter), nb))

    # open 소진 또는 max_expansions 초과 → 실패.
    return AStarResult(False, None, 0.0, 0, expanded, visited,
                       (time.perf_counter() - t0) * 1000.0)


def astar_weighted(
    occ: OccupancyMap,
    start: Cell,
    goal: Cell,
    params: "RouteParams",
    *,
    collect_visited: bool = False,
    max_expansions: int | None = None,
    corridor: "set | None" = None,
) -> AStarResult:
    """비용함수(turn penalty/클리어런스/단 분리)를 적용한 직교 A* (Step 1.3).

    [기본 A* 와의 핵심 차이 — 방향을 가진 상태]
      turn penalty 는 "직전 진행 방향과 다른 방향으로 꺾을 때" 부과되므로, 같은 셀이라도
      '어느 방향으로 들어왔는지'에 따라 이후 비용이 달라진다. 따라서 탐색 상태를
      (셀, 진입방향) 으로 확장한다. g/closed/came_from 은 모두 이 상태 단위로 관리한다.
        - 진입방향은 NEIGHBORS_6 의 인덱스(0~5)로, 시작 상태는 -1(방향 없음).

    매개변수:
        occ             : OccupancyMap (is_blocked 만 사용).
        start, goal     : 시작/목표 셀.
        params          : RouteParams (cell_mm/w_turn/w_clear/clearance_radius/w_tier).
        collect_visited : True 면 확장한 셀을 모은다(시각화/디버그).
        max_expansions  : 확장 상태 수 상한(폭주 방지).
    반환값:
        AStarResult. cost_mm 에 페널티 포함 총 비용, length_mm 에 기하 길이.

    비고:
        모든 비용항이 ≥0 이고 한 칸 이동 ≥ cell_mm 이므로, 맨해튼×cell_mm 휴리스틱이
        admissible & consistent → 찾은 경로는 '총 비용' 기준 최적이다.

    지역 변수:
        model      : CostModel(클리어런스 사전계산 + move_cost/heuristic).
        open_heap  : (f, counter, cell, dir_idx) 최소 힙.
        g          : {(cell, dir_idx) -> 누적 비용}.
        came_from  : {(cell, dir_idx) -> 직전 (cell, dir_idx)}.
        closed     : 확정된 (cell, dir_idx) 집합.
    """
    from .cost import CostModel  # 지연 import (순환 의존 방지)

    t0 = time.perf_counter()
    visited: list[Cell] | None = [] if collect_visited else None

    if occ.is_blocked(start) or occ.is_blocked(goal):
        return AStarResult(False, None, 0.0, 0, 0, visited,
                           (time.perf_counter() - t0) * 1000.0)
    if start == goal:
        if collect_visited:
            visited.append(start)
        return AStarResult(True, [start], 0.0, 0, 1, visited,
                           (time.perf_counter() - t0) * 1000.0, cost_mm=0.0)

    model = CostModel(occ, params, corridor=corridor)
    cell_mm = params.cell_mm

    counter = itertools.count()
    # 상태 = (cell, dir_idx). dir_idx = -1 은 '진입 방향 없음'(시작).
    open_heap: list[tuple[float, int, Cell, int]] = [
        (model.heuristic(start, goal), next(counter), start, -1)
    ]
    g: dict[tuple[Cell, int], float] = {(start, -1): 0.0}
    came_from: dict[tuple[Cell, int], tuple[Cell, int]] = {}
    closed: set[tuple[Cell, int]] = set()
    expanded = 0

    while open_heap:
        _, _, cell, dir_idx = heapq.heappop(open_heap)
        state = (cell, dir_idx)
        if state in closed:
            continue
        closed.add(state)
        expanded += 1
        if visited is not None:
            visited.append(cell)

        if cell == goal:
            path = _reconstruct_states(came_from, state)
            return AStarResult(
                True, path, (len(path) - 1) * cell_mm, count_turns(path),
                expanded, visited, (time.perf_counter() - t0) * 1000.0,
                cost_mm=g[state],
            )

        if max_expansions is not None and expanded >= max_expansions:
            break

        prev_off = None if dir_idx < 0 else NEIGHBORS_6[dir_idx]
        g_cur = g[state]
        ci, cj, ck = cell
        for nidx, (di, dj, dk) in enumerate(NEIGHBORS_6):
            nb = (ci + di, cj + dj, ck + dk)
            nstate = (nb, nidx)
            if nstate in closed or occ.is_blocked(nb):
                continue
            tentative = g_cur + model.move_cost(nb, prev_off, (di, dj, dk))
            if tentative < g.get(nstate, float("inf")):
                g[nstate] = tentative
                came_from[nstate] = state
                heapq.heappush(open_heap, (tentative + model.heuristic(nb, goal),
                                           next(counter), nb, nidx))

    return AStarResult(False, None, 0.0, 0, expanded, visited,
                       (time.perf_counter() - t0) * 1000.0)


def astar_world(
    occ: OccupancyMap,
    start_mm: tuple[float, float, float],
    goal_mm: tuple[float, float, float],
    **kwargs,
) -> AStarResult:
    """월드 좌표(mm)로 start/goal 을 받아 셀로 변환 후 astar 를 호출하는 편의 함수."""
    return astar(occ, occ.to_cell(start_mm), occ.to_cell(goal_mm), **kwargs)


def _reconstruct(came_from: dict[Cell, Cell], goal: Cell) -> list[Cell]:
    """came_from 맵을 goal 에서 start 까지 역추적해 정방향 경로 리스트로 만든다."""
    path = [goal]
    cur = goal
    while cur in came_from:
        cur = came_from[cur]
        path.append(cur)
    path.reverse()
    return path


def _reconstruct_states(
    came_from: dict[tuple[Cell, int], tuple[Cell, int]],
    goal_state: tuple[Cell, int],
) -> list[Cell]:
    """(cell, dir_idx) 상태 체인을 역추적해 셀 경로 리스트로 만든다(방향 성분 제거)."""
    cells = [goal_state[0]]
    s = goal_state
    while s in came_from:
        s = came_from[s]
        cells.append(s[0])
    cells.reverse()
    return cells


# ------------------------------------------------------------------ CLI 진입점

def _main(argv: list[str] | None = None) -> int:
    """커맨드라인 진입점. DB 영역에서 start→goal 경로를 찾고 지표·시각화를 낸다."""
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except (AttributeError, ValueError):
            pass

    parser = argparse.ArgumentParser(description="직교 A* 경로 탐색 (DB 영역)")
    parser.add_argument("--region", nargs=6, type=float, required=True,
                        metavar=("MINX", "MINY", "MINZ", "MAXX", "MAXY", "MAXZ"),
                        help="관심 영역 mm")
    parser.add_argument("--start", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"), help="시작점 mm")
    parser.add_argument("--goal", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"), help="목표점 mm")
    parser.add_argument("--cell-mm", type=float, default=50.0, help="셀 크기 mm (기본 50)")
    parser.add_argument("--types", nargs="+", default=None, help="장애물 OST_TYPE 필터")
    parser.add_argument("--inflate", type=int, default=0,
                        help="장애물 팽창 셀 수(하드 클리어런스). 기본 0")
    parser.add_argument("--w-turn", type=float, default=0.0,
                        help="회전 1회당 비용 mm(>0 이면 비용함수 A* 사용). 기본 0")
    parser.add_argument("--w-clear", type=float, default=0.0,
                        help="클리어런스 근접 페널티 계수 mm/셀. 기본 0")
    parser.add_argument("--clearance", type=int, default=2,
                        help="클리어런스 페널티 적용 반경(셀). 기본 2")
    parser.add_argument("--screenshot", default=None, help="경로+점유맵 PNG 경로")
    parser.add_argument("--html", default=None, help="경로+점유맵 인터랙티브 HTML 경로")
    parser.add_argument("--show", action="store_true", help="인터랙티브 창 표시")
    parser.add_argument("--visited", action="store_true",
                        help="A* 가 확장한 방문 셀을 반투명 레이어로 함께 렌더")
    parser.add_argument("--dbname", default=None, help="DB 이름 덮어쓰기")
    args = parser.parse_args(argv)

    from .obstacle_db import PgConnConfig, build_occupancy, load_obstacles

    region = (tuple(args.region[0:3]), tuple(args.region[3:6]))
    overrides = {"dbname": args.dbname} if args.dbname else {}
    config = PgConnConfig.from_env(**overrides)

    obstacles = load_obstacles(config, ost_types=args.types, region=region)
    occ = build_occupancy(obstacles, cell_mm=args.cell_mm, region=region).occupancy
    if args.inflate > 0:
        occ = occ.inflate(args.inflate)
    print(f"점유맵: 셀 {occ.shape}, 점유 {occ.count_blocked():,} (장애물 {len(obstacles)}건)")

    start_cell = occ.to_cell(tuple(args.start))
    goal_cell = occ.to_cell(tuple(args.goal))
    print(f"시작 {tuple(args.start)} → 셀 {start_cell} (점유={occ.is_blocked(start_cell)})")
    print(f"목표 {tuple(args.goal)} → 셀 {goal_cell} (점유={occ.is_blocked(goal_cell)})")

    # w_turn 또는 w_clear 가 양수면 비용함수 A*(astar_weighted) 사용.
    if args.w_turn > 0 or args.w_clear > 0:
        from .cost import RouteParams
        params = RouteParams(
            cell_mm=args.cell_mm, w_turn=args.w_turn,
            w_clear=args.w_clear, clearance_radius=args.clearance,
        )
        print(f"비용함수: w_turn={args.w_turn} w_clear={args.w_clear} clearance={args.clearance}")
        result = astar_weighted(occ, start_cell, goal_cell, params,
                                collect_visited=args.visited)
    else:
        result = astar(occ, start_cell, goal_cell, collect_visited=args.visited)
    print(result.summary())

    if (args.screenshot or args.html or args.show) and result.success:
        from .viz import render_occupancy
        render_occupancy(
            {"obstacles": occ},
            opacity=0.25,
            path=result.path,
            visited=result.visited if args.visited else None,
            show=args.show,
            screenshot=args.screenshot,
            html=args.html,
        )
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(_main())
