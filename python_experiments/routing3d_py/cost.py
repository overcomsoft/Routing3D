"""비용함수 (Cost Function) — Phase 1 Step 1.3
================================================================================

[실행 명령어]  (editable 설치 후 프로젝트 루트에서)
  # 비용함수 적용 A* (turn penalty / 클리어런스 / 단 분리)
  .\\.venv\\Scripts\\python.exe -m routing3d_py.astar ^
      --region 195000 8000 14000 205000 12000 16000 --cell-mm 50 ^
      --start 195300 8300 14775 --goal 204700 11700 14775 ^
      --w-turn 500 --w-clear 10 --clearance 2 --screenshot python_experiments/out/route_cost.png

  # 단위 테스트
  .\\.venv\\Scripts\\python.exe -m pytest python_experiments/tests/test_cost.py -v

================================================================================
[이 모듈이 하는 일]
--------------------------------------------------------------------------------
직교 A* 의 '이동 비용'을 정의한다. 기본 이동비용(셀=cell_mm) 위에 다음을 더한다.
  1) Turn penalty  : 진행 방향이 바뀔 때 가산 → 직각 회전 최소화(엘보 줄이기).
  2) 클리어런스 페널티: 장애물에 가까운 셀일수록 가산 → 벽에서 떨어져 지나가게 유도.
  3) 단(段) 분리   : z 레벨별 가산 → 특정 단(배관 랙)으로 유도/회피.

[중요 설계 결정 — 왜 '보너스'가 아니라 '페널티'인가]
--------------------------------------------------------------------------------
계획서 초안은 클리어런스를 "여유 셀 수에 비례한 비용 감산(보너스)"으로 표현했다.
그러나 비용을 감산하면 한 칸 이동 비용이 cell_mm 보다 작아질 수 있어, 맨해튼 거리
×cell_mm 휴리스틱이 실제 비용을 '과대평가'하게 되고 → A* 의 admissibility(최적성)가
깨진다. 그래서 동일한 목적(벽에서 멀어지기)을 **장애물 근접 시 가산하는 페널티**로
구현한다. 모든 가산항은 ≥ 0 이므로 이동 비용 ≥ cell_mm 이 보장되고, 맨해튼 휴리스틱이
admissible & consistent 하게 유지된다(A* 최적성 보존).

[전체 흐름]
--------------------------------------------------------------------------------
  RouteParams(가중치)  +  OccupancyMap
        │
        ▼  CostModel(occ, params)         # 생성 시 클리어런스 맵 1회 사전계산
        │
        ▼  move_cost(to_cell, prev_off, move_off)
        =  cell_mm
           + (w_turn  if 방향 바뀜)
           + 클리어런스 페널티(to_cell)     # 장애물에 가까울수록 큼
           + 단 분리 페널티(to_cell.z)

  clearance_map(occ, r): 각 셀의 '가장 가까운 장애물까지 거리(셀)'를 r 로 상한.
        장애물=0, 멀리 떨어진 셀=r. 반복적 이진 팽창(bounded distance transform)으로 계산.
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .occupancy import NEIGHBORS_6, NEIGHBORS_26, Cell, OccupancyMap, _dilate_once


@dataclass
class RouteParams:
    """라우팅 비용 파라미터(모든 비용 단위 mm).

    필드:
        cell_mm                : 셀 1칸 이동 기본 비용(mm). 기본 50.
        w_turn                 : 방향 전환(직각 회전) 1회당 가산 비용(mm). 기본 500
                                 (= 셀 10칸). 클수록 회전을 강하게 회피.
        w_clear                : 클리어런스 페널티 계수(mm/셀). 장애물에 1셀 더 가까울수록
                                 이만큼 가산. 0 이면 클리어런스 비활성.
        clearance_radius       : 페널티를 적용할 최대 근접 거리(셀). 이보다 멀면 페널티 0.
        clearance_connectivity : 클리어런스 거리 측정 이웃(6 또는 26). 기본 6.
        w_tier                 : 단 분리 가중치 {z셀인덱스: 가산 mm}. 기본 없음(전부 0).

    검증:
        값이 음수면 ValueError(보너스/감산은 admissibility 보호 위해 금지).
    """

    cell_mm: float = 50.0
    w_turn: float = 500.0
    w_clear: float = 10.0
    clearance_radius: int = 2
    clearance_connectivity: int = 6
    w_tier: dict[int, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.cell_mm <= 0:
            raise ValueError(f"cell_mm must be positive, got {self.cell_mm}")
        if self.w_turn < 0 or self.w_clear < 0:
            raise ValueError("w_turn/w_clear must be >= 0 (penalties, not bonuses)")
        if self.clearance_radius < 0:
            raise ValueError("clearance_radius must be >= 0")
        if self.clearance_connectivity not in (6, 26):
            raise ValueError("clearance_connectivity must be 6 or 26")
        if any(v < 0 for v in self.w_tier.values()):
            raise ValueError("w_tier values must be >= 0")


def clearance_map(occ: OccupancyMap, max_radius: int, *, connectivity: int = 6) -> np.ndarray:
    """각 셀에서 '가장 가까운 장애물까지의 거리(셀 단위)'를 max_radius 로 상한해 계산한다.

    [알고리즘] bounded distance transform (반복적 이진 팽창)
      - 장애물 셀 = 거리 0.
      - 장애물을 1단계 팽창할 때마다 새로 덮이는 셀의 거리 = 그 단계 번호.
      - max_radius 단계까지만 수행. 끝까지 안 덮인(멀리 떨어진) 셀 = max_radius.

    매개변수:
        occ          : OccupancyMap (to_numpy 로 dense bool 사용).
        max_radius   : 거리 상한(셀). 이보다 먼 셀은 모두 max_radius 로 본다.
        connectivity : 거리 측정 이웃(6=맨해튼, 26=체비셰프). 기본 6.
    반환값:
        np.int16 배열(nx,ny,nz). 장애물=0, 멀수록 큼(최대 max_radius).

    지역 변수:
        grid    : 장애물 bool 배열(True=장애).
        dist    : 결과 거리 배열(초기 max_radius, 장애물 0).
        current : 단계별로 누적 팽창되는 영역.
        newly   : 이번 단계에서 처음 덮인 셀 → 거리 = 현재 단계 d.
    """
    if max_radius < 0:
        raise ValueError("max_radius must be >= 0")
    grid = occ.to_numpy()
    dist = np.full(grid.shape, max_radius, dtype=np.int16)
    dist[grid] = 0
    if max_radius == 0:
        return dist
    offsets = NEIGHBORS_6 if connectivity == 6 else NEIGHBORS_26
    current = grid.copy()
    for d in range(1, max_radius + 1):
        dilated = _dilate_once(current, offsets)
        newly = dilated & ~current  # 이번 단계에서 처음 장애물 영향권에 든 셀
        dist[newly] = d
        current = dilated
    return dist


class CostModel:
    """RouteParams + OccupancyMap 으로 이동 비용을 계산하는 모델.

    생성 시 클리어런스 맵을 1회 사전계산해두고(질의마다 재계산 방지), 이동 비용과
    휴리스틱을 제공한다.

    속성:
        occ      : 대상 점유맵.
        p        : RouteParams.
        clearance: 클리어런스 거리 배열(없으면 None).
    """

    def __init__(self, occ: OccupancyMap, params: RouteParams) -> None:
        self.occ = occ
        self.p = params
        self.clearance: np.ndarray | None = None
        if params.w_clear > 0 and params.clearance_radius > 0:
            self.clearance = clearance_map(
                occ, params.clearance_radius, connectivity=params.clearance_connectivity
            )

    def cell_penalty(self, cell: Cell) -> float:
        """목적지 셀에 대한 가산 페널티(클리어런스 근접 + 단 분리)."""
        pen = 0.0
        if self.clearance is not None:
            d = int(self.clearance[cell])  # 가장 가까운 장애물까지 거리(셀)
            if d < self.p.clearance_radius:
                # 가까울수록(d 작을수록) 큰 페널티. 장애물 인접(d=0)에서 최대.
                pen += self.p.w_clear * (self.p.clearance_radius - d)
        if self.p.w_tier:
            pen += self.p.w_tier.get(cell[2], 0.0)
        return pen

    def move_cost(self, to_cell: Cell, prev_off: Cell | None, move_off: Cell) -> float:
        """from→to 이동 1회의 총 비용.

        매개변수:
            to_cell  : 이동할 목적지 셀.
            prev_off : 직전 이동 방향 오프셋(시작 셀이면 None → 회전 페널티 없음).
            move_off : 이번 이동 방향 오프셋.
        반환값:
            cell_mm + (방향 바뀌면 w_turn) + cell_penalty(to_cell).
        """
        c = self.p.cell_mm
        if prev_off is not None and move_off != prev_off:
            c += self.p.w_turn
        c += self.cell_penalty(to_cell)
        return c

    def heuristic(self, cell: Cell, goal: Cell) -> float:
        """맨해튼 거리 × cell_mm. 모든 가산항이 ≥0 이므로 admissible & consistent."""
        return (
            abs(cell[0] - goal[0]) + abs(cell[1] - goal[1]) + abs(cell[2] - goal[2])
        ) * self.p.cell_mm
