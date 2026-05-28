// 직교 A* 탐색 — Routing3D C++ 엔진 (Phase 3, Step 3.3/3.4)
// =============================================================================
// [이 파일이 하는 일]
//   점유맵 위에서 6방향 직교 최단 경로를 찾는다.
//   - astar          : 균일 비용(상태=셀).            명세 algorithm_spec.md §3
//   - astar_weighted : 비용함수(상태=(셀,진입방향)).  명세 algorithm_spec.md §4
//   휴리스틱 = manhattan × cell_mm (admissible & consistent). 단위 mm.
//   결정성(A2/W1): (f, 삽입순서 counter) tie-break + 고정 이웃 순서 → 재현 가능.
// =============================================================================
#pragma once

#include <vector>

#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/occupancy.hpp"

namespace routing3d {

struct AStarResult {
    bool success = false;
    std::vector<Cell> path;        // [start..goal], 실패 시 비어 있음
    double length_mm = 0.0;        // (셀 수 − 1) × cell_mm
    int turns = 0;                 // 방향 전환 횟수
    long long expanded_nodes = 0;  // 확장한 노드(상태) 수
    double cost_mm = 0.0;          // 페널티 포함 총 비용(균일이면 length 와 동일)
    double elapsed_ms = 0.0;
};

int count_turns(const std::vector<Cell>& path);

// 균일 비용 A*. step_cost < 0 이면 occ.cell_mm() 사용. max_expansions < 0 이면 무제한.
AStarResult astar(const DenseOccupancy& occ, Cell start, Cell goal,
                  double step_cost = -1.0, long long max_expansions = -1);

// 비용함수 A* (turn penalty / 클리어런스 / 단 분리).
AStarResult astar_weighted(const DenseOccupancy& occ, Cell start, Cell goal,
                           const RouteParams& params, long long max_expansions = -1);

}  // namespace routing3d
