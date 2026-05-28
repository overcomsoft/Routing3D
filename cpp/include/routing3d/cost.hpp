// 비용함수 (Cost Function) — Routing3D C++ 엔진 (Phase 3, Step 3.4)
// =============================================================================
// [이 파일이 하는 일]
//   RouteParams(비용 가중치) + 점유맵으로 이동 비용/휴리스틱을 계산한다.
//   클리어런스는 admissibility 보호를 위해 '가산 페널티'(감산 보너스 아님).
//   명세 algorithm_spec.md §4 와 1:1 대응. 단위 mm.
// =============================================================================
#pragma once

#include <map>
#include <vector>

#include "routing3d/geometry.hpp"
#include "routing3d/occupancy.hpp"

namespace routing3d {

// 라우팅 비용 파라미터 (모든 비용 mm). 모든 값 >= 0 (보너스/감산 금지).
struct RouteParams {
    double cell_mm = 50.0;
    double w_turn = 500.0;
    double w_clear = 10.0;
    int clearance_radius = 2;
    int clearance_connectivity = 6;  // 6 또는 26
    std::map<int, double> w_tier;     // z셀 → 가산 mm
};

// 각 셀에서 가장 가까운 장애물까지 거리(셀, 상한 max_radius). 장애물=0, 멀수록 큼.
// bounded distance transform 을 다중소스 BFS 로 계산(연결성 6/26).
std::vector<int> clearance_map(const DenseOccupancy& occ, int max_radius, int connectivity = 6);

class CostModel {
public:
    CostModel(const DenseOccupancy& occ, RouteParams params);

    double cell_penalty(const Cell& c) const;  // 클리어런스 근접 + 단 분리
    // from→to 이동 1회 비용. prev_off == nullptr 이면 시작(회전 없음).
    double move_cost(const Cell& to, const Cell* prev_off, const Cell& move_off) const;
    double heuristic(const Cell& c, const Cell& goal) const;  // manhattan × cell_mm

private:
    const DenseOccupancy& occ_;
    RouteParams p_;
    std::vector<int> clearance_;  // 비어 있으면 클리어런스 비활성
    bool has_clear_ = false;
};

}  // namespace routing3d
