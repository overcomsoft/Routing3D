// 비용함수 (Cost Function) — Routing3D C++ 엔진 (Phase 3, Step 3.4/3.6)
// =============================================================================
// [이 파일이 하는 일]
//   RouteParams(비용 가중치) + 점유맵으로 이동 비용/휴리스틱을 계산한다.
//   클리어런스는 admissibility 보호를 위해 '가산 페널티'(감산 보너스 아님).
//   명세 algorithm_spec.md §4 와 1:1 대응. 단위 mm.
//
//   Step 3.6: 점유맵 백엔드(Dense/Sparse/Vdb)에 **무관**하도록 템플릿화(헤더 정의).
//   백엔드는 동일 질의 인터페이스(in_bounds/is_blocked/lin/unlin/shape/size)만 제공하면 된다.
// =============================================================================
#pragma once

#include <array>
#include <deque>
#include <map>
#include <stdexcept>
#include <vector>

#include "routing3d/geometry.hpp"

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

// 26-연결 이웃(면+모서리+꼭짓점, 중심 제외). 고정 순서.
inline const std::array<Cell, 26>& neighbors26() {
    static const std::array<Cell, 26> n = []() {
        std::array<Cell, 26> out{};
        int m = 0;
        for (int di = -1; di <= 1; ++di)
            for (int dj = -1; dj <= 1; ++dj)
                for (int dk = -1; dk <= 1; ++dk)
                    if (di || dj || dk) out[m++] = Cell{di, dj, dk};
        return out;
    }();
    return n;
}

// 각 셀에서 가장 가까운 장애물까지 거리(셀, 상한 max_radius). 장애물=0, 멀수록 큼.
// bounded distance transform 을 다중소스 BFS 로 계산(연결성 6/26). 백엔드 무관(템플릿).
template <class Occ>
std::vector<int> clearance_map(const Occ& occ, int max_radius, int connectivity = 6) {
    if (max_radius < 0) throw std::invalid_argument("max_radius must be >= 0");
    if (connectivity != 6 && connectivity != 26)
        throw std::invalid_argument("connectivity must be 6 or 26");
    const long long N = occ.size();
    std::vector<int> dist(static_cast<size_t>(N), max_radius);
    if (max_radius == 0) {
        for (long long idx = 0; idx < N; ++idx)
            if (occ.is_blocked(occ.unlin(static_cast<int>(idx)))) dist[static_cast<size_t>(idx)] = 0;
        return dist;
    }

    // 다중소스 BFS: 장애물(dist=0)에서 출발해 연결성 이웃으로 +1 씩 확장(max_radius 상한).
    std::deque<Cell> q;
    for (long long idx = 0; idx < N; ++idx) {
        Cell c = occ.unlin(static_cast<int>(idx));
        if (occ.is_blocked(c)) {
            dist[static_cast<size_t>(idx)] = 0;
            q.push_back(c);
        }
    }
    const Cell* off = (connectivity == 26) ? neighbors26().data() : NEIGHBORS_6.data();
    const int noff = (connectivity == 26) ? 26 : 6;

    while (!q.empty()) {
        Cell c = q.front();
        q.pop_front();
        int d = dist[static_cast<size_t>(occ.lin(c))];
        if (d >= max_radius) continue;
        for (int t = 0; t < noff; ++t) {
            Cell nb{c.i + off[t].i, c.j + off[t].j, c.k + off[t].k};
            if (!occ.in_bounds(nb)) continue;
            size_t nl = static_cast<size_t>(occ.lin(nb));
            if (dist[nl] > d + 1) {
                dist[nl] = d + 1;
                q.push_back(nb);
            }
        }
    }
    return dist;
}

// 이동 비용/휴리스틱 계산기. 백엔드 무관(템플릿). 클리어런스 비활성이면 distance transform 미계산.
template <class Occ>
class CostModel {
public:
    CostModel(const Occ& occ, RouteParams params) : occ_(occ), p_(std::move(params)) {
        if (p_.w_clear > 0.0 && p_.clearance_radius > 0) {
            clearance_ = clearance_map(occ_, p_.clearance_radius, p_.clearance_connectivity);
            has_clear_ = true;
        }
    }

    // 클리어런스 근접 + 단(z) 분리 가산 페널티.
    double cell_penalty(const Cell& c) const {
        double pen = 0.0;
        if (has_clear_) {
            int d = clearance_[static_cast<size_t>(occ_.lin(c))];
            if (d < p_.clearance_radius) pen += p_.w_clear * (p_.clearance_radius - d);
        }
        if (!p_.w_tier.empty()) {
            auto it = p_.w_tier.find(c.k);
            if (it != p_.w_tier.end()) pen += it->second;
        }
        return pen;
    }

    // from→to 이동 1회 비용. prev_off == nullptr 이면 시작(회전 없음).
    double move_cost(const Cell& to, const Cell* prev_off, const Cell& move_off) const {
        double c = p_.cell_mm;
        if (prev_off != nullptr && !(*prev_off == move_off)) c += p_.w_turn;
        c += cell_penalty(to);
        return c;
    }

    double heuristic(const Cell& c, const Cell& goal) const {  // manhattan × cell_mm
        return manhattan(c, goal) * p_.cell_mm;
    }

private:
    const Occ& occ_;
    RouteParams p_;
    std::vector<int> clearance_;  // 비어 있으면 클리어런스 비활성
    bool has_clear_ = false;
};

}  // namespace routing3d
