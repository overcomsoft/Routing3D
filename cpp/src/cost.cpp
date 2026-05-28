// 비용함수 구현 — cost.hpp 참고. 명세 algorithm_spec.md §4.
#include "routing3d/cost.hpp"

#include <array>
#include <deque>
#include <stdexcept>

namespace routing3d {

namespace {
// 26-연결 이웃(면+모서리+꼭짓점). 중심(0,0,0) 제외.
std::array<Cell, 26> make_neighbors26() {
    std::array<Cell, 26> out{};
    int n = 0;
    for (int di = -1; di <= 1; ++di)
        for (int dj = -1; dj <= 1; ++dj)
            for (int dk = -1; dk <= 1; ++dk)
                if (di || dj || dk) out[n++] = Cell{di, dj, dk};
    return out;
}
}  // namespace

std::vector<int> clearance_map(const DenseOccupancy& occ, int max_radius, int connectivity) {
    if (max_radius < 0) throw std::invalid_argument("max_radius must be >= 0");
    const Cell sh = occ.shape();
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
    const auto n6 = NEIGHBORS_6;
    const auto n26 = make_neighbors26();
    const Cell* off = (connectivity == 26) ? n26.data() : n6.data();
    const int noff = (connectivity == 26) ? 26 : 6;
    if (connectivity != 6 && connectivity != 26)
        throw std::invalid_argument("connectivity must be 6 or 26");

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

CostModel::CostModel(const DenseOccupancy& occ, RouteParams params)
    : occ_(occ), p_(std::move(params)) {
    if (p_.w_clear > 0.0 && p_.clearance_radius > 0) {
        clearance_ = clearance_map(occ_, p_.clearance_radius, p_.clearance_connectivity);
        has_clear_ = true;
    }
}

double CostModel::cell_penalty(const Cell& c) const {
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

double CostModel::move_cost(const Cell& to, const Cell* prev_off, const Cell& move_off) const {
    double c = p_.cell_mm;
    if (prev_off != nullptr && !(*prev_off == move_off)) c += p_.w_turn;
    c += cell_penalty(to);
    return c;
}

double CostModel::heuristic(const Cell& c, const Cell& goal) const {
    return manhattan(c, goal) * p_.cell_mm;
}

}  // namespace routing3d
