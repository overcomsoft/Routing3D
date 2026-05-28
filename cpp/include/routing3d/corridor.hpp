// 계층 corridor 라우팅 (Hierarchical Corridor) — Routing3D C++ 엔진 (Phase 3, Step 3.6 part 4)
// =============================================================================
// [이 파일이 하는 일]
//   초대형(8,000m) 장면에서도 빠른 단일 배관 탐색을 위해 두 가지를 제공한다.
//   1) astar_hashed : g/closed 를 **해시 기반**(셀 패킹키)으로 잡는 균일비용 A*.
//      현 astar 는 closed 를 occ.size() 크기 배열로 잡아 초대형 격자에서 할당 불가지만,
//      해시 기반은 '실제 탐색한 셀 수'에만 비례 → 거대 장면의 **로컬 배관**을 즉시 라우팅.
//      탐색 영역을 in_corridor(셀) 술어로 제한할 수 있다(전체 허용 시 전역 탐색).
//   2) route_corridor : coarse 점유맵에서 대략 경로를 찾고(가이드), 그 주변(반경 radius)만
//      fine A* 로 정밀 탐색(tube 한정) → 장거리 경로의 탐색량을 크게 줄인다.
//
//   결정성: astar 와 동일한 (f, counter) tie-break + 고정 이웃 순서 → in_corridor 가 전체
//   허용이면 astar 와 **동일 경로/확장수**를 낸다(셀 좌표는 packing 가능 범위, 축<2^20).
//
//   주의(현 한계): coarse 점유맵은 호출자가 구성한다(보통 동일 장애물을 coarse 해상도로
//   add_box). 보수적(겹치면 점유) 구성은 얇은 통로를 과차단할 수 있어 radius 로 완화한다.
//   클리어런스(distance transform)는 전역 배열이 필요해 초대형에서 비활성(향후 로컬화).
// =============================================================================
#pragma once

#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing3d/astar.hpp"  // AStarResult, count_turns, detail::PQItem/PQCmp/Clock
#include "routing3d/geometry.hpp"

namespace routing3d {

// 셀 → 64비트 패킹키(축당 20비트, 0..1,048,575). 8,000m/50mm=160,000 < 2^20 이므로 충분.
inline uint64_t pack20(const Cell& c) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(c.i)) << 40) |
           (static_cast<uint64_t>(static_cast<uint32_t>(c.j)) << 20) |
           static_cast<uint64_t>(static_cast<uint32_t>(c.k));
}
inline Cell unpack20(uint64_t k) {
    return Cell{static_cast<int>((k >> 40) & 0xFFFFF), static_cast<int>((k >> 20) & 0xFFFFF),
                static_cast<int>(k & 0xFFFFF)};
}

// 모든 셀을 허용하는 corridor 술어(전역 탐색).
struct CorridorAll {
    bool operator()(const Cell&) const { return true; }
};

// 해시 기반 균일비용 A*. step_cost<0 이면 occ.cell_mm(). in_corridor(셀)==false 면 탐색 제외.
// occ.size() 크기 배열을 쓰지 않으므로 초대형 격자에서도 동작(메모리 ∝ 탐색한 셀 수).
template <class Occ, class InCorridor>
AStarResult astar_hashed(const Occ& occ, Cell start, Cell goal, double step_cost,
                         InCorridor in_corridor, long long max_expansions = -1) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double sc = (step_cost < 0.0) ? occ.cell_mm() : step_cost;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (!in_corridor(start) || !in_corridor(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    auto h = [&](const Cell& c) { return manhattan(c, goal) * sc; };

    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, uint64_t> came;  // nb 키 → 직전 셀 키
    std::unordered_set<uint64_t> closed;

    long long counter = 0;
    g[pack20(start)] = 0.0;
    open.push({h(start), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        detail::PQItem cur = open.top();
        open.pop();
        uint64_t ck = pack20(cur.cell);
        if (closed.count(ck)) continue;
        closed.insert(ck);
        ++expanded;

        if (cur.cell == goal) {
            std::vector<Cell> path{goal};
            uint64_t k = ck;
            auto it = came.find(k);
            while (it != came.end()) {
                k = it->second;
                path.push_back(unpack20(k));
                it = came.find(k);
            }
            for (size_t a = 0, b = path.size() - 1; a < b; ++a, --b) std::swap(path[a], path[b]);
            R.success = true;
            R.path = std::move(path);
            R.length_mm = (R.path.size() - 1) * sc;
            R.cost_mm = R.length_mm;
            R.turns = count_turns(R.path);
            R.expanded_nodes = expanded;
            R.elapsed_ms = detail::ms_since(t0);
            return R;
        }
        if (max_expansions > 0 && expanded >= max_expansions) break;

        double g_cur = g[ck];
        for (const Cell& d : NEIGHBORS_6) {
            Cell nb{cur.cell.i + d.i, cur.cell.j + d.j, cur.cell.k + d.k};
            if (!occ.in_bounds(nb) || occ.is_blocked(nb)) continue;
            if (!in_corridor(nb)) continue;
            uint64_t nk = pack20(nb);
            if (closed.count(nk)) continue;
            double t = g_cur + sc;
            auto git = g.find(nk);
            if (git == g.end() || t < git->second) {
                g[nk] = t;
                came[nk] = ck;
                open.push({t + h(nb), counter++, nb, -1});
            }
        }
    }
    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    return R;
}

// 계층 corridor 라우팅 결과.
struct CorridorRoute {
    AStarResult fine;                 // 최종 fine 경로(성공/길이/회전/확장수).
    std::vector<Cell> coarse_path;    // coarse 가이드 경로(coarse 셀).
    bool coarse_success = false;      // coarse 단계 성공 여부.
    long long corridor_cells = 0;     // corridor(팽창된 coarse 셀) 개수.
};

// 계층 corridor 라우팅: coarse 가이드 → 반경 radius 팽창 tube → fine A*(tube 한정).
//   fine, coarse : 동일 origin, coarse.cell_mm == factor × fine.cell_mm 이어야 한다.
//   factor       : coarse/fine 셀 비율(예: 16).
//   radius       : corridor 팽창 반경(coarse 셀, Chebyshev).
template <class FineOcc, class CoarseOcc>
CorridorRoute route_corridor(const FineOcc& fine, const CoarseOcc& coarse, Cell start_fine,
                             Cell goal_fine, int factor, int radius,
                             long long max_expansions = -1) {
    CorridorRoute R;
    auto to_coarse = [&](const Cell& c) {
        return Cell{c.i / factor, c.j / factor, c.k / factor};  // i>=0 → 바닥 나눗셈.
    };

    // 1) coarse 가이드 경로.
    AStarResult cres =
        astar_hashed(coarse, to_coarse(start_fine), to_coarse(goal_fine), coarse.cell_mm(),
                     CorridorAll{}, max_expansions);
    R.coarse_success = cres.success;
    R.coarse_path = cres.path;
    if (!cres.success) return R;

    // 2) coarse 경로를 반경 radius 로 팽창 → corridor(coarse 셀 키 집합).
    std::unordered_set<uint64_t> corr;
    for (const Cell& c : cres.path)
        for (int di = -radius; di <= radius; ++di)
            for (int dj = -radius; dj <= radius; ++dj)
                for (int dk = -radius; dk <= radius; ++dk)
                    corr.insert(pack20(Cell{c.i + di, c.j + dj, c.k + dk}));
    R.corridor_cells = static_cast<long long>(corr.size());

    // 3) fine A* — fine 셀의 coarse 셀이 corridor 에 있을 때만 탐색.
    auto in_corr = [&](const Cell& fc) { return corr.count(pack20(to_coarse(fc))) > 0; };
    R.fine = astar_hashed(fine, start_fine, goal_fine, fine.cell_mm(), in_corr, max_expansions);
    return R;
}

}  // namespace routing3d
