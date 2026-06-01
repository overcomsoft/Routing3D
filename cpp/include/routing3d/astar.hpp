// 직교 A* 탐색 — Routing3D C++ 엔진 (Phase 3, Step 3.3/3.4/3.6)
// =============================================================================
// [이 파일이 하는 일]
//   점유맵 위에서 6방향 직교 최단 경로를 찾는다.
//   - astar          : 균일 비용(상태=셀).            명세 algorithm_spec.md §3
//   - astar_weighted : 비용함수(상태=(셀,진입방향)).  명세 algorithm_spec.md §4
//   휴리스틱 = manhattan × cell_mm (admissible & consistent). 단위 mm.
//   결정성(A2/W1): (f, 삽입순서 counter) tie-break + 고정 이웃 순서 → 재현 가능.
//
//   Step 3.6: 점유맵 백엔드(Dense/Sparse/Vdb)에 **무관**하도록 템플릿화(헤더 정의).
//   백엔드는 in_bounds/is_blocked/lin/unlin/size/cell_mm 만 제공하면 된다.
//   주의: g/closed 를 lin() 선형 인덱스 배열로 잡으므로 '경계가 한정된'(작은/corridor) 격자용.
//         초대형 격자 전역 탐색은 후속(계층 corridor)에서 해시 기반으로 확장한다.
// =============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"

namespace routing3d {

struct AStarResult {
    bool success = false;
    std::vector<Cell> path;        // [start..goal], 실패 시 비어 있음
    double length_mm = 0.0;        // (셀 수 − 1) × cell_mm
    int turns = 0;                 // 방향 전환 횟수
    long long expanded_nodes = 0;  // 확장한 노드(상태) 수
    double cost_mm = 0.0;          // 페널티 포함 총 비용(균일이면 length 와 동일)
    double elapsed_ms = 0.0;
    // 방문(확장) 셀 목록 — collect_visited=true 일 때만 채워진다(셀 단위 중복 제거).
    // 가시화(방문맵 레이어, scene.txt [visited] 섹션) 용도. 길이 = 고유 expanded 셀 수.
    std::vector<Cell> visited;
};

// 경로의 방향 전환 횟수(백엔드 무관). 헤더 인라인.
inline int count_turns(const std::vector<Cell>& path) {
    if (path.size() < 3) return 0;
    int turns = 0;
    Cell prev{path[1].i - path[0].i, path[1].j - path[0].j, path[1].k - path[0].k};
    for (size_t n = 2; n < path.size(); ++n) {
        Cell cur{path[n].i - path[n - 1].i, path[n].j - path[n - 1].j, path[n].k - path[n - 1].k};
        if (!(cur == prev)) ++turns;
        prev = cur;
    }
    return turns;
}

namespace detail {

using Clock = std::chrono::steady_clock;

inline double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// 우선순위 큐 항목: (f, counter, cell, dir). 최소 힙(f 작은 것 우선, 동률은 counter 작은 것).
struct PQItem {
    double f;
    long long counter;
    Cell cell;
    int dir;
};
struct PQCmp {
    bool operator()(const PQItem& a, const PQItem& b) const {
        if (a.f != b.f) return a.f > b.f;  // 작은 f 가 top
        return a.counter > b.counter;       // 동률이면 먼저 삽입된 것이 top
    }
};

}  // namespace detail

// ---------------------------------------------------------------- 균일 비용 A*
// step_cost < 0 이면 occ.cell_mm() 사용. max_expansions < 0 이면 무제한.
template <class Occ>
AStarResult astar(const Occ& occ, Cell start, Cell goal, double step_cost = -1.0,
                  long long max_expansions = -1, bool collect_visited = false) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double sc = (step_cost < 0.0) ? occ.cell_mm() : step_cost;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    auto h = [&](const Cell& c) { return manhattan(c, goal) * sc; };

    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;
    std::unordered_map<int, double> g;
    std::unordered_map<int, Cell> came;  // nb 의 lin → 직전 cell
    std::vector<uint8_t> closed(static_cast<size_t>(occ.size()), 0);

    long long counter = 0;
    g[occ.lin(start)] = 0.0;
    open.push({h(start), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        detail::PQItem cur = open.top();
        open.pop();
        int cl = occ.lin(cur.cell);
        if (closed[static_cast<size_t>(cl)]) continue;
        closed[static_cast<size_t>(cl)] = 1;
        ++expanded;
        if (collect_visited) R.visited.push_back(cur.cell);

        if (cur.cell == goal) {
            std::vector<Cell> path{goal};
            Cell c = goal;
            auto it = came.find(occ.lin(c));
            while (it != came.end()) {
                c = it->second;
                path.push_back(c);
                it = came.find(occ.lin(c));
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

        double g_cur = g[cl];
        for (const Cell& d : NEIGHBORS_6) {
            Cell nb{cur.cell.i + d.i, cur.cell.j + d.j, cur.cell.k + d.k};
            if (!occ.in_bounds(nb) || occ.is_blocked(nb)) continue;
            int nl = occ.lin(nb);
            if (closed[static_cast<size_t>(nl)]) continue;
            double t = g_cur + sc;
            auto git = g.find(nl);
            if (git == g.end() || t < git->second) {
                g[nl] = t;
                came[nl] = cur.cell;
                open.push({t + h(nb), counter++, nb, -1});
            }
        }
    }
    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    return R;
}

// ------------------------------------------------------------- 비용함수 A*
// 상태 = (셀, 진입방향 dir). dir ∈ [-1,5] → state = lin*7 + (dir+1).
template <class Occ>
AStarResult astar_weighted(const Occ& occ, Cell start, Cell goal, const RouteParams& params,
                           long long max_expansions = -1, bool collect_visited = false,
                           const std::unordered_set<int>* corridor = nullptr) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double cell_mm = params.cell_mm;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    CostModel<Occ> model(occ, params, corridor);
    auto state_of = [&](int lin, int dir) -> long long {
        return static_cast<long long>(lin) * 7 + (dir + 1);
    };

    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;
    std::unordered_map<long long, double> g;
    std::unordered_map<long long, long long> came;  // state → 직전 state
    // 확정(closed) state 집합 — 해시 기반(메모리 ∝ 탐색 셀). dense 배열(occ.size()*7)로 잡으면
    // 대형/정밀 격자(예 360x577x626 ≈ 1.3억 셀 → 910MB)에서 메모리 폭발/크래시. 결정성은 PQ
    // (f, counter) 가 보장하므로 closed 자료구조 교체는 결과/expanded_nodes 에 영향 없음.
    std::unordered_set<long long> closed;
    // 셀 단위 방문 dedupe(collect_visited 일 때만). 같은 셀이 여러 진입방향으로 expand 돼도 시각화는 한 번.
    std::unordered_set<int> visited_seen;

    long long counter = 0;
    long long s0 = state_of(occ.lin(start), -1);
    g[s0] = 0.0;
    open.push({model.heuristic(start, goal), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        detail::PQItem cur = open.top();
        open.pop();
        long long st = state_of(occ.lin(cur.cell), cur.dir);
        if (!closed.insert(st).second) continue;   // 이미 확정된 state면 skip.
        ++expanded;
        if (collect_visited) {
            int cl = occ.lin(cur.cell);
            if (visited_seen.insert(cl).second) R.visited.push_back(cur.cell);
        }

        if (cur.cell == goal) {
            std::vector<Cell> path;
            long long s = st;
            path.push_back(occ.unlin(static_cast<int>(s / 7)));
            auto it = came.find(s);
            while (it != came.end()) {
                s = it->second;
                path.push_back(occ.unlin(static_cast<int>(s / 7)));
                it = came.find(s);
            }
            for (size_t a = 0, b = path.size() - 1; a < b; ++a, --b) std::swap(path[a], path[b]);
            R.success = true;
            R.path = std::move(path);
            R.length_mm = (R.path.size() - 1) * cell_mm;
            R.cost_mm = g[st];
            R.turns = count_turns(R.path);
            R.expanded_nodes = expanded;
            R.elapsed_ms = detail::ms_since(t0);
            return R;
        }
        if (max_expansions > 0 && expanded >= max_expansions) break;

        const Cell* prev_off = (cur.dir < 0) ? nullptr : &NEIGHBORS_6[static_cast<size_t>(cur.dir)];
        double g_cur = g[st];
        for (int nidx = 0; nidx < 6; ++nidx) {
            const Cell& d = NEIGHBORS_6[static_cast<size_t>(nidx)];
            Cell nb{cur.cell.i + d.i, cur.cell.j + d.j, cur.cell.k + d.k};
            if (!occ.in_bounds(nb) || occ.is_blocked(nb)) continue;
            long long ns = state_of(occ.lin(nb), nidx);
            if (closed.count(ns)) continue;
            double t = g_cur + model.move_cost(nb, prev_off, d);
            auto git = g.find(ns);
            if (git == g.end() || t < git->second) {
                g[ns] = t;
                came[ns] = st;
                open.push({t + model.heuristic(nb, goal), counter++, nb, nidx});
            }
        }
    }
    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    return R;
}

}  // namespace routing3d
