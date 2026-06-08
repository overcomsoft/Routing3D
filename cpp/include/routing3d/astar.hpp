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
#include <functional>
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

// 모든 셀을 허용하는 corridor 술어 — astar_weighted 의 기본값(전역 탐색, 제한 없음).
// 기존 호출은 이 기본을 쓰므로 `if(!in_corridor(nb))` 가 항상 false → 컴파일러가 제거(코드/결과 불변).
struct AllowAll {
    bool operator()(const Cell&) const noexcept { return true; }
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
// on_progress(expanded, progress01) → bool: 탐색 중 진행율 콜백(뷰어 진행 다이얼로그의 '처리상태 %').
//   progress01 = 1 - h_min/h_start (목표까지 남은 최소 휴리스틱의 감소율, [0,0.99] 클램프).
//   progress_every>0 이고 on_progress 가 유효하면 그 간격(확장 수)마다 호출. 결과/결정성에는 영향 없음.
//   반환 true=취소(abort) → 탐색 루프를 즉시 종료(실패 결과 반환). 호출자가 협력적 취소에 사용.
template <class Occ, class InCorridor = AllowAll>
AStarResult astar_weighted(const Occ& occ, Cell start, Cell goal, const RouteParams& params,
                           long long max_expansions = -1, bool collect_visited = false,
                           const std::unordered_set<long long>* corridor = nullptr,
                           const std::function<bool(long long, double)>* on_progress = nullptr,
                           long long progress_every = 0, InCorridor in_corridor = {}) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double cell_mm = params.cell_mm;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    // corridor 술어(하드 제한) — start/goal 이 튜브 밖이면 즉시 실패(호출자가 무제한으로 폴백).
    // 기본 AllowAll 이면 항상 true → 분기 제거(기존 동작·골든 결과 완전 불변).
    if (!in_corridor(start) || !in_corridor(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    CostModel<Occ> model(occ, params, corridor);
    // 상태키 = lin*7 + (dir+1). lin 은 백엔드 키 타입(Dense/Sparse=int, Implicit=long long)을
    // long long 으로 받아 곱한다 → 10mm 거대격자(20억 셀)에서도 int 오버플로 없음(S2).
    auto state_of = [&](long long lin, int dir) -> long long { return lin * 7 + (dir + 1); };

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
    const double h_start = model.heuristic(start, goal);  // 진행율 기준(시작→목표 휴리스틱).
    double h_min = h_start;                                // 지금까지 본 목표 최근접 휴리스틱.
    // 동적(수렴) 가중 — w_heur_near 가 (0, w_heur) 면 목표까지 거리비로 가중을 보간. 비활성이면
    // weighted_h(c)=heuristic_raw(c)*wf 로 정적 model.heuristic 과 부동소수까지 동일(골든 불변).
    const double wf = (params.w_heur > 0.0) ? params.w_heur : 1.0;
    const double wn = params.w_heur_near;
    // 동적 가중은 '무제한' 탐색에서만 적용한다(max_expansions<=0). 예산-게이트(probe 300k·hier 12M)
    // 탐색에서 목표근처 가중을 낮추면 예산을 초과해 escalation/폴백이 폭주(실측 cell=25 1.3s→33s)하므로,
    // 거대격자 경로(항상 상한 부여)에선 자동 비활성 → 정적 w_heur 유지. 골든(wn=0)은 항상 비활성.
    const bool dyn = (wn > 0.0 && wn < wf && max_expansions <= 0);
    const double h_start_raw = model.heuristic_raw(start, goal);
    auto weighted_h = [&](const Cell& c) -> double {
        const double hr = model.heuristic_raw(c, goal);
        if (!dyn) return hr * wf;
        const double w = wn + (wf - wn) * (h_start_raw > 0.0 ? hr / h_start_raw : 0.0);
        return hr * w;
    };
    open.push({weighted_h(start), counter++, start, -1});
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
        // 진행율 보고(처리상태 %): 목표까지 남은 최소 휴리스틱의 감소율.
        if (on_progress && progress_every > 0) {
            double hc = model.heuristic(cur.cell, goal);
            if (hc < h_min) h_min = hc;
            if (expanded % progress_every == 0) {
                double prog = (h_start > 0.0) ? 1.0 - h_min / h_start : 0.0;
                if (prog < 0.0) prog = 0.0;
                if (prog > 0.99) prog = 0.99;
                // 콜백이 취소(true)를 반환하면 탐색을 즉시 중단(실패 결과 반환) — 협력적 취소.
                if ((*on_progress)(expanded, prog)) { R.expanded_nodes = expanded; R.elapsed_ms = detail::ms_since(t0); return R; }
            }
        }

        if (cur.cell == goal) {
            std::vector<Cell> path;
            long long s = st;
            path.push_back(occ.unlin(s / 7));  // unlin 은 long long 인자(백엔드 무관, S2).
            auto it = came.find(s);
            while (it != came.end()) {
                s = it->second;
                path.push_back(occ.unlin(s / 7));
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
            if (!in_corridor(nb)) continue;   // 하드 corridor 제한(기본 AllowAll → 제거됨).
            long long ns = state_of(occ.lin(nb), nidx);
            if (closed.count(ns)) continue;
            double t = g_cur + model.move_cost(nb, prev_off, d);
            auto git = g.find(ns);
            if (git == g.end() || t < git->second) {
                g[ns] = t;
                came[ns] = st;
                open.push({t + weighted_h(nb), counter++, nb, nidx});
            }
        }
    }
    R.expanded_nodes = expanded;
    R.elapsed_ms = detail::ms_since(t0);
    return R;
}

}  // namespace routing3d
