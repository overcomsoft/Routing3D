// 직교 A* 구현 — astar.hpp 참고. 명세 algorithm_spec.md §3,§4.
#include "routing3d/astar.hpp"

#include <chrono>
#include <queue>
#include <unordered_map>
#include <vector>

namespace routing3d {

namespace {

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0) {
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
        if (a.f != b.f) return a.f > b.f;     // 작은 f 가 top
        return a.counter > b.counter;          // 동률이면 먼저 삽입된 것이 top
    }
};

}  // namespace

int count_turns(const std::vector<Cell>& path) {
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

// ---------------------------------------------------------------- 균일 비용 A*
AStarResult astar(const DenseOccupancy& occ, Cell start, Cell goal,
                  double step_cost, long long max_expansions) {
    auto t0 = Clock::now();
    AStarResult R;
    const double sc = (step_cost < 0.0) ? occ.cell_mm() : step_cost;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = ms_since(t0);
        return R;
    }

    auto h = [&](const Cell& c) { return manhattan(c, goal) * sc; };

    std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open;
    std::unordered_map<int, double> g;
    std::unordered_map<int, Cell> came;  // nb 의 lin → 직전 cell
    std::vector<uint8_t> closed(static_cast<size_t>(occ.size()), 0);

    long long counter = 0;
    g[occ.lin(start)] = 0.0;
    open.push({h(start), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        PQItem cur = open.top();
        open.pop();
        int cl = occ.lin(cur.cell);
        if (closed[static_cast<size_t>(cl)]) continue;
        closed[static_cast<size_t>(cl)] = 1;
        ++expanded;

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
            R.elapsed_ms = ms_since(t0);
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
    R.elapsed_ms = ms_since(t0);
    return R;
}

// ------------------------------------------------------------- 비용함수 A*
// 상태 = (셀, 진입방향 dir). dir ∈ [-1,5] → state = lin*7 + (dir+1).
AStarResult astar_weighted(const DenseOccupancy& occ, Cell start, Cell goal,
                           const RouteParams& params, long long max_expansions) {
    auto t0 = Clock::now();
    AStarResult R;
    const double cell_mm = params.cell_mm;

    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = ms_since(t0);
        return R;
    }

    CostModel model(occ, params);
    auto state_of = [&](int lin, int dir) -> long long {
        return static_cast<long long>(lin) * 7 + (dir + 1);
    };

    std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> open;
    std::unordered_map<long long, double> g;
    std::unordered_map<long long, long long> came;  // state → 직전 state
    std::vector<uint8_t> closed(static_cast<size_t>(occ.size()) * 7, 0);

    long long counter = 0;
    long long s0 = state_of(occ.lin(start), -1);
    g[s0] = 0.0;
    open.push({model.heuristic(start, goal), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        PQItem cur = open.top();
        open.pop();
        long long st = state_of(occ.lin(cur.cell), cur.dir);
        if (closed[static_cast<size_t>(st)]) continue;
        closed[static_cast<size_t>(st)] = 1;
        ++expanded;

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
            R.elapsed_ms = ms_since(t0);
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
            if (closed[static_cast<size_t>(ns)]) continue;
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
    R.elapsed_ms = ms_since(t0);
    return R;
}

}  // namespace routing3d
