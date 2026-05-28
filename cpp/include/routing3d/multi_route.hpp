// 다중 배관 순차 라우팅 (Multi-Pipe Sequential Routing) — Routing3D C++ 엔진 (Phase 3, Step 3.5/3.6)
// =============================================================================
// [이 파일이 하는 일]
//   여러 배관(start→end 작업)을 '한 개씩 차례로' 라우팅한다. 핵심은 **이미 깔린
//   배관을 다음 배관의 장애물로 추가**(mark_pipe)하여 배관끼리 같은 셀을 점유하지
//   않게(충돌 0) 만드는 것이다. greedy sequential 베이스라인(rip-up/CBS 는 후속 단계).
//   Python 레퍼런스 routing3d_py/multi_route.py 와 1:1 대응, 명세 algorithm_spec.md §5.
//
// [전체 흐름]
//   route_sequential(occ, tasks, params, priority, pipe_radius, snap_to_free):
//     1) order_tasks 로 작업 순서 결정(기본 longest = 맨해튼 거리 긴 것 먼저).
//     2) work = occ.copy()  ← 원본 점유맵 불변(계약 M2).
//     3) 작업마다: start/end 가 점유면 snap → astar_weighted → 성공이면 mark_pipe.
//     4) MultiRouteResult(성공수/실패수/성공률/총길이) 반환.
//   계약 M1(충돌 0): 성공 경로들은 쌍별로 셀을 공유하지 않는다.
//
//   Step 3.6: 점유맵 백엔드(Dense/Sparse/Vdb)에 **무관**하도록 템플릿화(헤더 정의).
// =============================================================================
#pragma once

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/route_task.hpp"

namespace routing3d {

// 배관 1개의 라우팅 결과(백엔드 무관).
struct PipeResult {
    RouteTask task;        // 라우팅 작업.
    AStarResult result;    // A* 결과(성공 여부/경로/길이/회전/확장수).
    int order_index = 0;   // 실제 라우팅된 순서(0부터).
};

// 다중 배관 순차 라우팅 결과 묶음. occupancy 는 사용한 백엔드 타입을 그대로 담는다.
template <class Occ>
struct MultiRouteResult {
    std::vector<PipeResult> pipes;  // 라우팅 순서대로의 결과.
    Occ occupancy;                  // 최종 작업용 점유맵(장애물 + 모든 성공 배관).
    std::string priority;           // 사용한 우선순위 규칙 이름.

    int success_count() const {
        int n = 0;
        for (const PipeResult& p : pipes)
            if (p.result.success) ++n;
        return n;
    }
    int fail_count() const { return static_cast<int>(pipes.size()) - success_count(); }
    double total_length_mm() const {
        double s = 0.0;
        for (const PipeResult& p : pipes)
            if (p.result.success) s += p.result.length_mm;
        return s;
    }
    double success_rate() const {
        return pipes.empty() ? 0.0
                             : static_cast<double>(success_count()) / static_cast<double>(pipes.size());
    }
};

// ---- 우선순위 ----
// 우선순위 규칙에 따라 작업 순서를 정렬해 반환한다(원본 변경 없음, 안정 정렬 = Python sorted).
//   longest  : 맨해튼 거리 긴 것 먼저(기본).  shortest : 짧은 것 먼저.
//   utility  : (유틸 라벨 오름차순, 거리 내림차순).  original : 입력 순서.
template <class Occ>
std::vector<RouteTask> order_tasks(const Occ& occ, const std::vector<RouteTask>& tasks,
                                   const std::string& priority) {
    auto dist = [&](const RouteTask& t) {
        return manhattan(occ.to_cell(t.start_mm), occ.to_cell(t.end_mm));
    };
    std::vector<RouteTask> out = tasks;  // 원본 보존(사본 정렬).
    if (priority == "original") return out;
    if (priority == "shortest") {
        std::stable_sort(out.begin(), out.end(),
                         [&](const RouteTask& a, const RouteTask& b) { return dist(a) < dist(b); });
        return out;
    }
    if (priority == "longest") {
        std::stable_sort(out.begin(), out.end(),
                         [&](const RouteTask& a, const RouteTask& b) { return dist(a) > dist(b); });
        return out;
    }
    if (priority == "utility") {
        std::stable_sort(out.begin(), out.end(), [&](const RouteTask& a, const RouteTask& b) {
            const std::string la = a.utility_label(), lb = b.utility_label();
            if (la != lb) return la < lb;
            return dist(a) > dist(b);
        });
        return out;
    }
    throw std::invalid_argument("unknown priority: " + priority);
}

// ---- 보조: 스냅 ----
// cell 이 점유면 반경 radius 내 가장 가까운 빈 셀 반환(없으면 원래 cell).
// 거리 동률은 (di,dj,dk) 사전순 첫 셀(Python 과 동일 결정성).
template <class Occ>
Cell snap_to_free_cell(const Occ& occ, Cell cell, int radius) {
    if (!occ.is_blocked(cell)) return cell;
    if (radius <= 0) return cell;
    Cell best = cell;
    int best_d = -1;
    for (int di = -radius; di <= radius; ++di)
        for (int dj = -radius; dj <= radius; ++dj)
            for (int dk = -radius; dk <= radius; ++dk) {
                Cell c{cell.i + di, cell.j + dj, cell.k + dk};
                if (occ.is_blocked(c)) continue;
                int d = std::abs(di) + std::abs(dj) + std::abs(dk);
                if (best_d < 0 || d < best_d) {
                    best_d = d;
                    best = c;
                }
            }
    return best;
}

// ---- 보조: 배관 점유 ----
// 경로 셀(+반경 radius 6-이웃)을 점유로 표시한다(다음 배관이 피하도록).
template <class Occ>
void mark_pipe(Occ& occ, const std::vector<Cell>& path, int radius) {
    for (const Cell& c : path) occ.block_cell(c);
    if (radius <= 0) return;
    std::vector<Cell> frontier = path;
    for (int step = 0; step < radius; ++step) {
        std::vector<Cell> next;
        for (const Cell& c : frontier)
            for (const Cell& d : NEIGHBORS_6) {
                Cell nb{c.i + d.i, c.j + d.j, c.k + d.k};
                if (occ.in_bounds(nb) && !occ.is_blocked(nb)) {
                    occ.block_cell(nb);
                    next.push_back(nb);
                }
            }
        frontier = std::move(next);
    }
}

// ---- 순차 라우팅 ----
// 배관들을 순차적으로(충돌 없이) 라우팅한다. occ 는 변경하지 않는다(내부 사본 사용).
//   pipe_radius  : 깔린 배관을 점유로 추가할 때 팽창 반경(셀). 0=경로 셀만.
//   snap_to_free : start/end 가 점유면 빈 셀 탐색 반경(셀).
template <class Occ>
MultiRouteResult<Occ> route_sequential(const Occ& occ, const std::vector<RouteTask>& tasks,
                                       const RouteParams& params,
                                       const std::string& priority = "longest",
                                       int pipe_radius = 0, int snap_to_free = 2,
                                       long long max_expansions = -1) {
    Occ work = occ.copy();  // 원본 불변(계약 M2).
    std::vector<RouteTask> ordered = order_tasks(occ, tasks, priority);

    std::vector<PipeResult> pipes;
    pipes.reserve(ordered.size());
    for (int idx = 0; idx < static_cast<int>(ordered.size()); ++idx) {
        const RouteTask& task = ordered[static_cast<size_t>(idx)];
        Cell s = snap_to_free_cell(work, work.to_cell(task.start_mm), snap_to_free);
        Cell g = snap_to_free_cell(work, work.to_cell(task.end_mm), snap_to_free);
        AStarResult res = astar_weighted(work, s, g, params, max_expansions);
        bool ok = res.success && !res.path.empty();
        std::vector<Cell> path = res.path;  // mark_pipe 용(res 는 이동).
        pipes.push_back(PipeResult{task, std::move(res), idx});
        if (ok) mark_pipe(work, path, pipe_radius);  // 깔린 경로를 점유로 추가.
    }

    return MultiRouteResult<Occ>{std::move(pipes), std::move(work), priority};
}

}  // namespace routing3d
