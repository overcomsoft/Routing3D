// 다중 배관 순차 라우팅 구현 — multi_route.hpp 참고. 명세 algorithm_spec.md §5.
// Python 레퍼런스 routing3d_py/multi_route.py 와 1:1 대응(정렬은 안정 정렬로 동일 결정성).
#include "routing3d/multi_route.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace routing3d {

// ---------------------------------------------------------------- 결과 집계
int MultiRouteResult::success_count() const {
    int n = 0;
    for (const PipeResult& p : pipes)
        if (p.result.success) ++n;
    return n;
}

int MultiRouteResult::fail_count() const {
    return static_cast<int>(pipes.size()) - success_count();
}

double MultiRouteResult::total_length_mm() const {
    double sum = 0.0;
    for (const PipeResult& p : pipes)
        if (p.result.success) sum += p.result.length_mm;
    return sum;
}

double MultiRouteResult::success_rate() const {
    return pipes.empty() ? 0.0
                         : static_cast<double>(success_count()) / static_cast<double>(pipes.size());
}

// ---------------------------------------------------------------- 우선순위
std::vector<RouteTask> order_tasks(const DenseOccupancy& occ,
                                   const std::vector<RouteTask>& tasks,
                                   const std::string& priority) {
    // 작업의 '난이도'(시작-끝 맨해튼 셀 거리). 정렬 키.
    auto dist = [&](const RouteTask& t) {
        return manhattan(occ.to_cell(t.start_mm), occ.to_cell(t.end_mm));
    };

    std::vector<RouteTask> out = tasks;  // 원본 보존(사본 정렬).
    if (priority == "original") return out;

    // Python sorted 는 안정 정렬 → 동률(거리 같음)은 입력 순서 보존. std::stable_sort 로 일치.
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
        // (유틸 라벨 오름차순, 거리 내림차순).
        std::stable_sort(out.begin(), out.end(), [&](const RouteTask& a, const RouteTask& b) {
            const std::string la = a.utility_label(), lb = b.utility_label();
            if (la != lb) return la < lb;
            return dist(a) > dist(b);
        });
        return out;
    }
    throw std::invalid_argument("unknown priority: " + priority);
}

// ---------------------------------------------------------------- 보조: 스냅
Cell snap_to_free_cell(const DenseOccupancy& occ, Cell cell, int radius) {
    if (!occ.is_blocked(cell)) return cell;  // 이미 빈 셀.
    if (radius <= 0) return cell;

    Cell best = cell;
    int best_d = -1;  // 아직 후보 없음.
    // (di,dj,dk) 사전순 순회 → 거리 동률이면 사전순 첫 셀 선택(Python 과 동일 결정성).
    for (int di = -radius; di <= radius; ++di)
        for (int dj = -radius; dj <= radius; ++dj)
            for (int dk = -radius; dk <= radius; ++dk) {
                Cell c{cell.i + di, cell.j + dj, cell.k + dk};
                if (occ.is_blocked(c)) continue;  // 격자 밖/점유는 스킵(is_blocked 가 둘 다 true).
                int d = std::abs(di) + std::abs(dj) + std::abs(dk);
                if (best_d < 0 || d < best_d) {
                    best_d = d;
                    best = c;
                }
            }
    return best;  // 후보가 하나도 없으면 원래 cell.
}

// ---------------------------------------------------------------- 보조: 배관 점유
void mark_pipe(DenseOccupancy& occ, const std::vector<Cell>& path, int radius) {
    for (const Cell& c : path) occ.block_cell(c);
    if (radius <= 0) return;

    // 경계에서 빈 6-이웃을 radius 단계 확장하며 함께 막는다(배관 굵기/이격 근사).
    // 한 셀은 한 번만 막히므로(is_blocked 체크) 다음 라운드 frontier 에 중복 없음.
    std::vector<Cell> frontier = path;
    for (int step = 0; step < radius; ++step) {
        std::vector<Cell> next;
        for (const Cell& c : frontier) {
            for (const Cell& d : NEIGHBORS_6) {
                Cell nb{c.i + d.i, c.j + d.j, c.k + d.k};
                if (occ.in_bounds(nb) && !occ.is_blocked(nb)) {
                    occ.block_cell(nb);
                    next.push_back(nb);
                }
            }
        }
        frontier = std::move(next);
    }
}

// ---------------------------------------------------------------- 순차 라우팅
MultiRouteResult route_sequential(const DenseOccupancy& occ,
                                  const std::vector<RouteTask>& tasks,
                                  const RouteParams& params,
                                  const std::string& priority,
                                  int pipe_radius,
                                  int snap_to_free,
                                  long long max_expansions) {
    DenseOccupancy work = occ.copy();                       // 원본 불변(계약 M2).
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

    return MultiRouteResult{std::move(pipes), std::move(work), priority};
}

}  // namespace routing3d
