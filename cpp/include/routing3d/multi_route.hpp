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
#include <cstdint>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_set>
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
// 우선순위 규칙에 따른 '원본 인덱스 순열'을 반환한다(안정 정렬 = Python sorted 결정성).
//   longest  : 맨해튼 거리 긴 것 먼저(기본).  shortest : 짧은 것 먼저.
//   utility  : (유틸 라벨 오름차순, 거리 내림차순).  original : 입력 순서.
template <class Occ>
std::vector<int> order_indices(const Occ& occ, const std::vector<RouteTask>& tasks,
                               const std::string& priority) {
    auto dist = [&](int t) {
        return manhattan(occ.to_cell(tasks[static_cast<size_t>(t)].start_mm),
                         occ.to_cell(tasks[static_cast<size_t>(t)].end_mm));
    };
    std::vector<int> idx(tasks.size());
    for (int i = 0; i < static_cast<int>(tasks.size()); ++i) idx[static_cast<size_t>(i)] = i;
    if (priority == "original") return idx;
    if (priority == "shortest") {
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) { return dist(a) < dist(b); });
        return idx;
    }
    if (priority == "longest") {
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) { return dist(a) > dist(b); });
        return idx;
    }
    if (priority == "utility") {
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
            const std::string la = tasks[static_cast<size_t>(a)].utility_label();
            const std::string lb = tasks[static_cast<size_t>(b)].utility_label();
            if (la != lb) return la < lb;
            return dist(a) > dist(b);
        });
        return idx;
    }
    throw std::invalid_argument("unknown priority: " + priority);
}

// 우선순위 규칙에 따라 작업 순서를 정렬해 반환한다(원본 변경 없음). order_indices 위에 구현.
template <class Occ>
std::vector<RouteTask> order_tasks(const Occ& occ, const std::vector<RouteTask>& tasks,
                                   const std::string& priority) {
    std::vector<int> idx = order_indices(occ, tasks, priority);
    std::vector<RouteTask> out;
    out.reserve(tasks.size());
    for (int i : idx) out.push_back(tasks[static_cast<size_t>(i)]);
    return out;
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

// ---- 보조: 회랑 성장 ----
// 경로 셀 + radius 6-이웃(격자 내)을 회랑(occ.lin 인덱스) 집합에 추가한다(Python _add_corridor 와 1:1).
// 이후 배관이 이 곁을 '싸게'(w_corridor 면제) 지나가 공용 랙으로 뭉친다 → 기존 설계 유사.
template <class Occ>
void add_corridor_cells(const Occ& occ, std::unordered_set<int>& corridor,
                        const std::vector<Cell>& path, int radius) {
    std::vector<Cell> frontier;
    for (const Cell& c : path) {
        if (occ.in_bounds(c)) { corridor.insert(occ.lin(c)); frontier.push_back(c); }
    }
    for (int r = 0; r < (radius > 0 ? radius : 0); ++r) {
        std::vector<Cell> next;
        for (const Cell& c : frontier)
            for (const Cell& d : NEIGHBORS_6) {
                Cell nb{c.i + d.i, c.j + d.j, c.k + d.k};
                if (occ.in_bounds(nb) && corridor.insert(occ.lin(nb)).second) next.push_back(nb);
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
                                       long long max_expansions = -1,
                                       bool collect_visited = false,
                                       int corridor_radius = 1) {
    Occ work = occ.copy();  // 원본 불변(계약 M2).
    std::vector<RouteTask> ordered = order_tasks(occ, tasks, priority);

    // 회랑 인력(w_corridor>0)이면 깔린 배관 곁을 회랑으로 키워 다음 배관을 끌어모은다.
    std::unordered_set<int> corridor;
    const bool use_corridor = params.w_corridor > 0.0;

    std::vector<PipeResult> pipes;
    pipes.reserve(ordered.size());
    for (int idx = 0; idx < static_cast<int>(ordered.size()); ++idx) {
        const RouteTask& task = ordered[static_cast<size_t>(idx)];
        Cell s = snap_to_free_cell(work, work.to_cell(task.start_mm), snap_to_free);
        Cell g = snap_to_free_cell(work, work.to_cell(task.end_mm), snap_to_free);
        AStarResult res = astar_weighted(work, s, g, params, max_expansions, collect_visited,
                                         use_corridor ? &corridor : nullptr);
        bool ok = res.success && !res.path.empty();
        std::vector<Cell> path = res.path;  // mark_pipe 용(res 는 이동).
        pipes.push_back(PipeResult{task, std::move(res), idx});
        if (ok) {
            mark_pipe(work, path, pipe_radius);  // 깔린 경로를 점유로 추가.
            if (use_corridor) add_corridor_cells(work, corridor, path, corridor_radius);
        }
    }

    return MultiRouteResult<Occ>{std::move(pipes), std::move(work), priority};
}

// ---- rip-up & reroute (Step 3.8) ----
// 순차 라우팅 후 막힌 배관을, '장애물만' 이상 경로가 가로지르는 기존 배관(blocker)을
// 뜯어내고 재배치해 해소한다. **무손실(채택 시 성공 +1)** 결정적 알고리즘:
//   라운드마다 실패 배관 f 의 장애물-only 이상 경로가 통과하는 placed 배관을 blocker 로
//   잡아 모두 뜯어내고, f 를 깐 뒤 blocker 를 (키 오름차순) 전부 재라우팅 → f 성공 +
//   모든 blocker 재배치 성공일 때만 채택. 하나라도 실패하면 시도를 버린다.
// 성공 수는 단조 증가(채택 +1)하므로 라운드/시도는 유한(초기 실패 수 상한). Python
// routing3d_py.multi_route.route_ripup 와 1:1 대응(동일 occ 상태·순서 → 동일 결과).
template <class Occ>
MultiRouteResult<Occ> route_ripup(const Occ& occ, const std::vector<RouteTask>& tasks,
                                  const RouteParams& params,
                                  const std::string& priority = "longest",
                                  int pipe_radius = 0, int snap_to_free = 2,
                                  long long max_expansions = -1, int max_rounds = 10,
                                  int max_ripup = 4, bool collect_visited = false) {
    std::vector<RouteTask> ordered = order_tasks(occ, tasks, priority);
    const int n = static_cast<int>(ordered.size());
    Occ static_occ = occ.copy();  // 장애물만(불변 기준).

    auto pack = [](const Cell& c) -> uint64_t {
        return (static_cast<uint64_t>(c.i) << 42) | (static_cast<uint64_t>(c.j) << 21) |
               static_cast<uint64_t>(c.k);
    };
    auto route_on = [&](const Occ& w, const RouteTask& t) -> AStarResult {
        Cell s = snap_to_free_cell(w, w.to_cell(t.start_mm), snap_to_free);
        Cell g = snap_to_free_cell(w, w.to_cell(t.end_mm), snap_to_free);
        return astar_weighted(w, s, g, params, max_expansions, collect_visited);
    };
    auto build_work = [&](const std::map<int, std::vector<Cell>>& paths) -> Occ {
        Occ w = static_occ.copy();
        for (const auto& kv : paths) mark_pipe(w, kv.second, pipe_radius);
        return w;
    };

    std::map<int, std::vector<Cell>> placed;          // 정렬 인덱스 → 경로(결정적 반복).
    std::vector<AStarResult> results(static_cast<size_t>(n));
    std::vector<char> have(static_cast<size_t>(n), 0);

    // 1) 베이스라인 순차 라우팅(route_sequential 과 동일).
    {
        Occ work = static_occ.copy();
        for (int idx = 0; idx < n; ++idx) {
            AStarResult res = route_on(work, ordered[static_cast<size_t>(idx)]);
            bool ok = res.success && !res.path.empty();
            if (ok) {
                mark_pipe(work, res.path, pipe_radius);
                placed[idx] = res.path;
            }
            results[static_cast<size_t>(idx)] = std::move(res);
            have[static_cast<size_t>(idx)] = ok ? 1 : 0;
        }
    }

    // 2) rip-up 라운드.
    for (int round = 0; round < max_rounds; ++round) {
        std::vector<int> failed;
        for (int idx = 0; idx < n; ++idx)
            if (!have[static_cast<size_t>(idx)]) failed.push_back(idx);
        if (failed.empty()) break;

        bool changed = false;
        for (int f : failed) {
            AStarResult ideal = route_on(static_occ, ordered[static_cast<size_t>(f)]);
            if (!(ideal.success && !ideal.path.empty())) continue;  // 장애물만으로도 불가.

            std::unordered_set<uint64_t> cellset;
            cellset.reserve(ideal.path.size() * 2);
            for (const Cell& c : ideal.path) cellset.insert(pack(c));

            std::vector<int> blockers;  // placed 키 오름차순(std::map) → 결정적.
            for (const auto& kv : placed) {
                for (const Cell& c : kv.second)
                    if (cellset.count(pack(c))) {
                        blockers.push_back(kv.first);
                        break;
                    }
            }
            if (blockers.empty() || static_cast<int>(blockers.size()) > max_ripup) continue;

            std::map<int, std::vector<Cell>> trial = placed;
            for (int b : blockers) trial.erase(b);
            Occ wt = build_work(trial);

            AStarResult rf = route_on(wt, ordered[static_cast<size_t>(f)]);
            if (!(rf.success && !rf.path.empty())) continue;
            mark_pipe(wt, rf.path, pipe_radius);
            trial[f] = rf.path;

            std::vector<AStarResult> reres(blockers.size());
            bool all_ok = true;
            for (size_t bi = 0; bi < blockers.size(); ++bi) {
                AStarResult rb = route_on(wt, ordered[static_cast<size_t>(blockers[bi])]);
                bool bok = rb.success && !rb.path.empty();
                if (bok) {
                    mark_pipe(wt, rb.path, pipe_radius);
                    trial[blockers[bi]] = rb.path;
                } else {
                    all_ok = false;
                }
                reres[bi] = std::move(rb);
            }

            if (all_ok) {  // 무손실일 때만 채택(성공 +1).
                placed = std::move(trial);
                results[static_cast<size_t>(f)] = std::move(rf);
                have[static_cast<size_t>(f)] = 1;
                for (size_t bi = 0; bi < blockers.size(); ++bi) {
                    results[static_cast<size_t>(blockers[bi])] = std::move(reres[bi]);
                    have[static_cast<size_t>(blockers[bi])] = 1;
                }
                changed = true;
            }
        }
        if (!changed) break;
    }

    std::vector<PipeResult> pipes;
    pipes.reserve(static_cast<size_t>(n));
    for (int idx = 0; idx < n; ++idx)
        pipes.push_back(PipeResult{ordered[static_cast<size_t>(idx)],
                                   std::move(results[static_cast<size_t>(idx)]), idx});
    return MultiRouteResult<Occ>{std::move(pipes), build_work(placed), priority};
}

}  // namespace routing3d
