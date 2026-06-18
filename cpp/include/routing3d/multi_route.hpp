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
//
// [핵심 불변식]
//   M1 — 성공한 배관들은 쌍별로 셀을 공유하지 않는다(충돌 0).
//   M2 — 원본 점유맵(occ)은 어떤 함수도 수정하지 않는다. 항상 occ.copy() 사본에서 작업.
//   A2/W1 — 동일 입력 → 동일 경로/확장수(결정성). stable_sort + (f, counter) tie-break.
//
// [관련 파일]
//   routing3d_py/multi_route.py   — Python 레퍼런스(1:1 대응)
//   routing3d/astar.hpp           — astar_weighted (단일 배관 A* 탐색)
//   routing3d/route_task.hpp      — RouteTask(작업 정의, 관경/목표진입축 포함)
//   routing3d/cost.hpp            — RouteParams(w_corridor, w_turn, clearance_radius 등)
//   cpp/capi/routing3d_capi.cpp   — route_multi_impl (최종 엔트리 포인트, 이 파일 함수들 조합)
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

// =============================================================================
// 배관 1개의 라우팅 결과(백엔드 무관).
//   task        : 원본 RouteTask(start_mm, end_mm, diameter_mm, goal_dir 등).
//   result      : astar_weighted 가 반환한 AStarResult.
//                 result.success=false 이면 경로 없음(혼잡/막힘/탐색상한 초과).
//   order_index : 이 배관이 실제로 라우팅된 순서(priority 정렬 후 인덱스).
//                 C# 뷰어의 TaskRowVM.RouteOrder 에 표시, 결과 리포트에 기록됨.
// =============================================================================
struct PipeResult {
    RouteTask task;        // 라우팅 작업.
    AStarResult result;    // A* 결과(성공 여부/경로/길이/회전/확장수).
    int order_index = 0;   // 실제 라우팅된 순서(0부터).
};

// =============================================================================
// 다중 배관 순차 라우팅 결과 묶음. occupancy 는 사용한 백엔드 타입을 그대로 담는다.
//   pipes     : priority 정렬 후 라우팅된 순서대로의 PipeResult 배열.
//   occupancy : 장애물 + 모든 성공 배관이 마킹된 최종 작업용 점유맵.
//               route_ripup/route_multi_impl 에서 다음 단계(CBS, min-straight)의
//               기초 점유 상태로 사용된다.
//   priority  : 사용한 우선순위 규칙 이름(결과 분석·로그용).
//
// [집계 메서드]
//   success_count() : 성공 배관 수.
//   fail_count()    : 실패 배관 수.
//   total_length_mm : 성공 배관의 총 경로 길이(mm).
//   success_rate()  : 성공률 [0.0, 1.0].
// =============================================================================
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

// =============================================================================
// ---- 우선순위 정렬: order_indices ----
// 우선순위 규칙에 따른 '원본 인덱스 순열'을 반환한다(안정 정렬 = Python sorted 결정성).
//
// [정렬 모드 상세]
//   "longest"  : 맨해튼 거리(셀 단위) 긴 것 먼저(기본값).
//                장거리 배관이 먼저 최단 경로를 선점하고, 단거리 배관은 나머지 공간 활용.
//
//   "shortest" : 맨해튼 거리 짧은 것 먼저. 가까운 배관이 혼잡 지점 근처에 먼저 자리 잡음.
//
//   "diameter" : [Phase P3q B1] 굵은 배관(diameter_mm 큰 것) 먼저, 동률은 거리 긴 것.
//                굵은 배관이 직선(최단) 경로를 선점하고 가는 배관이 그 옆을 피해 가도록 한다.
//                → 굵은 배관이 마지막에 깔려 우회·충돌하던 문제(사용자 지적)를 해소.
//                → 관경 미상(diameter_mm=0)이면 모두 동률 → "longest"와 동일 동작(골든 불변).
//                → 실측(Exhaust 20작업): "longest" 135꺾임 → "diameter" 135꺾임, 총길이 ↓.
//
//   "utility"  : (유틸 라벨 오름차순 → 굵은 배관 먼저 → 거리 내림차순).
//                같은 유틸 그룹(Exhaust/Water 등)을 연속 배치해 번들(다발) 형성을 돕고,
//                묶음 안에서도 굵은 배관이 먼저 직선 레인을 잡는다.
//                관경 0 이면 (유틸, 거리) 정렬과 동일(기존 동작 불변).
//
//   "original" : 입력 순서 유지(정렬 없음). 주로 테스트/재현용.
//
// [결정성 보장]
//   std::stable_sort 를 사용해 동률(같은 키) 항목은 입력 순서 유지 → Python sorted 와 동일.
//   dist, dia 는 tasks[] 조회만 하고 occ 상태를 변경하지 않는다.
// =============================================================================
template <class Occ>
std::vector<int> order_indices(const Occ& occ, const std::vector<RouteTask>& tasks,
                               const std::string& priority) {
    // 맨해튼 거리: 셀 좌표 차이의 절댓값 합(이동 비용 추정치).
    auto dist = [&](int t) {
        return manhattan(occ.to_cell(tasks[static_cast<size_t>(t)].start_mm),
                         occ.to_cell(tasks[static_cast<size_t>(t)].end_mm));
    };
    // 관경(mm). 미설정(0)이면 모든 배관이 동률 → "longest"와 동일.
    auto dia = [&](int t) { return tasks[static_cast<size_t>(t)].diameter_mm; };
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
    if (priority == "diameter") {
        // 1차 키: 관경 내림차순(굵은 배관 먼저).
        // 2차 키: 맨해튼 거리 내림차순(동일 관경이면 긴 것 먼저).
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
            if (dia(a) != dia(b)) return dia(a) > dia(b);
            return dist(a) > dist(b);
        });
        return idx;
    }
    if (priority == "utility") {
        // 1차 키: 유틸 라벨 오름차순(같은 유틸 연속 배치 → 번들 형성).
        // 2차 키: 관경 내림차순(묶음 안에서 굵은 배관 먼저).
        // 3차 키: 맨해튼 거리 내림차순(동일 관경이면 긴 것 먼저).
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
            const std::string la = tasks[static_cast<size_t>(a)].utility_label();
            const std::string lb = tasks[static_cast<size_t>(b)].utility_label();
            if (la != lb) return la < lb;
            if (dia(a) != dia(b)) return dia(a) > dia(b);
            return dist(a) > dist(b);
        });
        return idx;
    }
    throw std::invalid_argument("unknown priority: " + priority);
}

// =============================================================================
// order_tasks — 우선순위 규칙으로 정렬된 RouteTask 배열 반환
//
// order_indices 로 순열을 구한 뒤, 해당 순서로 tasks 를 복사해 반환한다.
// 원본 tasks 는 수정하지 않는다.
// route_sequential / route_ripup 이 이 함수를 호출해 라우팅 순서를 결정한다.
// =============================================================================
template <class Occ>
std::vector<RouteTask> order_tasks(const Occ& occ, const std::vector<RouteTask>& tasks,
                                   const std::string& priority) {
    std::vector<int> idx = order_indices(occ, tasks, priority);
    std::vector<RouteTask> out;
    out.reserve(tasks.size());
    for (int i : idx) out.push_back(tasks[static_cast<size_t>(i)]);
    return out;
}

// =============================================================================
// snap_to_free_cell — 점유된 시작/목표 셀을 가장 가까운 빈 셀로 이동
//
// [목적]
//   배관 종단(PoC)이 장애물에 파묻혀 있을 때(파묻힌 PoC),
//   반경 radius 맨해튼 이웃 중 가장 가까운 비점유 셀을 찾아 반환한다.
//   장애물이 없으면(is_blocked==false) cell 을 그대로 반환.
//   radius 안에 빈 셀이 없으면 원래 cell 반환(A* 가 exp=0 으로 즉시 실패).
//
// [결정성]
//   di→dj→dk 삼중 루프 순서(Python enumerate 와 동일) + 최소 맨해튼 거리 첫 승자 →
//   동일 입력에서 항상 같은 셀 선택(A2/W1 불변식 지원).
//
// [파라미터]
//   occ    : 현재 점유맵(작업 사본 work, 이미 깔린 배관 포함).
//   cell   : 변환할 대상 셀(to_cell(start_mm) 또는 to_cell(end_mm) 결과).
//   radius : 탐색 반경(셀). route_sequential/ripup 은 기본 2, per-task 반경(B1) 보정 시
//            2+pipe_radius 로 키워 팽창된 이웃 배관에 묻힌 PoC 를 구출한다.
// =============================================================================
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

// =============================================================================
// mark_pipe — 완료된 배관 경로를 점유맵에 기록(다음 배관이 충돌하지 않도록)
//
// [동작]
//   Step 1: 경로 셀(path) 전체를 occ.block_cell 로 점유.
//   Step 2: radius > 0 이면 BFS 로 radius 단계씩 6-이웃을 추가 점유(팽창).
//           → 센터선에서 radius 셀 거리만큼 이격 공간을 확보해 다음 배관이
//             물리적으로 인접 배치(표면 맞닿음)되는 것을 방지.
//
// [radius 결정 방식]
//   radius = 0 : 셀 중심선만 점유(충돌 0 최소 보장).
//   radius > 0 : per-task 반경(B1) — route_multi_impl 의 radius_of() 공식:
//                radius_of(d_mm, cell_mm) = clamp(ceil(d_mm / cell_mm) - 1, 0, 8).
//                예) d=150mm, cell=25mm → radius = ceil(6.0)-1 = 5.
//                쌍(pairwise) 마킹(pipe_gap_mm, Phase P3s): 이미 깔린 배관을
//                ceil((r_a + r_b + gap_mm) / cell_mm) 반경으로 재마킹해
//                센터선 거리 ≥ r_a + r_b + gap_mm 보장.
//
// [불변식]
//   이 함수는 항상 occ 의 사본(work)에 대해 호출된다. 원본 occ 는 절대 수정하지 않음(M2).
//   BFS 팽창은 이미 점유된 셀을 건너뛰어 중복 방문을 피한다.
//
// [파라미터]
//   occ    : 작업용 점유맵 사본(work). 변경 가능.
//   path   : A* 가 반환한 경로 셀 목록.
//   radius : 팽창 반경(셀). 0 이면 경로 셀만 점유.
// =============================================================================
template <class Occ>
void mark_pipe(Occ& occ, const std::vector<Cell>& path, int radius) {
    // Step 1: 경로 셀 직접 점유.
    for (const Cell& c : path) occ.block_cell(c);
    if (radius <= 0) return;
    // Step 2: BFS 팽창 — radius 단계만큼 6-이웃을 순서대로 점유(팽창 껍질 단위).
    std::vector<Cell> frontier = path;
    for (int step = 0; step < radius; ++step) {
        std::vector<Cell> next;
        for (const Cell& c : frontier)
            for (const Cell& d : NEIGHBORS_6) {
                Cell nb{c.i + d.i, c.j + d.j, c.k + d.k};
                if (occ.in_bounds(nb) && !occ.is_blocked(nb)) {
                    occ.block_cell(nb);
                    next.push_back(nb);  // 이번 단계에 새로 점유된 셀만 다음 frontier 로.
                }
            }
        frontier = std::move(next);
    }
}

// =============================================================================
// add_corridor_cells — 완료된 배관 경로 주변을 '회랑'(공용 랙 유인 영역)으로 등록
//
// [목적]
//   이미 깔린 배관의 경로 셀과 그 반경 radius 이웃(6-이웃 BFS)을 corridor 집합에 추가.
//   후속 배관이 astar_weighted(..., &corridor) 로 탐색할 때, corridor 셀을 통과하면
//   RouteParams.w_corridor 만큼 비용 할인 → 배관들이 자연스럽게 같은 '랙' 레인으로 묶임.
//   Python routing3d_py.multi_route._add_corridor 와 1:1 대응.
//
// [파라미터]
//   occ      : 점유맵(경계 검사용, 수정 없음).
//   corridor : 누적되는 회랑 셀 집합(lin 인덱스 = occ.lin(cell)).
//   path     : 이번에 깔린 배관의 경로 셀.
//   radius   : 회랑 팽창 반경(기본 1셀 → 배관 바로 옆 레인도 할인 대상).
//
// [활성 조건]
//   route_sequential 에서 params.w_corridor > 0.0 일 때만 호출.
//   w_corridor = 0 이면 corridor 집합 미사용(순수 A* 비용).
// =============================================================================
template <class Occ>
void add_corridor_cells(const Occ& occ, std::unordered_set<long long>& corridor,
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

// =============================================================================
// route_sequential — 핵심 순차 배관 라우팅 루프
//
// [알고리즘 개요]
//   배관들을 priority 순서대로 한 번씩 A* 탐색하고, 성공 시 경로를 점유맵에 마킹한다.
//   실패 배관은 그대로 건너뜀(rip-up 없음). 가장 단순하고 빠른 베이스라인.
//
// [처리 단계]
//   Step 1) order_tasks 로 작업을 priority 기준 정렬.
//   Step 2) work = occ.copy() — 원본 점유맵 불변 보장(계약 M2).
//           work 에는 이후 성공 배관들이 순차로 추가된다.
//   Step 3) 각 작업마다:
//     3-1) snap_to_free_cell — start/end 가 점유된 셀이면 빈 셀로 이동.
//     3-2) astar_weighted — work 기준으로 경로 탐색. goal_dir 진입축 제약 적용.
//     3-3) goal_dir 제약 실패 시 — 무제약(-1)으로 1회 재시도(연결 성공률 우선).
//     3-4) 성공이면 mark_pipe(work, path, pipe_radius) 로 경로 점유 마킹.
//          w_corridor>0 이면 add_corridor_cells 로 회랑도 확장(다음 배관 유인).
//   Step 4) MultiRouteResult 반환(pipes 배열 + 최종 work 점유맵).
//
// [불변식]
//   M1 — 성공 경로들은 쌍별 셀 공유 없음(work 에 이미 마킹된 셀은 A* 가 우회).
//   M2 — occ 수정 없음(work 는 occ.copy() 사본).
//   A2 — 동일 입력 → 동일 순서·경로(stable_sort + A* (f,counter) tie-break).
//
// [파라미터]
//   occ           : 초기 점유맵(장애물만). 읽기 전용.
//   tasks         : 라우팅할 작업 목록. order_tasks 가 복사·정렬.
//   params        : A* 비용 파라미터(w_turn, w_corridor, clearance_radius 등).
//   priority      : 정렬 모드("longest"|"shortest"|"diameter"|"utility"|"original").
//   pipe_radius   : mark_pipe 팽창 반경(셀). 0=셀 중심선만, >0=이격 공간 확보.
//   snap_to_free  : snap_to_free_cell 탐색 반경(셀). 기본 2.
//   max_expansions: A* 탐색 노드 상한(-1=무제한). 거대 격자 탐색 폭발 방지.
//   collect_visited: A* 방문 셀 수집 여부(3D 탐색 시각화용, 기본 OFF).
//   corridor_radius: add_corridor_cells 팽창 반경(셀). w_corridor>0 일 때만 사용.
// =============================================================================
template <class Occ>
MultiRouteResult<Occ> route_sequential(const Occ& occ, const std::vector<RouteTask>& tasks,
                                       const RouteParams& params,
                                       const std::string& priority = "longest",
                                       int pipe_radius = 0, int snap_to_free = 2,
                                       long long max_expansions = -1,
                                       bool collect_visited = false,
                                       int corridor_radius = 1) {
    // Step 2: 원본 불변(계약 M2) — 이후 모든 점유 수정은 work 에만.
    Occ work = occ.copy();
    // Step 1: 우선순위 정렬. ordered 는 tasks 의 복사본(원본 tasks 불변).
    std::vector<RouteTask> ordered = order_tasks(occ, tasks, priority);

    // 회랑 인력(w_corridor>0)이면 깔린 배관 곁을 회랑으로 키워 다음 배관을 끌어모은다.
    std::unordered_set<long long> corridor;
    const bool use_corridor = params.w_corridor > 0.0;

    std::vector<PipeResult> pipes;
    pipes.reserve(ordered.size());
    for (int idx = 0; idx < static_cast<int>(ordered.size()); ++idx) {
        const RouteTask& task = ordered[static_cast<size_t>(idx)];

        // Step 3-1: 시작/목표 셀이 점유면 빈 셀로 스냅.
        Cell s = snap_to_free_cell(work, work.to_cell(task.start_mm), snap_to_free);
        Cell g = snap_to_free_cell(work, work.to_cell(task.end_mm), snap_to_free);

        // Step 3-2: A* 탐색. goal_dir 진입축 제약 적용(P3o).
        AStarResult res = astar_weighted(work, s, g, params, max_expansions, collect_visited,
                                         use_corridor ? &corridor : nullptr, nullptr, 0,
                                         AllowAll{}, task.goal_dir);
        // Step 3-3: goal_dir 제약으로 실패하면 무제약(-1)으로 1회 폴백 — 연결 우선(성공률 보존).
        if ((!res.success || res.path.empty()) && task.goal_dir >= 0)
            res = astar_weighted(work, s, g, params, max_expansions, collect_visited,
                                 use_corridor ? &corridor : nullptr, nullptr, 0, AllowAll{}, -1);
        bool ok = res.success && !res.path.empty();
        std::vector<Cell> path = res.path;  // mark_pipe 용 사본(res 는 이후 move 됨).
        pipes.push_back(PipeResult{task, std::move(res), idx});

        // Step 3-4: 성공 경로를 점유(+회랑)에 마킹.
        if (ok) {
            mark_pipe(work, path, pipe_radius);  // 깔린 경로를 점유로 추가.
            if (use_corridor) add_corridor_cells(work, corridor, path, corridor_radius);
        }
    }

    return MultiRouteResult<Occ>{std::move(pipes), std::move(work), priority};
}

// =============================================================================
// route_ripup — Rip-Up & Reroute (Step 3.8)
//
// [알고리즘 개요]
//   순차 라우팅 후에도 실패한 배관을 해소하기 위한 후처리 전략.
//   실패 배관 f 의 '직접 blocker'(f 의 이상 경로를 가로막는 기존 배관)를 뜯어내고
//   재배치하여, 무손실(성공 수 단조 증가)로 추가 배관을 연결한다.
//
// [처리 단계]
//   Step 1) 베이스라인 순차 라우팅(route_sequential 과 동일 로직).
//            placed: 성공 배관의 (정렬 인덱스 → 경로) 맵(std::map = 키 오름차순 → 결정적).
//   Step 2) rip-up 라운드 반복(최대 max_rounds 회):
//     2-1) 실패 배관 목록 수집. 없으면 종료.
//     2-2) 각 실패 배관 f 에 대해:
//       (a) 장애물만 있는 static_occ 에서 f 의 이상 경로(ideal) 탐색.
//           ideal 이 없으면 장애물 자체가 막음 — rip-up 대상 없음, 건너뜀.
//       (b) ideal 경로 셀 집합(cellset) 구성.
//       (c) placed 에서 cellset 과 교차하는 배관을 blockers 로 수집
//           (std::map 순서 = 오름차순 → 결정적).
//       (d) blockers 수 > max_ripup 이면 건너뜀(너무 많은 배관을 흔드는 것 방지).
//       (e) trial = placed - blockers. build_work(trial) 로 임시 점유맵 wt 구성.
//       (f) wt 에서 f 를 탐색(rf). 실패하면 포기.
//       (g) wt 에 rf 마킹 후, blockers 를 순서대로 재탐색(reres).
//       (h) f + 모든 blockers 가 **모두** 성공이면 채택(무손실 조건).
//           하나라도 실패하면 trial 버림(placed 불변).
//     2-3) 라운드에서 changed(채택 1건 이상)가 없으면 조기 종료.
//
// [무손실 불변식]
//   채택 시 성공 수 +1 (f 성공 + 모든 blockers 성공 = 기존 성공 수 유지 + f 추가).
//   성공 수는 단조 증가 → 라운드/시도는 유한(초기 실패 수 상한으로 종료 보장).
//
// [결정성]
//   std::map<int,...> placed: 키(정렬 인덱스) 오름차순 → 동일 입력 → 동일 blockers 순서.
//   pack(): Cell → uint64_t 패킹(i<<42 | j<<21 | k) → 집합 연산 결정적.
//   route_on, build_work, mark_pipe 모두 상태 무관 → A2/W1 보장.
//
// [Python 대응]
//   routing3d_py.multi_route.route_ripup 와 1:1 대응.
//   동일 occ 상태·정렬 순서에서 동일 결과를 반환한다.
//
// [파라미터]
//   max_rounds : rip-up 라운드 상한(기본 10). 보통 1~2 라운드면 수렴.
//   max_ripup  : 한 번에 뜯어낼 수 있는 blocker 최대 수(기본 4).
//                너무 크면 연쇄 재배치가 실패할 가능성 증가.
// =============================================================================
template <class Occ>
MultiRouteResult<Occ> route_ripup(const Occ& occ, const std::vector<RouteTask>& tasks,
                                  const RouteParams& params,
                                  const std::string& priority = "longest",
                                  int pipe_radius = 0, int snap_to_free = 2,
                                  long long max_expansions = -1, int max_rounds = 10,
                                  int max_ripup = 4, bool collect_visited = false) {
    std::vector<RouteTask> ordered = order_tasks(occ, tasks, priority);
    const int n = static_cast<int>(ordered.size());
    // static_occ: 장애물만 있는 불변 기준 점유맵.
    // build_work 가 항상 이것을 복사해 placed 배관만 추가하므로 M2 보장.
    Occ static_occ = occ.copy();

    // 셀 → 64비트 키(충돌 검사용 해시). i<42, j<21, k<0 로 패킹(축당 21비트).
    auto pack = [](const Cell& c) -> uint64_t {
        return (static_cast<uint64_t>(c.i) << 42) | (static_cast<uint64_t>(c.j) << 21) |
               static_cast<uint64_t>(c.k);
    };
    // route_on: 주어진 점유맵 w 에서 작업 t 를 탐색. goal_dir 폴백 포함.
    auto route_on = [&](const Occ& w, const RouteTask& t) -> AStarResult {
        Cell s = snap_to_free_cell(w, w.to_cell(t.start_mm), snap_to_free);
        Cell g = snap_to_free_cell(w, w.to_cell(t.end_mm), snap_to_free);
        AStarResult res = astar_weighted(w, s, g, params, max_expansions, collect_visited,
                                         nullptr, nullptr, 0, AllowAll{}, t.goal_dir);
        if ((!res.success || res.path.empty()) && t.goal_dir >= 0)   // 진입축 막힘 → 무제약 폴백.
            res = astar_weighted(w, s, g, params, max_expansions, collect_visited);
        return res;
    };
    // build_work: static_occ(장애물만) 복사 후 paths 에 있는 배관들만 mark_pipe.
    // 이것이 M1/M2 불변식을 유지하는 핵심 — 항상 깨끗한 기준에서 재구성.
    auto build_work = [&](const std::map<int, std::vector<Cell>>& paths) -> Occ {
        Occ w = static_occ.copy();
        for (const auto& kv : paths) mark_pipe(w, kv.second, pipe_radius);
        return w;
    };

    std::map<int, std::vector<Cell>> placed;          // 정렬 인덱스 → 경로(결정적 반복).
    std::vector<AStarResult> results(static_cast<size_t>(n));
    std::vector<char> have(static_cast<size_t>(n), 0);

    // Step 1: 베이스라인 순차 라우팅(route_sequential 과 동일).
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

    // Step 2: rip-up 라운드 반복.
    for (int round = 0; round < max_rounds; ++round) {
        // Step 2-1: 실패 배관 목록. 없으면 이미 전 배관 성공 → 조기 종료.
        std::vector<int> failed;
        for (int idx = 0; idx < n; ++idx)
            if (!have[static_cast<size_t>(idx)]) failed.push_back(idx);
        if (failed.empty()) break;

        bool changed = false;
        for (int f : failed) {
            // Step 2-2(a): 장애물만 있는 static_occ 에서 이상 경로 탐색.
            AStarResult ideal = route_on(static_occ, ordered[static_cast<size_t>(f)]);
            if (!(ideal.success && !ideal.path.empty())) continue;  // 장애물 자체가 막음 — 건너뜀.

            // Step 2-2(b): 이상 경로 셀 집합 구성.
            std::unordered_set<uint64_t> cellset;
            cellset.reserve(ideal.path.size() * 2);
            for (const Cell& c : ideal.path) cellset.insert(pack(c));

            // Step 2-2(c): cellset 과 교차하는 placed 배관 = blockers.
            //              std::map 의 오름차순 순회로 결정적 순서 보장.
            std::vector<int> blockers;
            for (const auto& kv : placed) {
                for (const Cell& c : kv.second)
                    if (cellset.count(pack(c))) {
                        blockers.push_back(kv.first);
                        break;
                    }
            }
            // Step 2-2(d): blocker 수 상한 초과 → 건너뜀.
            if (blockers.empty() || static_cast<int>(blockers.size()) > max_ripup) continue;

            // Step 2-2(e): trial = placed - blockers. 임시 점유맵 wt 구성.
            std::map<int, std::vector<Cell>> trial = placed;
            for (int b : blockers) trial.erase(b);
            Occ wt = build_work(trial);

            // Step 2-2(f): wt 에서 f 탐색. 실패하면 포기(trial 버림).
            AStarResult rf = route_on(wt, ordered[static_cast<size_t>(f)]);
            if (!(rf.success && !rf.path.empty())) continue;
            mark_pipe(wt, rf.path, pipe_radius);
            trial[f] = rf.path;

            // Step 2-2(g): blockers 를 순서대로 재탐색(앞서 깔린 배관 반영).
            std::vector<AStarResult> reres(blockers.size());
            bool all_ok = true;
            for (size_t bi = 0; bi < blockers.size(); ++bi) {
                AStarResult rb = route_on(wt, ordered[static_cast<size_t>(blockers[bi])]);
                bool bok = rb.success && !rb.path.empty();
                if (bok) {
                    mark_pipe(wt, rb.path, pipe_radius);
                    trial[blockers[bi]] = rb.path;
                } else {
                    all_ok = false;  // 하나라도 실패하면 무손실 조건 위반 → 채택 불가.
                }
                reres[bi] = std::move(rb);
            }

            // Step 2-2(h): 무손실일 때만 채택(성공 +1, 단조 증가 보장).
            if (all_ok) {
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
        // Step 2-3: 이 라운드에서 개선이 없으면 더 이상 rip-up 이 도움이 안 됨 → 종료.
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
