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
// [2단계 계층 라우팅 흐름]
//   coarse_implicit_from_doc 로 거친 점유맵(factor 배 셀 크기) 구성
//   → route_hier 가 coarse A*(w=1.0, 최적) → fine 튜브 corridor A*
//   → fine 실패 시 corridor 반경 ×2 로 최대 3회 재시도(Stage 2)
//   → 모두 실패하면 전체격자 direct A* 폴백(capi 의 escalation 로직)
//
// [결정성]
//   astar 와 동일한 (f, counter) tie-break + 고정 이웃 순서 → in_corridor 가 전체
//   허용이면 astar 와 **동일 경로/확장수**를 낸다(셀 좌표는 packing 가능 범위, 축<2^20).
//
// [주의(현 한계)]
//   coarse 점유맵은 호출자가 구성한다(보통 동일 장애물을 coarse 해상도로 add_box).
//   보수적(겹치면 점유) 구성은 얇은 통로를 과차단할 수 있어 radius 로 완화한다.
//   클리어런스(distance transform)는 전역 배열이 필요해 초대형에서 비활성(향후 로컬화).
//
// [Stage 3b: 적응형 coarse factor]
//   routing3d_capi.cpp 의 route_hier 호출부에서 factor = round(200 / cell_mm) [4,32] 로
//   고정해 coarse 셀 크기가 해상도 무관하게 ~200mm 를 유지한다.
//   cell=25 → factor=8(기존 동일), cell=10 → factor=20, cell=50 → factor=4.
//   이로써 10mm 초미세격자에서도 coarse 격자 셀 수가 한정돼 coarse 탐색 비용이 안 터진다.
//
// [관련 파일]
//   routing3d/astar.hpp           — astar_weighted (weighted A*, min_straight 상태)
//   routing3d/occupancy.hpp       — DenseOccupancy (Dense 점유맵)
//   routing3d/vdb_occupancy.hpp   — VdbOccupancy (OpenVDB 백엔드)
//   routing3d/box_index.hpp       — ImplicitOccupancy (거대격자 AABB 색인)
//   cpp/capi/routing3d_capi.cpp   — route_hier / coarse_implicit_from_doc 호출부
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

// =============================================================================
// pack20 / unpack20 — Cell 좌표를 64비트 정수 키로 직렬화 (해시맵 키·집합 원소로 사용)
//
// 축당 20비트(i<<40 | j<<20 | k) → 최대 셀 좌표 2^20-1 = 1,048,575.
// 8,000m 장면 / 50mm 셀 = 160,000 < 2^20 → 실제 플랜트 규모에서 충분.
// unpack20 은 경로 역추적 시 came 맵(64비트 키)을 Cell 로 되돌릴 때 사용.
// =============================================================================
inline uint64_t pack20(const Cell& c) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(c.i)) << 40) |
           (static_cast<uint64_t>(static_cast<uint32_t>(c.j)) << 20) |
           static_cast<uint64_t>(static_cast<uint32_t>(c.k));
}
inline Cell unpack20(uint64_t k) {
    return Cell{static_cast<int>((k >> 40) & 0xFFFFF), static_cast<int>((k >> 20) & 0xFFFFF),
                static_cast<int>(k & 0xFFFFF)};
}

// =============================================================================
// CorridorAll — 모든 셀을 허용하는 corridor 술어 (전역 탐색용 기본 술어)
//
// astar_hashed 의 in_corridor 파라미터로 전달하면 corridor 제한 없이 전역 탐색.
// route_corridor 의 Step 1(coarse 탐색)에서 사용 — coarse 격자는 이미 장거리
// 경로 찾기용이므로 별도 corridor 제한 없이 전체를 탐색한다.
// =============================================================================
struct CorridorAll {
    bool operator()(const Cell&) const { return true; }
};

// =============================================================================
// astar_hashed — 해시 기반 균일비용 A* (초대형 격자 전용)
//
// [목적]
//   astar_weighted 는 g/closed 를 occ.size() 크기 고정 배열로 잡아
//   초대형 격자(예: 17.8억 셀 = cell=10mm)에서 메모리 할당 자체가 불가능하다.
//   astar_hashed 는 g/came/closed 를 모두 64비트 키 해시맵으로 잡아
//   **실제 탐색한 셀 수에 비례한 메모리**만 사용한다 → 거대 장면 로컬 배관 즉시 라우팅.
//
// [corridor 제한]
//   in_corridor(cell) == false 인 셀은 탐색에서 제외.
//   route_corridor 의 Step 3(fine 탐색)에서 coarse 튜브 내부만 허용하는 람다를 넘긴다.
//   CorridorAll{} 을 넘기면 제한 없이 전역 탐색(coarse 단계에서 사용).
//
// [균일비용 A*]
//   step_cost < 0 이면 occ.cell_mm() 자동 사용(균일 비용).
//   비용 함수가 균일해 h(c)=맨해튼거리×sc 는 정확히 admissible → 최적 경로 보장.
//   회전 페널티/클리어런스 없음 — coarse 골격 탐색에서는 균일 비용이 충분히 좋은 가이드.
//
// [결정성]
//   open 큐: (f=g+h, counter, cell, dir=-1) → (f, counter) tie-break = astar_weighted 와 동일.
//   counter 는 삽입 순서 단조 증가 → 동일 f 에서 먼저 삽입된 것이 먼저 확장 = 결정적.
//
// [경로 역추적]
//   came[nk] = ck (이웃 셀 키 → 직전 셀 키). 목표 도달 시 역순 링크 따라 경로 복원.
//   마지막에 std::swap 으로 시작→목표 순서로 뒤집는다.
//
// [파라미터]
//   step_cost     : 한 셀 이동 비용(mm). <0 이면 occ.cell_mm() 자동 사용.
//   in_corridor   : 탐색 허용 술어. false 셀은 open 에 추가하지 않음.
//   max_expansions: 탐색 노드 상한(-1=무제한).
// =============================================================================
template <class Occ, class InCorridor>
AStarResult astar_hashed(const Occ& occ, Cell start, Cell goal, double step_cost,
                         InCorridor in_corridor, long long max_expansions = -1) {
    auto t0 = detail::Clock::now();
    AStarResult R;
    const double sc = (step_cost < 0.0) ? occ.cell_mm() : step_cost;

    // 시작/목표가 점유이거나 corridor 밖이면 즉시 실패.
    if (occ.is_blocked(start) || occ.is_blocked(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (!in_corridor(start) || !in_corridor(goal)) { R.elapsed_ms = detail::ms_since(t0); return R; }
    if (start == goal) {
        R.success = true; R.path = {start}; R.expanded_nodes = 1; R.elapsed_ms = detail::ms_since(t0);
        return R;
    }

    // h(c) = 맨해튼 거리 × step_cost = admissible 휴리스틱(균일비용 최적 보장).
    auto h = [&](const Cell& c) { return manhattan(c, goal) * sc; };

    // 해시 기반 탐색 상태 컨테이너(occ.size() 고정 배열 대신).
    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> open;
    std::unordered_map<uint64_t, double> g;       // 셀 키 → 현재 최단 g 값.
    std::unordered_map<uint64_t, uint64_t> came;  // 셀 키 → 직전 셀 키(역추적용).
    std::unordered_set<uint64_t> closed;           // 확장 완료 셀.

    long long counter = 0;  // tie-break 용 삽입 순서 카운터.
    g[pack20(start)] = 0.0;
    open.push({h(start), counter++, start, -1});
    long long expanded = 0;

    while (!open.empty()) {
        detail::PQItem cur = open.top();
        open.pop();
        uint64_t ck = pack20(cur.cell);
        if (closed.count(ck)) continue;  // 이미 확장된 셀(지연 삭제 방식).
        closed.insert(ck);
        ++expanded;

        // 목표 도달 → 경로 역추적 후 반환.
        if (cur.cell == goal) {
            std::vector<Cell> path{goal};
            uint64_t k = ck;
            auto it = came.find(k);
            while (it != came.end()) {
                k = it->second;
                path.push_back(unpack20(k));
                it = came.find(k);
            }
            // came 을 역순으로 따라오면 목표→시작 순서 → reverse 로 뒤집기.
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
        // 탐색 상한 초과 → 실패 반환(corridor 바운드라 메모리 안전).
        if (max_expansions > 0 && expanded >= max_expansions) break;

        double g_cur = g[ck];
        for (const Cell& d : NEIGHBORS_6) {
            Cell nb{cur.cell.i + d.i, cur.cell.j + d.j, cur.cell.k + d.k};
            if (!occ.in_bounds(nb) || occ.is_blocked(nb)) continue;
            if (!in_corridor(nb)) continue;  // corridor 술어 필터.
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

// =============================================================================
// CorridorRoute — 계층 corridor 라우팅 결과 묶음
//
//   fine           : 최종 fine 해상도 경로(성공/길이/회전/확장수). 호출자가 실제로 쓰는 결과.
//   coarse_path    : coarse 가이드 경로(coarse 셀 단위). 진단·시각화용.
//   coarse_success : coarse 단계 성공 여부. false 이면 fine 탐색을 건너뜀.
//   corridor_cells : 팽창된 coarse corridor 셀 집합 크기(진단/로그용).
// =============================================================================
struct CorridorRoute {
    AStarResult fine;                 // 최종 fine 경로(성공/길이/회전/확장수).
    std::vector<Cell> coarse_path;    // coarse 가이드 경로(coarse 셀).
    bool coarse_success = false;      // coarse 단계 성공 여부.
    long long corridor_cells = 0;     // corridor(팽창된 coarse 셀) 개수.
};

// =============================================================================
// route_corridor — 2단계 계층 corridor 라우팅 (단일 배관, route_hier 의 내부 구현)
//
// [알고리즘 개요]
//   초대형 격자에서 단일 배관을 직접 A* 로 탐색하면 수억 셀을 전부 검사해 느리거나
//   탐색 상한에 걸린다. 이를 해결하기 위해 두 단계로 나눈다:
//   Step 1(coarse) → Step 2(fine 튜브) → 재시도(Stage 2, 호출자 route_hier 에서).
//
// [Step 1: coarse 가이드 경로 (Stage 1 품질 개선)]
//   astar_hashed(coarse, CorridorAll, w=1.0 균일비용) 로 coarse 점유맵 전역 탐색.
//   coarse 격자는 fine 의 factor³ 배 작은 셀 수(예: factor=8 → 512배 작음)이므로
//   **최적(w=1.0) 탐색도 저렴**하다 — 이전 w_heur=2.0(greedy)은 우회 골격을 만들어
//   fine 튜브를 좁은 통로에 가두는 문제가 있었다(Stage 1 수정, 199s→127s, 1.57×↑).
//   coarse 실패(장애물 전체 차단) → fine 건너뛰고 CorridorRoute.coarse_success=false 반환.
//
// [Step 2: fine 튜브 A*]
//   coarse 경로 각 셀을 Chebyshev 반경 radius 로 팽창해 corridor 키 집합 구성.
//   astar_hashed(fine, in_corr) 로 fine 점유맵에서 corridor 내부만 탐색.
//   → 전체 fine 격자가 아닌 'coarse 경로 ±radius' 튜브만 탐색 → 탐색량 크게 감소.
//
//   fine 실패 시 처리(Stage 2 재시도)는 이 함수가 아닌 호출자(route_hier, capi)에서
//   radius ×2 로 최대 3회 호출하는 방식으로 구현된다.
//   → 재시도 성공 경로는 첫 시도와 달리 넓은 튜브를 사용하지만,
//     첫 시도 성공이면 기존 결과와 동일(무회귀).
//
// [사전 조건]
//   fine.cell_mm() × factor ≈ coarse.cell_mm() (정수 배율).
//   fine, coarse 의 origin(좌하단 좌표) 이 동일해야 to_coarse 변환이 정확함.
//   셀 좌표 i,j,k >= 0 (pack20 의 uint32_t 캐스트가 음수를 처리 못함).
//
// [파라미터]
//   fine           : fine 해상도 점유맵(실제 라우팅 해상도).
//   coarse         : coarse 해상도 점유맵(factor 배 셀 크기, 동일 장애물).
//   start_fine     : fine 격자 시작 셀.
//   goal_fine      : fine 격자 목표 셀.
//   factor         : coarse/fine 셀 크기 비율(정수).
//                    Stage 3b: routing3d_capi.cpp 에서 round(200/cell_mm) [4,32] 로 자동 결정.
//   radius         : corridor 팽창 반경(coarse 셀 단위, Chebyshev 거리).
//                    Stage 2 재시도 시 ×2 씩 확장(최대 3회).
//   max_expansions : astar_hashed 의 탐색 노드 상한.
// =============================================================================
template <class FineOcc, class CoarseOcc>
CorridorRoute route_corridor(const FineOcc& fine, const CoarseOcc& coarse, Cell start_fine,
                             Cell goal_fine, int factor, int radius,
                             long long max_expansions = -1) {
    CorridorRoute R;
    // to_coarse: fine 셀 → 대응하는 coarse 셀(바닥 나눗셈, i>=0 가정).
    auto to_coarse = [&](const Cell& c) {
        return Cell{c.i / factor, c.j / factor, c.k / factor};
    };

    // Step 1: coarse 가이드 경로 탐색.
    //   CorridorAll = 전역 허용, step_cost = coarse.cell_mm(), w=1.0(균일·최적).
    //   coarse 格子는 fine 의 factor³ 배 작아 최적 탐색도 저렴 — greedy 대신 최적 경로.
    AStarResult cres =
        astar_hashed(coarse, to_coarse(start_fine), to_coarse(goal_fine), coarse.cell_mm(),
                     CorridorAll{}, max_expansions);
    R.coarse_success = cres.success;
    R.coarse_path = cres.path;
    if (!cres.success) return R;  // coarse 실패 → 장애물 전체 차단, fine 불필요.

    // Step 2-a: coarse 경로를 Chebyshev 반경 radius 로 팽창 → corridor 집합.
    //   각 coarse 셀의 [-radius, +radius]³ 이웃을 모두 포함(Chebyshev 박스 팽창).
    std::unordered_set<uint64_t> corr;
    for (const Cell& c : cres.path)
        for (int di = -radius; di <= radius; ++di)
            for (int dj = -radius; dj <= radius; ++dj)
                for (int dk = -radius; dk <= radius; ++dk)
                    corr.insert(pack20(Cell{c.i + di, c.j + dj, c.k + dk}));
    R.corridor_cells = static_cast<long long>(corr.size());

    // Step 2-b: fine A* — fine 셀의 대응 coarse 셀이 corridor 집합에 있을 때만 탐색.
    //   in_corr 술어: fc(fine셀) → to_coarse(fc) 가 corr 에 있으면 true.
    auto in_corr = [&](const Cell& fc) { return corr.count(pack20(to_coarse(fc))) > 0; };
    R.fine = astar_hashed(fine, start_fine, goal_fine, fine.cell_mm(), in_corr, max_expansions);
    // fine 실패 시: 호출자(route_hier)가 radius ×2 로 재호출해 Stage 2 재시도 수행.
    return R;
}

}  // namespace routing3d
