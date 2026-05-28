// 골든셋 재현 테스트 — Routing3D C++ 엔진 (Phase 3)
// =============================================================================
// [이 파일이 하는 일]
//   Python 회귀 골든셋(docs/spec/regression_set.md)의 01_single_empty / 02_single_obstacle /
//   03_multi_tier 를 C++ 엔진으로 재현하고 기대지표(길이/회전/장애물비통과/우회비율/
//   확장상한/충돌수)를 검증한다. Python 과 동일 tie-break 라 길이·회전이 정확히 일치해야 한다.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>
#include <unordered_set>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;

static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

static int path_hits_obstacle(const DenseOccupancy& occ, const AStarResult& r) {
    int hits = 0;
    for (const Cell& c : r.path)
        if (occ.is_blocked(c)) ++hits;
    return hits;
}

static RouteParams baseline() {
    RouteParams p;  // cell_mm 50, w_turn 500, w_clear 10, clearance_radius 2, conn 6
    return p;
}

// ---- 백엔드 교차검증 헬퍼 (Step 3.6: A* 가 백엔드 무관임을 확인) ----
// 임의 점유 백엔드에 동일 장면을 구성해 단일 비용함수 A* 를 실행.
template <class Occ>
static AStarResult run_single(Cell shape, Vec3 origin, double cell, const std::vector<AABB>& boxes,
                              Vec3 s_mm, Vec3 g_mm) {
    Occ occ(shape, origin, cell);
    for (const AABB& b : boxes) occ.add_box(b);
    return astar_weighted(occ, occ.to_cell(s_mm), occ.to_cell(g_mm), baseline());
}

// 두 AStarResult 가 지표 + 경로 셀까지 완전히 같은지.
static bool same_result(const AStarResult& a, const AStarResult& b) {
    return a.success == b.success && a.length_mm == b.length_mm && a.turns == b.turns &&
           a.expanded_nodes == b.expanded_nodes && a.cost_mm == b.cost_mm && a.path == b.path;
}

static void scenario_01_single_empty() {
    std::printf("=== 01_single_empty ===\n");
    DenseOccupancy occ(Cell{20, 20, 20}, Vec3{0, 0, 0}, 50.0);
    Cell s = occ.to_cell(Vec3{25, 25, 25});      // (0,0,0)
    Cell g = occ.to_cell(Vec3{975, 975, 975});   // (19,19,19)
    AStarResult r = astar_weighted(occ, s, g, baseline());
    std::printf("  length=%.1f turns=%d expanded=%lld cost=%.1f (%.2f ms)\n",
                r.length_mm, r.turns, r.expanded_nodes, r.cost_mm, r.elapsed_ms);
    check(r.success, "success");
    check(r.length_mm == 2850.0, "length_mm == 2850");
    check(r.turns == 2, "turns == 2");
    check(path_hits_obstacle(occ, r) == 0, "path_hits_obstacle == 0");
    check(r.expanded_nodes <= 30000, "expanded_nodes <= 30000");
}

static void scenario_02_single_obstacle() {
    std::printf("=== 02_single_obstacle ===\n");
    DenseOccupancy occ(Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(AABB(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000}));
    Cell s = occ.to_cell(Vec3{275, 2025, 2025});
    Cell g = occ.to_cell(Vec3{3725, 2025, 2025});
    AStarResult r = astar_weighted(occ, s, g, baseline());
    double manhattan_mm = manhattan(s, g) * 50.0;
    std::printf("  length=%.1f (manhattan=%.1f, ratio=%.3f) turns=%d expanded=%lld (%.2f ms)\n",
                r.length_mm, manhattan_mm, r.length_mm / manhattan_mm, r.turns,
                r.expanded_nodes, r.elapsed_ms);
    check(r.success, "success");
    check(r.length_mm == 3950.0, "length_mm == 3950");
    check(r.turns == 2, "turns == 2");
    check(path_hits_obstacle(occ, r) == 0, "path_hits_obstacle == 0");
    check(r.length_mm <= manhattan_mm * 1.2 + 1e-6, "detour ratio <= 1.2");
    check(r.expanded_nodes <= 12000, "expanded_nodes <= 12000");
}

// 성공 경로들의 쌍별 셀 공유 횟수(충돌 수). scenario_runner.py 의 collisions 계산과 동일. 백엔드 무관.
template <class Occ>
static int count_collisions(const Occ& occ, const MultiRouteResult<Occ>& mr) {
    std::vector<std::unordered_set<int>> sets;  // 성공 배관별 점유 셀(선형 인덱스) 집합.
    for (const PipeResult& p : mr.pipes) {
        if (!p.result.success) continue;
        std::unordered_set<int> s;
        for (const Cell& c : p.result.path) s.insert(occ.lin(c));
        sets.push_back(std::move(s));
    }
    int collisions = 0;
    for (size_t i = 0; i < sets.size(); ++i)
        for (size_t j = i + 1; j < sets.size(); ++j) {
            bool shared = false;
            const auto& small = (sets[i].size() <= sets[j].size()) ? sets[i] : sets[j];
            const auto& big = (sets[i].size() <= sets[j].size()) ? sets[j] : sets[i];
            for (int v : small)
                if (big.count(v)) { shared = true; break; }
            if (shared) ++collisions;
        }
    return collisions;
}

// 골든03 의 5개 배관 작업(같은 통로 start/end 동일).
static std::vector<RouteTask> golden03_tasks() {
    RouteTask t;
    t.start_mm = Vec3{275, 3025, 1525};
    t.end_mm = Vec3{5725, 3025, 1525};
    std::vector<RouteTask> tasks;
    const char* utils[5][2] = {
        {"UPW_S", "UPW"}, {"NFW", "Waste Liquid"}, {"PA", "Gas"},
        {"NW", "Water"}, {"ACID", "Exhaust"},
    };
    for (auto& u : utils) {
        t.utility = u[0];
        t.utility_group = u[1];
        tasks.push_back(t);
    }
    return tasks;
}

static void scenario_03_multi_tier() {
    std::printf("=== 03_multi_tier ===\n");
    // shape 120x120x60, 셀 50mm. 바닥 슬래브(z 0~250mm = 셀 0~4) 장애물.
    DenseOccupancy occ(Cell{120, 120, 60}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250}));

    auto mr = route_sequential(occ, golden03_tasks(), baseline(), "longest");
    int collisions = count_collisions(occ, mr);
    std::printf("  success=%d/%zu fail=%d total_length=%.1f collisions=%d\n",
                mr.success_count(), mr.pipes.size(), mr.fail_count(),
                mr.total_length_mm(), collisions);
    check(mr.success_count() == 5, "success_count == 5");
    check(mr.fail_count() == 0, "fail_count == 0");
    check(mr.success_rate() == 1.0, "success_rate == 1.0");
    check(collisions == 0, "collisions == 0");
    check(mr.total_length_mm() == 28050.0, "total_length_mm == 28050");
}

// 백엔드 무관 검증: 동일 골든 입력을 DenseOccupancy 와 SparseOccupancy 로 라우팅하면
// 지표·경로가 완전히 같아야 한다(엔진이 백엔드 무관 = O1 + 결정성 보존).
static void cross_backend_sparse() {
    std::printf("=== 백엔드 교차검증: Dense vs Sparse ===\n");
    // 골든01(빈 공간).
    {
        AStarResult d = run_single<DenseOccupancy>(Cell{20, 20, 20}, Vec3{0, 0, 0}, 50.0, {},
                                                   Vec3{25, 25, 25}, Vec3{975, 975, 975});
        AStarResult s = run_single<SparseOccupancy>(Cell{20, 20, 20}, Vec3{0, 0, 0}, 50.0, {},
                                                    Vec3{25, 25, 25}, Vec3{975, 975, 975});
        check(same_result(d, s), "golden01 Dense==Sparse (지표+경로)");
    }
    // 골든02(장애물 우회).
    {
        std::vector<AABB> boxes{AABB(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000})};
        AStarResult d = run_single<DenseOccupancy>(Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0, boxes,
                                                   Vec3{275, 2025, 2025}, Vec3{3725, 2025, 2025});
        AStarResult s = run_single<SparseOccupancy>(Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0, boxes,
                                                    Vec3{275, 2025, 2025}, Vec3{3725, 2025, 2025});
        check(same_result(d, s), "golden02 Dense==Sparse (지표+경로)");
    }
    // 골든03(다중 순차).
    {
        DenseOccupancy docc(Cell{120, 120, 60}, Vec3{0, 0, 0}, 50.0);
        docc.add_box(AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250}));
        SparseOccupancy socc(Cell{120, 120, 60}, Vec3{0, 0, 0}, 50.0);
        socc.add_box(AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250}));
        auto dmr = route_sequential(docc, golden03_tasks(), baseline(), "longest");
        auto smr = route_sequential(socc, golden03_tasks(), baseline(), "longest");
        bool eq = dmr.success_count() == smr.success_count() &&
                  dmr.total_length_mm() == smr.total_length_mm() &&
                  count_collisions(docc, dmr) == count_collisions(socc, smr);
        check(eq && smr.success_count() == 5 && count_collisions(socc, smr) == 0,
              "golden03 Dense==Sparse (성공수/총길이/충돌)");
    }
}

int main() {
    scenario_01_single_empty();
    scenario_02_single_obstacle();
    scenario_03_multi_tier();
    cross_backend_sparse();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
