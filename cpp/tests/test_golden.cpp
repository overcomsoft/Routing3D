// 골든셋 재현 테스트 — Routing3D C++ 엔진 (Phase 3)
// =============================================================================
// [이 파일이 하는 일]
//   Python 회귀 골든셋(docs/spec/regression_set.md)의 01_single_empty / 02_single_obstacle
//   를 C++ 비용함수 A* 로 재현하고 기대지표(길이/회전/장애물비통과/우회비율/확장상한)를
//   검증한다. Python 과 동일 tie-break 라 길이·회전이 정확히 일치해야 한다.
//   (다중 배관 03_multi_tier 는 multi_route 구현 후 추가 — Step 3.5.)
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
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

int main() {
    scenario_01_single_empty();
    scenario_02_single_obstacle();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
