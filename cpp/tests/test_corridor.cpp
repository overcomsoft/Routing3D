// 계층 corridor / 해시 A* 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.6 part 4)
// =============================================================================
// [이 파일이 하는 일]
//   1) astar_hashed(균일, corridor 전체허용)가 배열기반 astar 와 **동일 경로/확장수**인지.
//   2) 8,000m 초대형 장면(160000^3)에서 '로컬 배관'을 astar_hashed 로 빠르게(<1s) 라우팅
//      (배열기반 astar 는 closed=occ.size() 할당이 불가 → 해시 A* 만 가능).
//   3) route_corridor: coarse 가이드 + fine corridor 로 장애물 우회 경로를 찾고, 그 길이가
//      전역 최단(균일)과 같으며 탐색이 tube 로 제한됨을 확인.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R corridor --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/corridor.hpp"
#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;

static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

// 경로 유효성: 시작/끝 일치, 모든 셀 비점유, 연속 셀은 6-이웃(맨해튼 1).
template <class Occ>
static bool valid_path(const Occ& occ, const std::vector<Cell>& p, Cell s, Cell g) {
    if (p.empty() || !(p.front() == s) || !(p.back() == g)) return false;
    for (size_t i = 0; i < p.size(); ++i) {
        if (occ.is_blocked(p[i])) return false;
        if (i && manhattan(p[i], p[i - 1]) != 1) return false;
    }
    return true;
}

// (1) 해시 A* == 배열 A* (균일).
static void test_hashed_equals_array() {
    std::printf("=== astar_hashed == astar (균일) ===\n");
    // 골든01(빈 공간).
    {
        DenseOccupancy occ(Cell{20, 20, 20}, Vec3{0, 0, 0}, 50.0);
        Cell s = occ.to_cell(Vec3{25, 25, 25}), g = occ.to_cell(Vec3{975, 975, 975});
        AStarResult a = astar(occ, s, g);
        AStarResult hth = astar_hashed(occ, s, g, occ.cell_mm(), CorridorAll{});
        check(a.success == hth.success && a.length_mm == hth.length_mm && a.turns == hth.turns &&
                  a.expanded_nodes == hth.expanded_nodes && a.path == hth.path,
              "golden01 동일(길이/회전/확장수/경로)");
    }
    // 골든02(장애물 우회).
    {
        DenseOccupancy occ(Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0);
        occ.add_box(AABB(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000}));
        Cell s = occ.to_cell(Vec3{275, 2025, 2025}), g = occ.to_cell(Vec3{3725, 2025, 2025});
        AStarResult a = astar(occ, s, g);
        AStarResult hth = astar_hashed(occ, s, g, occ.cell_mm(), CorridorAll{});
        check(a.success == hth.success && a.length_mm == hth.length_mm && a.turns == hth.turns &&
                  a.expanded_nodes == hth.expanded_nodes && a.path == hth.path,
              "golden02 동일(길이/회전/확장수/경로)");
    }
}

// (2) 8,000m 장면의 로컬 배관: 해시 A* 로 빠르게 라우팅(배열 A* 는 할당 불가).
static void test_local_pipe_huge_scene() {
    std::printf("=== 8,000m 장면의 로컬 배관 (astar_hashed) ===\n");
    const int N = 160000;  // 8,000m / 50mm. 배열 closed=occ.size()=4e15 → 불가.
    SparseOccupancy occ(Cell{N, N, N}, Vec3{0, 0, 0}, 50.0);
    // 경로 근처에 장애물 박스 몇 개(우회 유발).
    occ.add_box(AABB(Vec3{1000000 + 2500, 1000000 + 0, 1000000 + 0},
                     Vec3{1000000 + 2550, 1000000 + 3000, 1000000 + 3000}));
    // start/goal: 월드 1,000,000mm 부근, 약 120셀 떨어짐.
    Cell s = occ.to_cell(Vec3{1000000 + 275, 1000000 + 1525, 1000000 + 1525});
    Cell g = occ.to_cell(Vec3{1000000 + 6275, 1000000 + 1525, 1000000 + 1525});
    AStarResult r = astar_hashed(occ, s, g, occ.cell_mm(), CorridorAll{});
    std::printf("  성공=%d 길이=%.0fmm 확장=%lld (%.1f ms), 맨해튼=%d셀\n", r.success, r.length_mm,
                r.expanded_nodes, r.elapsed_ms, manhattan(s, g));
    check(r.success, "로컬 배관 라우팅 성공");
    check(valid_path(occ, r.path, s, g), "경로 유효(비점유/연속)");
    check(r.elapsed_ms < 1000.0, "단일 배관 < 1초");
}

// (3) route_corridor: 벽 우회를 coarse 가이드 + fine corridor 로.
static void test_route_corridor() {
    std::printf("=== route_corridor (coarse 가이드 + fine tube) ===\n");
    const int factor = 8;
    // fine: 200x200x4 (10m x 10m x 0.2m), cell 50mm. 벽(x=5000~5050) 이 y<8000 을 막음(위로 우회).
    DenseOccupancy fine(Cell{200, 200, 4}, Vec3{0, 0, 0}, 50.0);
    fine.add_box(AABB(Vec3{5000, 0, 0}, Vec3{5050, 8000, 200}));
    // coarse: 동일 장애물을 coarse 해상도(400mm)로. shape = ceil(fine/factor).
    DenseOccupancy coarse(Cell{25, 25, 1}, Vec3{0, 0, 0}, 50.0 * factor);
    coarse.add_box(AABB(Vec3{5000, 0, 0}, Vec3{5050, 8000, 200}));

    Cell s = fine.to_cell(Vec3{500, 5000, 100});    // (10,100,2)
    Cell g = fine.to_cell(Vec3{9500, 5000, 100});   // (190,100,2)

    // 전역(균일) 최단 — 비교 기준.
    AStarResult global = astar_hashed(fine, s, g, fine.cell_mm(), CorridorAll{});
    // 계층 corridor.
    CorridorRoute cr = route_corridor(fine, coarse, s, g, factor, /*radius=*/2);

    std::printf("  coarse성공=%d corridor셀=%lld | fine성공=%d 길이=%.0f 확장=%lld | 전역길이=%.0f 확장=%lld\n",
                cr.coarse_success, cr.corridor_cells, cr.fine.success, cr.fine.length_mm,
                cr.fine.expanded_nodes, global.length_mm, global.expanded_nodes);
    check(cr.coarse_success, "coarse 가이드 성공");
    check(cr.fine.success, "fine corridor 경로 성공");
    check(valid_path(fine, cr.fine.path, s, g), "corridor 경로 유효(벽 비통과/연속)");
    check(cr.fine.length_mm == global.length_mm, "corridor 길이 == 전역 최단(균일)");
    check(cr.fine.expanded_nodes <= global.expanded_nodes, "corridor 탐색 <= 전역(tube 제한)");
}

int main() {
    test_hashed_equals_array();
    test_local_pipe_huge_scene();
    test_route_corridor();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
