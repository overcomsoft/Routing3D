// ImplicitOccupancy(sparse 해법 S1~S4) 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.11)
// =============================================================================
// [이 파일이 하는 일]
//   1) 불변식 O1: ImplicitOccupancy(복셀화 없는 AABB 색인)가 DenseOccupancy 와 **동일한
//      is_blocked/in_bounds** 를 내는지 전수 비교(경계 포함). 반열린 복셀화 일치(strict overlap).
//   2) 64비트 lin/unlin: 10mm·20억 셀 거대 격자에서 인덱스 오버플로 없이 왕복(S2).
//   3) 거대 격자 로컬 라우팅: 130MB/2GB 배열 없이 astar_weighted 가 짧은 경로를 즉시 산출(S1/S3).
//   4) 온디맨드 클리어런스: clearance_cells 가 거리(셀)를 상한 내에서 합리적으로 반환(S4).
//
// [빌드/실행]
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R implicit --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;
static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

// Dense vs Implicit 전수 is_blocked/in_bounds 비교(경계 밖 한 칸 포함).
static void o1_scene(const std::string& name, Cell shape, Vec3 origin, double cell_mm,
                     const std::vector<AABB>& boxes) {
    std::printf("=== O1 Dense==Implicit: %s (shape %d,%d,%d) ===\n", name.c_str(), shape.i, shape.j,
                shape.k);
    DenseOccupancy dense(shape, origin, cell_mm);
    ImplicitOccupancy impl(shape, origin, cell_mm);
    for (const AABB& b : boxes) { dense.add_box(b); impl.add_box(b); }

    long long mism = 0;
    for (int k = -1; k <= shape.k; ++k)
        for (int j = -1; j <= shape.j; ++j)
            for (int i = -1; i <= shape.i; ++i) {
                Cell c{i, j, k};
                if (dense.is_blocked(c) != impl.is_blocked(c)) ++mism;
                if (dense.in_bounds(c) != impl.in_bounds(c)) ++mism;
            }
    check(mism == 0, "is_blocked/in_bounds 전수 일치(반열린 복셀화 동일)");
}

static void test_o1() {
    o1_scene("single_obstacle", Cell{80, 80, 80}, Vec3{0, 0, 0}, 50.0,
             {AABB(Vec3{1900, 0, 0}, Vec3{2150, 2250, 4000})});
    o1_scene("multi_tier", Cell{120, 120, 60}, Vec3{0, 0, 0}, 50.0,
             {AABB(Vec3{0, 0, 0}, Vec3{6000, 6000, 250})});
    o1_scene("multi_box_frac_origin", Cell{40, 40, 40}, Vec3{-125.5, 33.0, 7.25}, 50.0,
             {AABB(Vec3{0, 0, 0}, Vec3{500, 500, 500}),
              AABB(Vec3{900, 200, 100}, Vec3{1100, 1900, 300}),
              AABB(Vec3{-50, -50, -50}, Vec3{120, 120, 120})});
}

// 10mm·project1 규모(900x1442x1563 ≈ 20.3억 셀)에서 오버플로/저장폭발 없이 동작.
static void test_huge_grid() {
    std::printf("=== 거대 격자 10mm (Implicit) ===\n");
    const double cell = 10.0;
    Cell shape{900, 1442, 1563};
    ImplicitOccupancy occ(shape, Vec3{0, 0, 0}, cell);
    const long long total = static_cast<long long>(shape.i) * shape.j * shape.k;
    check(total > 2000000000LL, "격자 셀 수 > 20억(int 한계 초과 규모)");
    check(occ.size() == total, "size() 정확(64비트)");

    // 64비트 lin/unlin 왕복(원점 근처 + 최대 셀).
    Cell corner{shape.i - 1, shape.j - 1, shape.k - 1};
    long long lc = occ.lin(corner);
    check(lc > 2000000000LL, "최대 셀 lin > 20억(오버플로 없음)");
    Cell back = occ.unlin(lc);
    check(back == corner, "unlin(lin(corner)) == corner");
    Cell mid{123, 1000, 1234};
    check(occ.unlin(occ.lin(mid)) == mid, "unlin(lin(mid)) == mid");

    // 로컬 장애물 벽(가운데 틈) + 짧은 우회 라우팅. 전역 배열 없이 즉시 성공해야 한다.
    // 벽: x=[2000,2100], y=[0,5000], z=[0,5000] 중 y 가운데에 틈.
    occ.add_box(AABB(Vec3{2000, 0, 0}, Vec3{2100, 2400, 5000}));
    occ.add_box(AABB(Vec3{2000, 2600, 0}, Vec3{2100, 5000, 5000}));  // 틈: y∈[2400,2600]

    RouteParams params;
    params.cell_mm = cell;
    params.w_turn = 500.0;
    params.w_clear = 10.0;       // 온디맨드 클리어런스 활성 — 전역 배열 안 만듦(S4).
    params.clearance_radius = 2;

    Cell s = occ.to_cell(Vec3{1500, 2500, 1000});  // 벽 왼쪽
    Cell g = occ.to_cell(Vec3{2600, 2500, 1000});  // 벽 오른쪽(틈 통과)
    check(!occ.is_blocked(s) && !occ.is_blocked(g), "시작/끝 셀 비점유");

    AStarResult r = astar_weighted(occ, s, g, params, 2000000LL, false);
    std::printf("  route success=%d len=%.0f turns=%d expanded=%lld %.1fms\n", r.success ? 1 : 0,
                r.length_mm, r.turns, r.expanded_nodes, r.elapsed_ms);
    check(r.success, "거대 격자 로컬 우회 경로 성공(틈 통과)");
    check(r.path.size() >= 2, "경로 셀 2개 이상");
}

// 온디맨드 클리어런스(박스 최근접) 합리성.
static void test_clearance() {
    std::printf("=== 온디맨드 클리어런스 ===\n");
    ImplicitOccupancy occ(Cell{200, 200, 50}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(AABB(Vec3{0, 0, 0}, Vec3{1000, 1000, 2500}));  // 한 모서리 큰 블록.

    // 블록에서 멀리 떨어진 셀 → 상한(max_radius) 반환.
    Cell far = occ.to_cell(Vec3{8000, 8000, 1000});
    check(occ.clearance_cells(far, 3) == 3, "먼 셀 = 상한 3");
    // 블록 표면 바로 옆 셀 → 작은 값(0~1).
    Cell near = occ.to_cell(Vec3{1050, 500, 1000});  // x=1050: 표면 x=1000 에서 50mm(1셀)
    int dn = occ.clearance_cells(near, 3);
    std::printf("  near clearance = %d cells\n", dn);
    check(dn <= 1, "표면 인접 셀 클리어런스 <= 1");
}

int main() {
    test_o1();
    test_huge_grid();
    test_clearance();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
