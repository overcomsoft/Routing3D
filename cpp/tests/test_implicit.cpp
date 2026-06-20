// Tests for ImplicitOccupancy sparse occupancy behavior.
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
    check(mism == 0, "is_blocked/in_bounds ?수 ?치(반열?복????일)");

    std::vector<Cell> blocked = impl.blocked_cells();
    check(static_cast<long long>(blocked.size()) == dense.count_blocked(),
          "blocked_cells count matches dense voxelization");
    long long bad = 0;
    for (const Cell& c : blocked)
        if (!dense.is_blocked(c)) ++bad;
    check(bad == 0, "blocked_cells entries are dense-blocked cells");
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

static void test_huge_grid() {
    std::printf("=== 거? 격자 10mm (Implicit) ===\n");
    const double cell = 10.0;
    Cell shape{900, 1442, 1563};
    ImplicitOccupancy occ(shape, Vec3{0, 0, 0}, cell);
    const long long total = static_cast<long long>(shape.i) * shape.j * shape.k;
    check(total > 2000000000LL, "격자 ? ??> 20??int ?계 초과 규모)");
    check(occ.size() == total, "size() ?확(64비트)");

    Cell corner{shape.i - 1, shape.j - 1, shape.k - 1};
    long long lc = occ.lin(corner);
    check(lc > 2000000000LL, "최? ? lin > 20???버?로 ?음)");
    Cell back = occ.unlin(lc);
    check(back == corner, "unlin(lin(corner)) == corner");
    Cell mid{123, 1000, 1234};
    check(occ.unlin(occ.lin(mid)) == mid, "unlin(lin(mid)) == mid");

    occ.add_box(AABB(Vec3{2000, 0, 0}, Vec3{2100, 2400, 5000}));
    occ.add_box(AABB(Vec3{2000, 2600, 0}, Vec3{2100, 5000, 5000}));  // ?? y??2400,2600]

    RouteParams params;
    params.cell_mm = cell;
    params.w_turn = 500.0;
    params.w_clear = 10.0;
    params.clearance_radius = 2;

    Cell s = occ.to_cell(Vec3{1500, 2500, 1000});
    Cell g = occ.to_cell(Vec3{2600, 2500, 1000});
    check(!occ.is_blocked(s) && !occ.is_blocked(g), "start/goal are free");

    AStarResult r = astar_weighted(occ, s, g, params, 2000000LL, false);
    std::printf("  route success=%d len=%.0f turns=%d expanded=%lld %.1fms\n", r.success ? 1 : 0,
                r.length_mm, r.turns, r.expanded_nodes, r.elapsed_ms);
    check(r.success, "거? 격자 로컬 ?회 경로 ?공(???과)");
    check(r.path.size() >= 2, "경로 ? 2??상");
}

static void test_clearance() {
    std::printf("=== ?디맨드 ?리?런??===\n");
    ImplicitOccupancy occ(Cell{200, 200, 50}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(AABB(Vec3{0, 0, 0}, Vec3{1000, 1000, 2500}));

    Cell far = occ.to_cell(Vec3{8000, 8000, 1000});
    check(occ.clearance_cells(far, 3) == 3, "?? = ?한 3");
    Cell near = occ.to_cell(Vec3{1050, 500, 1000});
    int dn = occ.clearance_cells(near, 3);
    std::printf("  near clearance = %d cells\n", dn);
    check(dn <= 1, "?면 ?접 ? ?리?런??<= 1");
}

int main() {
    test_o1();
    test_huge_grid();
    test_clearance();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
