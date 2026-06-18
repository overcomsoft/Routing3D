// ImplicitOccupancy(sparse ?´ë²• S1~S4) ?ŒìŠ¤????Routing3D C++ ?”ì§„ (Phase 3, Step 3.11)
// =============================================================================
// [???Œì¼???˜ëŠ” ??
//   1) ë¶ˆë???O1: ImplicitOccupancy(ë³µì????†ëŠ” AABB ?‰ì¸)ê°€ DenseOccupancy ?€ **?™ì¼??
//      is_blocked/in_bounds** ë¥??´ëŠ”ì§€ ?„ìˆ˜ ë¹„êµ(ê²½ê³„ ?¬í•¨). ë°˜ì—´ë¦?ë³µì????¼ì¹˜(strict overlap).
//   2) 64ë¹„íŠ¸ lin/unlin: 10mmÂ·20???€ ê±°ë? ê²©ì?ì„œ ?¸ë±???¤ë²„?Œë¡œ ?†ì´ ?•ë³µ(S2).
//   3) ê±°ë? ê²©ì ë¡œì»¬ ?¼ìš°?? 130MB/2GB ë°°ì—´ ?†ì´ astar_weighted ê°€ ì§§ì? ê²½ë¡œë¥?ì¦‰ì‹œ ?°ì¶œ(S1/S3).
//   4) ?¨ë””ë§¨ë“œ ?´ë¦¬?´ëŸ°?? clearance_cells ê°€ ê±°ë¦¬(?€)ë¥??í•œ ?´ì—???©ë¦¬?ìœ¼ë¡?ë°˜í™˜(S4).
//
// [ë¹Œë“œ/?¤í–‰]
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

// Dense vs Implicit ?„ìˆ˜ is_blocked/in_bounds ë¹„êµ(ê²½ê³„ ë°???ì¹??¬í•¨).
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
    check(mism == 0, "is_blocked/in_bounds ?„ìˆ˜ ?¼ì¹˜(ë°˜ì—´ë¦?ë³µì????™ì¼)");

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

// 10mmÂ·project1 ê·œëª¨(900x1442x1563 ??20.3???€)?ì„œ ?¤ë²„?Œë¡œ/?€?¥í­ë°??†ì´ ?™ì‘.
static void test_huge_grid() {
    std::printf("=== ê±°ë? ê²©ì 10mm (Implicit) ===\n");
    const double cell = 10.0;
    Cell shape{900, 1442, 1563};
    ImplicitOccupancy occ(shape, Vec3{0, 0, 0}, cell);
    const long long total = static_cast<long long>(shape.i) * shape.j * shape.k;
    check(total > 2000000000LL, "ê²©ì ?€ ??> 20??int ?œê³„ ì´ˆê³¼ ê·œëª¨)");
    check(occ.size() == total, "size() ?•í™•(64ë¹„íŠ¸)");

    // 64ë¹„íŠ¸ lin/unlin ?•ë³µ(?ì  ê·¼ì²˜ + ìµœë? ?€).
    Cell corner{shape.i - 1, shape.j - 1, shape.k - 1};
    long long lc = occ.lin(corner);
    check(lc > 2000000000LL, "ìµœë? ?€ lin > 20???¤ë²„?Œë¡œ ?†ìŒ)");
    Cell back = occ.unlin(lc);
    check(back == corner, "unlin(lin(corner)) == corner");
    Cell mid{123, 1000, 1234};
    check(occ.unlin(occ.lin(mid)) == mid, "unlin(lin(mid)) == mid");

    // ë¡œì»¬ ?¥ì• ë¬?ë²?ê°€?´ë° ?? + ì§§ì? ?°íšŒ ?¼ìš°?? ?„ì—­ ë°°ì—´ ?†ì´ ì¦‰ì‹œ ?±ê³µ?´ì•¼ ?œë‹¤.
    // ë²? x=[2000,2100], y=[0,5000], z=[0,5000] ì¤?y ê°€?´ë°????
    occ.add_box(AABB(Vec3{2000, 0, 0}, Vec3{2100, 2400, 5000}));
    occ.add_box(AABB(Vec3{2000, 2600, 0}, Vec3{2100, 5000, 5000}));  // ?? y??2400,2600]

    RouteParams params;
    params.cell_mm = cell;
    params.w_turn = 500.0;
    params.w_clear = 10.0;       // ?¨ë””ë§¨ë“œ ?´ë¦¬?´ëŸ°???œì„± ???„ì—­ ë°°ì—´ ??ë§Œë“¦(S4).
    params.clearance_radius = 2;

    Cell s = occ.to_cell(Vec3{1500, 2500, 1000});  // ë²??¼ìª½
    Cell g = occ.to_cell(Vec3{2600, 2500, 1000});  // ë²??¤ë¥¸ìª????µê³¼)
    check(!occ.is_blocked(s) && !occ.is_blocked(g), "start/goal are free");

    AStarResult r = astar_weighted(occ, s, g, params, 2000000LL, false);
    std::printf("  route success=%d len=%.0f turns=%d expanded=%lld %.1fms\n", r.success ? 1 : 0,
                r.length_mm, r.turns, r.expanded_nodes, r.elapsed_ms);
    check(r.success, "ê±°ë? ê²©ì ë¡œì»¬ ?°íšŒ ê²½ë¡œ ?±ê³µ(???µê³¼)");
    check(r.path.size() >= 2, "ê²½ë¡œ ?€ 2ê°??´ìƒ");
}

// ?¨ë””ë§¨ë“œ ?´ë¦¬?´ëŸ°??ë°•ìŠ¤ ìµœê·¼?? ?©ë¦¬??
static void test_clearance() {
    std::printf("=== ?¨ë””ë§¨ë“œ ?´ë¦¬?´ëŸ°??===\n");
    ImplicitOccupancy occ(Cell{200, 200, 50}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(AABB(Vec3{0, 0, 0}, Vec3{1000, 1000, 2500}));  // ??ëª¨ì„œë¦???ë¸”ë¡.

    // ë¸”ë¡?ì„œ ë©€ë¦??¨ì–´ì§??€ ???í•œ(max_radius) ë°˜í™˜.
    Cell far = occ.to_cell(Vec3{8000, 8000, 1000});
    check(occ.clearance_cells(far, 3) == 3, "ë¨??€ = ?í•œ 3");
    // ë¸”ë¡ ?œë©´ ë°”ë¡œ ???€ ???‘ì? ê°?0~1).
    Cell near = occ.to_cell(Vec3{1050, 500, 1000});  // x=1050: ?œë©´ x=1000 ?ì„œ 50mm(1?€)
    int dn = occ.clearance_cells(near, 3);
    std::printf("  near clearance = %d cells\n", dn);
    check(dn <= 1, "?œë©´ ?¸ì ‘ ?€ ?´ë¦¬?´ëŸ°??<= 1");
}

int main() {
    test_o1();
    test_huge_grid();
    test_clearance();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
