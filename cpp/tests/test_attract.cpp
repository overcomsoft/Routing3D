// 회랑(corridor) 인력 + 랙(rack) 레벨 테스트 — Python tests/test_corridor.py 와 1:1.
//   1) cell_penalty: 회랑 밖 가산 / 회랑·랙 면제 / w_corridor=0 비활성.
//   2) route_sequential: w_corridor 켜면 둘째 배관이 첫 배관 곁으로 뭉치고(번들링) 총길이 증가.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake --build cpp/build --config Release --target test_attract
//   ctest --test-dir cpp/build -C Release -R attract --output-on-failure
#include <cstdio>
#include <string>
#include <unordered_set>
#include <vector>

#include "routing3d/cost.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;
static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

int main() {
    std::printf("=== 회랑 인력 + 랙 레벨 ===\n");
    DenseOccupancy occ(Cell{30, 30, 1}, Vec3{0, 0, 0}, 100.0);

    // --- 1) cell_penalty 회랑/랙 로직 ---
    {  // 회랑 밖 가산, 회랑 안 면제
        RouteParams p; p.cell_mm = 100.0; p.w_turn = 0.0; p.w_clear = 0.0; p.w_corridor = 100.0;
        std::unordered_set<int> corr; corr.insert(occ.lin(Cell{5, 5, 0}));
        CostModel<DenseOccupancy> cm(occ, p, &corr);
        check(cm.cell_penalty(Cell{5, 5, 0}) == 0.0, "회랑 안 셀 면제");
        check(cm.cell_penalty(Cell{1, 1, 0}) == 100.0, "회랑 밖 셀 가산");
    }
    {  // rack_levels 면제(회랑 없어도 z=0 면제)
        RouteParams p; p.cell_mm = 100.0; p.w_turn = 0.0; p.w_clear = 0.0; p.w_corridor = 100.0;
        p.rack_levels = {0};
        CostModel<DenseOccupancy> cm(occ, p, nullptr);
        check(cm.cell_penalty(Cell{9, 9, 0}) == 0.0, "rack 레벨(z=0) 면제");
    }
    {  // w_corridor=0 비활성
        RouteParams p; p.cell_mm = 100.0; p.w_turn = 0.0; p.w_clear = 0.0; p.w_corridor = 0.0;
        std::unordered_set<int> corr; corr.insert(occ.lin(Cell{5, 5, 0}));
        CostModel<DenseOccupancy> cm(occ, p, &corr);
        check(cm.cell_penalty(Cell{1, 1, 0}) == 0.0, "w_corridor=0 비활성");
    }

    // --- 2) route_sequential 번들링 + 길이 증가 ---
    // A: y=500(row5), B: y=1500(row15) 평행 수평 배관(10셀 떨어짐).
    RouteTask a; a.start_mm = Vec3{100, 500, 50};  a.end_mm = Vec3{2800, 500, 50};  a.utility = "u";
    RouteTask b; b.start_mm = Vec3{100, 1500, 50}; b.end_mm = Vec3{2800, 1500, 50}; b.utility = "u";
    std::vector<RouteTask> tv{a, b};

    RouteParams base; base.cell_mm = 100.0; base.w_turn = 0.0; base.w_clear = 0.0;
    RouteParams bund = base; bund.w_corridor = 1000.0;

    auto rb = route_sequential(occ, tv, base, "original");
    auto rc = route_sequential(occ, tv, bund, "original", 0, 2, -1, false, 1);  // corridor_radius=1
    check(rb.success_count() == 2, "baseline 2/2 성공");
    check(rc.success_count() == 2, "회랑 2/2 성공");

    auto near_frac = [&](const std::vector<Cell>& target, const std::vector<Cell>& q) -> double {
        std::unordered_set<int> nearset;
        for (const Cell& c : target)
            for (int di = -2; di <= 2; ++di)
                for (int dj = -2; dj <= 2; ++dj) {
                    Cell n{c.i + di, c.j + dj, c.k};
                    if (occ.in_bounds(n)) nearset.insert(occ.lin(n));
                }
        if (q.empty()) return 0.0;
        int cnt = 0;
        for (const Cell& c : q) if (nearset.count(occ.lin(c))) ++cnt;
        return static_cast<double>(cnt) / static_cast<double>(q.size());
    };

    double base_near = near_frac(rb.pipes[0].result.path, rb.pipes[1].result.path);
    double bund_near = near_frac(rc.pipes[0].result.path, rc.pipes[1].result.path);
    std::printf("  near base=%.2f bundled=%.2f  len base=%.0f bundled=%.0f\n",
                base_near, bund_near, rb.total_length_mm(), rc.total_length_mm());

    check(bund_near > base_near, "회랑 인력 → B 가 A 근처를 더 많이 지남");
    check(bund_near > 0.5, "회랑 인력 → B 절반 이상이 A 곁");
    check(rc.total_length_mm() >= rb.total_length_mm() - 1e-6, "회랑 인력 → 총 길이 증가(기존설계 우회 모방)");

    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
