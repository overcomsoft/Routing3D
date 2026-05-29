// rip-up & reroute 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.8)
// =============================================================================
// [이 파일이 하는 일]
//   합성 혼잡 시나리오로 route_ripup 이 순차(greedy) 라우팅의 실패를 해소함을 확인한다.
//   벽(j=4)에 틈 2개(i=2, i=6). 긴 배관(LONG)이 먼저 틈6 을 차지하면 짧은 배관(SHORT)이
//   막혀 순차는 1/2 에 그친다. rip-up 은 LONG 을 틈2 로 우회시켜 둘 다 성공(2/2).
//   값(LONG 1000→1300, SHORT 900)은 Python routing3d_py 합성 결과와 동일(교차검증).
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake --build cpp/build --config Release --target test_ripup
//   ctest --test-dir cpp/build -C Release -R ripup --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>

#include "routing3d/cost.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;
static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

// 정렬 순서(longest)에서 utility 라벨로 PipeResult 를 찾는다.
template <class MR>
static const AStarResult* find(const MR& mr, const std::string& util) {
    for (const auto& p : mr.pipes)
        if (p.task.utility && *p.task.utility == util) return &p.result;
    return nullptr;
}

static Vec3 center(double i, double j, double k = 0) {
    const double C = 100.0;
    return Vec3{(i + 0.5) * C, (j + 0.5) * C, (k + 0.5) * C};
}

int main() {
    std::printf("=== rip-up 합성 혼잡(틈 2개 벽) ===\n");
    const double C = 100.0;
    DenseOccupancy occ(Cell{9, 9, 1}, Vec3{0, 0, 0}, C);
    // j=4 행에 벽; i=2, i=6 만 틈.
    for (int i = 0; i < 9; ++i) {
        if (i == 2 || i == 6) continue;
        occ.add_box(AABB(Vec3{i * C, 4 * C, 0}, Vec3{(i + 1) * C, 5 * C, C}));
    }

    std::vector<RouteTask> tasks;
    {
        RouteTask t;
        t.start_mm = center(6, 0);
        t.end_mm = center(4, 8);  // dist 10 → 먼저, 틈6 선호.
        t.utility = "LONG";
        t.utility_group = "Demo";
        tasks.push_back(t);
    }
    {
        RouteTask t;
        t.start_mm = center(7, 0);
        t.end_mm = center(6, 8);  // dist 9 → 나중, 틈6 필요.
        t.utility = "SHORT";
        t.utility_group = "Demo";
        tasks.push_back(t);
    }

    RouteParams p;
    p.cell_mm = C;  // 나머지는 baseline 기본(Python RouteParams(cell_mm=100) 과 동일).

    auto base = route_sequential(occ, tasks, p, "longest");
    auto rip = route_ripup(occ, tasks, p, "longest");
    std::printf("  순차 %d/%zu, rip-up %d/%zu\n", base.success_count(), base.pipes.size(),
                rip.success_count(), rip.pipes.size());

    check(base.success_count() == 1, "순차(greedy)=1/2 (SHORT 막힘)");
    check(rip.success_count() == 2, "rip-up=2/2 (혼잡 해소, +1)");

    const AStarResult* bl = find(base, "LONG");
    const AStarResult* rl = find(rip, "LONG");
    const AStarResult* rs = find(rip, "SHORT");
    check(bl && bl->success && bl->length_mm == 1000.0, "순차 LONG 성공·길이 1000mm");
    check(rl && rl->success && rl->length_mm == 1300.0, "rip-up LONG 우회·길이 1300mm (Python 일치)");
    check(rs && rs->success && rs->length_mm == 900.0, "rip-up SHORT 성공·길이 900mm (Python 일치)");

    // 무손실 보장: rip-up 성공 수 >= 순차.
    check(rip.success_count() >= base.success_count(), "rip-up 성공 수 >= 순차(무손실)");

    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
