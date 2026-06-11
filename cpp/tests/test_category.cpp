// 카테고리 회귀 골든 (test_category) — Phase D2
// =============================================================================
// [이 파일이 하는 일]
//   엔진 고도화(Phase A/B) 신기능을 카테고리별로 직접 검증한다(프로그래밍 방식 C ABI, 픽스처 불필요):
//     ① 실패 사유 코드(A1)      — 묻힌 목표 → fail_reason == GoalBlocked.
//     ② 대형 격자(A3)           — >5M 셀(ImplicitOccupancy 경로) 개방 라우팅 성공·크래시 0.
//     ③ per-task 관경 반경(B1)  — 관경 다른 두 배관 + per_task ON → 둘 다 성공·성공 시 fail==None.
//     ④ C1 CBS(negotiated)      — 대형격자 다중배관에서 cbs_depth>0 가 성공 수 비감소(무손실)·결정적.
//     ⑤ C2 코너 최소반경         — min_straight>0 가 성공 비감소 + 꺾임 비증가(무손실 후처리)·결정적.
//   기본값=기존 동작 원칙(골든 불변)과 별개로, '새 동작이 의도대로 작동하는지'를 고정한다.
//
// [빌드/실행]  cmake --build cpp/build --config Release --target test_category
//             ctest --test-dir cpp/build -C Release -R category --output-on-failure
// =============================================================================
#include "routing3d_capi.h"

#include <cstdio>

static int g_failures = 0;
static void check(bool cond, const char* msg) {
    if (!cond) { std::printf("FAIL: %s\n", msg); ++g_failures; }
}

int main() {
    std::printf("r3d_version: %s\n", r3d_version());

    // ── ① 실패 사유(A1): 목표가 솔리드에 깊이 묻혀 스냅으로도 못 빠져나옴 → GoalBlocked ──
    {
        R3dEngine* e = r3d_create();
        R3dGrid grid{100.0, 0.0, 0.0, 0.0, 60, 60, 20};
        r3d_set_grid(e, &grid);
        R3dParams p{100.0, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
        r3d_set_params(e, &p);
        // 목표 (3000,3000,1000) 둘레 ±1000mm(10셀) 솔리드 → 스냅 반경(2) 으로 탈출 불가.
        r3d_add_obstacle(e, 2000, 2000, 0, 4000, 4000, 2000);
        int ti = r3d_add_task(e, 500, 500, 1000, 3000, 3000, 1000, "U", "G");
        check(ti >= 0, "cat1: add_task");
        check(r3d_route_multi(e, "longest") == R3D_OK, "cat1: route_multi");
        R3dResult r{};
        check(r3d_get_result(e, ti, &r) == R3D_OK, "cat1: get_result");
        check(r.success == 0, "cat1: buried goal fails");
        check(r.fail_reason == 2 /*GoalBlocked*/, "cat1: fail_reason == GoalBlocked");
        if (r.fail_reason != 2) std::printf("  (cat1 fail_reason=%d, expected 2)\n", r.fail_reason);
        r3d_destroy(e);
    }

    // ── ② 대형 격자(A3): 200x200x150 = 6,000,000 셀(>5M → Implicit) 개방 라우팅 성공 ──
    {
        R3dEngine* e = r3d_create();
        R3dGrid grid{100.0, 0.0, 0.0, 0.0, 200, 200, 150};
        r3d_set_grid(e, &grid);
        R3dParams p{100.0, 500.0, 10.0, 0.0, 2.0, 1.0, 2, 6};  // 대형=weighted(2.0)
        r3d_set_params(e, &p);
        r3d_add_obstacle(e, 5000, 5000, 5000, 6000, 6000, 6000);   // 작은 장애물 1개.
        int ti = r3d_add_task(e, 500, 500, 7500, 19000, 19000, 7500, "U", "G");
        check(ti >= 0, "cat2: add_task");
        check(r3d_route_multi(e, "longest") == R3D_OK, "cat2: route_multi (large grid)");
        R3dResult r{};
        check(r3d_get_result(e, ti, &r) == R3D_OK, "cat2: get_result");
        check(r.success != 0, "cat2: large-grid open route succeeds (Implicit)");
        if (!r.success) std::printf("  (cat2 fail_reason=%d)\n", r.fail_reason);
        r3d_destroy(e);
    }

    // ── ③ per-task 관경 반경(B1): 관경 다른 두 배관 + per_task ON → 둘 다 성공·fail==None ──
    {
        R3dEngine* e = r3d_create();
        R3dGrid grid{100.0, 0.0, 0.0, 0.0, 80, 40, 20};
        r3d_set_grid(e, &grid);
        R3dParams p{100.0, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
        r3d_set_params(e, &p);
        r3d_add_obstacle(e, 0, 0, 0, 8000, 4000, 300);   // 바닥.
        int t0 = r3d_add_task(e, 500, 1000, 1000, 7500, 1000, 1000, "U", "G");  // 굵은 관.
        int t1 = r3d_add_task(e, 500, 2500, 1000, 7500, 2500, 1000, "U", "G");  // 가는 관.
        r3d_set_task_diameter(e, t0, 300.0);   // radius 2.
        r3d_set_task_diameter(e, t1, 150.0);   // radius 1.
        check(r3d_set_per_task_radius(e, 1) == R3D_OK, "cat3: set_per_task_radius");
        check(r3d_route_multi(e, "diameter") == R3D_OK, "cat3: route_multi (per-task)");
        R3dResult r0{}, r1{};
        r3d_get_result(e, t0, &r0);
        r3d_get_result(e, t1, &r1);
        check(r0.success != 0 && r1.success != 0, "cat3: both mixed-diameter pipes route");
        check(r0.fail_reason == 0 && r1.fail_reason == 0, "cat3: success → fail_reason None");
        r3d_destroy(e);
    }

    // ── ④ C1 CBS(연쇄 rip-up): 대형격자 다중배관에서 cbs_depth>0 가 성공 비감소·결정적 ──
    //    합성 병목(여러 배관이 좁은 통로 공유)을 둔다. CBS 는 무손실(성공 단조 ≥ baseline)·결정적이 핵심
    //    불변식 — 특정 구제 건수가 아니라 'CBS 가 성공을 깎거나 비결정이 되지 않음'을 고정한다.
    {
        // 대형격자(>5M=Implicit·large_grid=true → CBS 경로 활성). 200x200x150 = 6M 셀, cell=100.
        auto build = [&](int cbs_depth, int& ok, long long& sig) {
            R3dEngine* e = r3d_create();
            R3dGrid grid{100.0, 0.0, 0.0, 0.0, 200, 200, 150};
            r3d_set_grid(e, &grid);
            R3dParams p{100.0, 500.0, 10.0, 0.0, 2.0, 1.0, 2, 6};
            r3d_set_params(e, &p);
            // 가운데 큰 벽 + 좁은 틈(여러 배관이 같은 틈으로 몰림 → 순서 의존 병목).
            r3d_add_obstacle(e, 9000, 0, 0, 11000, 9000, 15000);    // 아래 벽(틈 위쪽만 열림).
            r3d_add_obstacle(e, 9000, 11000, 0, 11000, 20000, 15000);  // 위 벽(틈은 y 9000~11000).
            int t[6];
            for (int q = 0; q < 6; ++q)
                t[q] = r3d_add_task(e, 1000, 2000 + q * 2800, 7500,
                                    19000, 2000 + q * 2800, 7500, "U", "G");
            if (cbs_depth > 0) r3d_set_cbs_depth(e, cbs_depth);
            r3d_route_multi(e, "longest");
            ok = 0; sig = 0;
            for (int q = 0; q < 6; ++q) {
                R3dResult r{};
                r3d_get_result(e, t[q], &r);
                if (r.success) { ++ok; sig += static_cast<long long>(r.length_mm) * (q + 1); }
            }
            r3d_destroy(e);
        };
        int ok0 = 0, okC = 0, okC2 = 0;
        long long sig0 = 0, sigC = 0, sigC2 = 0;
        build(0, ok0, sig0);     // baseline(평면 rip-up).
        build(2, okC, sigC);     // CBS depth=2.
        build(2, okC2, sigC2);   // 재현성.
        std::printf("cat4 CBS: baseline ok=%d, cbs2 ok=%d\n", ok0, okC);
        check(okC >= ok0, "cat4: CBS never reduces success (lossless)");
        check(okC == okC2 && sigC == sigC2, "cat4: CBS deterministic");
    }

    // ── ⑤ C2 코너 최소반경: min_straight>0 가 성공 비감소 + 꺾임 비증가(무손실 후처리)·결정적 ──
    {
        auto build = [&](double mult, int& ok, int& turns, long long& sig) {
            R3dEngine* e = r3d_create();
            R3dGrid grid{100.0, 0.0, 0.0, 0.0, 200, 200, 150};   // 대형격자(후처리 경로와 동일 조건).
            r3d_set_grid(e, &grid);
            R3dParams p{100.0, 500.0, 10.0, 0.0, 2.0, 1.0, 2, 6};
            r3d_set_params(e, &p);
            r3d_add_obstacle(e, 7000, 7000, 0, 9000, 13000, 15000);   // 우회 유발 장애물.
            int t[3];
            for (int q = 0; q < 3; ++q) {
                t[q] = r3d_add_task(e, 1000, 5000 + q * 3000, 7500,
                                    19000, 5000 + q * 3000, 7500, "U", "G");
                r3d_set_task_diameter(e, t[q], 200.0);
            }
            if (mult > 0.0) r3d_set_min_straight(e, mult);
            r3d_route_multi(e, "longest");
            ok = 0; turns = 0; sig = 0;
            for (int q = 0; q < 3; ++q) {
                R3dResult r{};
                r3d_get_result(e, t[q], &r);
                if (r.success) { ++ok; turns += r.turns; sig += static_cast<long long>(r.length_mm); }
            }
            r3d_destroy(e);
        };
        int ok0 = 0, okM = 0, okM2 = 0, tn0 = 0, tnM = 0, tnM2 = 0;
        long long sig0 = 0, sigM = 0, sigM2 = 0;
        build(0.0, ok0, tn0, sig0);    // baseline.
        build(2.0, okM, tnM, sigM);    // min-straight 2×관경.
        build(2.0, okM2, tnM2, sigM2); // 재현성.
        std::printf("cat5 min-straight: baseline ok=%d turns=%d, ms2 ok=%d turns=%d\n",
                    ok0, tn0, okM, tnM);
        check(okM >= ok0, "cat5: min-straight never reduces success");
        check(tnM <= tn0, "cat5: min-straight never increases turns (lossless post-process)");
        check(okM == okM2 && tnM == tnM2 && sigM == sigM2, "cat5: min-straight deterministic");
    }

    if (g_failures == 0) std::printf("ALL CATEGORY CHECKS PASSED\n");
    else std::printf("%d CATEGORY CHECK(S) FAILED\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
