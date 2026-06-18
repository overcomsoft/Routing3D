// C ABI ?�모???�스??(routing3d_capi) ??Phase 3
// =============================================================================
// [???�일???�는 ??
//   DLL(routing3d_capi)??경유???�진???�출?�고, ?�들 ABI �?골든03(5배�? ?�차)??
//   ?�현(5/5 ?�공·�?28050mm)?�는지, 문자??ABI(scene.txt ?�복)가 ?�작?�는지 검증한??
//   ctest ?�름: capi.
//
// [빌드/?�행]  cmake --build cpp/build --config Release --target test_capi
//             ctest --test-dir cpp/build -C Release -R capi --output-on-failure
// =============================================================================
#include "routing3d_capi.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int g_failures = 0;

static void check(bool cond, const char* msg) {
    if (!cond) {
        std::printf("FAIL: %s\n", msg);
        ++g_failures;
    }
}

int main() {
    std::printf("r3d_version: %s\n", r3d_version());

    // ---------------------------------------------------------------- Level 2: ?�들 ABI
    R3dEngine* e = r3d_create();
    check(e != nullptr, "r3d_create");
    if (!e) return 1;

    R3dGrid grid{50.0, 0.0, 0.0, 0.0, 120, 120, 60};
    check(r3d_set_grid(e, &grid) == R3D_OK, "r3d_set_grid");

    R3dParams params{50.0, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};  // cell,turn,clear,corridor,heur(1=?��?),heur_near(0=?�적),clr_r,clr_conn
    check(r3d_set_params(e, &params) == R3D_OK, "r3d_set_params");

    // 바닥 ?�래�?
    check(r3d_add_obstacle(e, 0, 0, 0, 6000, 6000, 250) == R3D_OK, "r3d_add_obstacle");

    // 같�? ?�로 5�?배�?(골든03).
    const char* utils[5][2] = {{"UPW_S", "UPW"}, {"NFW", "Waste Liquid"}, {"PA", "Gas"},
                               {"NW", "Water"}, {"ACID", "Exhaust"}};
    for (auto& u : utils) {
        int idx = r3d_add_task(e, 275, 3025, 1525, 5725, 3025, 1525, u[0], u[1]);
        check(idx >= 0, "r3d_add_task");
    }

    check(r3d_route_multi(e, "longest") == R3D_OK, "r3d_route_multi");

    int ok = 0;
    double total = 0.0;
    for (int t = 0; t < 5; ++t) {
        R3dResult r{};
        check(r3d_get_result(e, t, &r) == R3D_OK, "r3d_get_result");
        if (r.success) {
            ++ok;
            total += r.length_mm;
        }
        if (r.path_len > 0) {
            std::vector<int> buf(static_cast<size_t>(r.path_len) * 3);
            int n = r3d_copy_path(e, t, buf.data(), r.path_len);
            check(n == r.path_len, "r3d_copy_path count");
        }
    }
    std::printf("[handle] multi: %d/5 success, total %.0f mm\n", ok, total);
    check(ok == 5, "golden03 success 5/5");
    #ifdef ROUTING3D_USE_OPENVDB
    check(std::fabs(total - 28350.0) < 1e-6, "openvdb golden03 total 28350mm");
#else
    check(std::fabs(total - 28050.0) < 1e-6, "golden03 total 28050mm");
#endif

    // ---------------------------------------------------------------- Level 1: 문자??ABI
    char* scene = nullptr;
    check(r3d_dump_scene_text(e, &scene) == R3D_OK && scene != nullptr, "r3d_dump_scene_text");

    char* routed = nullptr;
    check(r3d_route_scene_text(scene, "multi", "longest", &routed) == R3D_OK && routed != nullptr,
          "r3d_route_scene_text");

    if (routed) {
        std::string rs(routed);
        int succ = 0;
        size_t pos = 0;
        const std::string needle = "\"success\":true";
        while ((pos = rs.find(needle, pos)) != std::string::npos) {
            ++succ;
            pos += needle.size();
        }
        std::printf("[string] multi: %d success markers\n", succ);
        check(succ == 5, "level1 multi 5 success markers in JSON");
    }

    r3d_free_string(scene);
    r3d_free_string(routed);
    r3d_destroy(e);

    // ---------------------------------------------------------------- corridor(?�??Sparse)
    // 2000x2000x8 격자(=Dense weighted A* ??closed 배열 2000*2000*8*7??.2e11 불�?)�?
    // Sparse + corridor �??�우?? �?공간 직선 ??길이 = 맨해??× cell.
    {
        R3dEngine* be = r3d_create();
        check(be != nullptr, "corridor create");
        const double sc = 50.0;
        R3dGrid bg{sc, 0, 0, 0, 2000, 2000, 8};
        check(r3d_set_grid(be, &bg) == R3D_OK, "corridor set_grid");
        R3dParams bp{sc, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};  // w_corridor=0=off, w_heur=1=?��?, heur_near=0=?�적
        check(r3d_set_params(be, &bp) == R3D_OK, "corridor set_params");

        const int si = 10, sj = 10, sk = 4, gi = 1990, gj = 1990, gk = 4;
        double sx = (si + 0.5) * sc, sy = (sj + 0.5) * sc, sz = (sk + 0.5) * sc;
        double gx = (gi + 0.5) * sc, gy = (gj + 0.5) * sc, gz = (gk + 0.5) * sc;
        int ti = r3d_add_task(be, sx, sy, sz, gx, gy, gz, "X", "Y");
        check(ti == 0, "corridor add_task");

        check(r3d_route_corridor(be, 16, 4) == R3D_OK, "route_corridor");
        R3dResult cr{};
        check(r3d_get_result(be, 0, &cr) == R3D_OK, "corridor get_result");
        double man = (double)((gi - si) + (gj - sj) + (gk - sk)) * sc;  // 198000
        std::printf("[corridor] success=%d length=%.0f (manhattan=%.0f) expanded=%lld\n",
                    cr.success, cr.length_mm, man, cr.expanded_nodes);
        check(cr.success != 0, "corridor success on huge sparse scene");
        check(cr.length_mm >= man - 1e-6 && cr.length_mm <= man * 1.10, "corridor length ~ manhattan");
        r3d_destroy(be);
    }

    // ---------------------------------------------------------------- 진행 콜백 + ?�력??취소
    // route_multi_progress ??콜백 ABI(int 반환=취소)�?검증한?? (1) ?�상(??�� 0 반환)?�면 골든03
    // 5/5 ?�현 + phase 1 콜백 5?? (2) 콜백??1(취소)??반환?�면 즉시 중단?�어 ?�료 배�? ?��? 5 미만.
    {
        struct Ctx { int phase1 = 0; int phase0 = 0; bool cancel = false; };
        auto cb = [](void* user, int32_t phase, int32_t, int32_t, int32_t, double, int32_t,
                     int64_t, double, int32_t, int32_t, double, const int32_t*, int32_t) -> int32_t {
            Ctx* c = static_cast<Ctx*>(user);
            if (phase == 0) ++c->phase0;
            if (phase == 1) ++c->phase1;
            return c->cancel ? 1 : 0;   // cancel=true �?취소(0?�님) 반환.
        };
        R3dGrid pg{50.0, 0.0, 0.0, 0.0, 120, 120, 60};
        R3dParams pp{50.0, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};

        // ?�상: 취소 ?�음 ??5배�? ?��? ?�료.
        R3dEngine* pe = r3d_create();
        check(pe != nullptr, "progress create");
        r3d_set_grid(pe, &pg);
        r3d_set_params(pe, &pp);
        r3d_add_obstacle(pe, 0, 0, 0, 6000, 6000, 250);
        for (auto& u : utils)
            r3d_add_task(pe, 275, 3025, 1525, 5725, 3025, 1525, u[0], u[1]);
        Ctx ok_ctx;
        check(r3d_route_multi_progress(pe, "longest", cb, &ok_ctx) == R3D_OK, "route_multi_progress ok");
        std::printf("[progress] no-cancel: phase1=%d (expect 5)\n", ok_ctx.phase1);
        check(ok_ctx.phase1 == 5, "progress phase1 fires 5 times (all pipes)");
        int pok = 0;
        for (int t = 0; t < 5; ++t) { R3dResult r{}; r3d_get_result(pe, t, &r); if (r.success) ++pok; }
        check(pok == 5, "progress no-cancel routes 5/5");
        r3d_destroy(pe);

        // 취소: �?콜백부??취소 반환 ??배치가 5배�???모두 ?�료?�기 ?�에 중단.
        R3dEngine* ce = r3d_create();
        r3d_set_grid(ce, &pg);
        r3d_set_params(ce, &pp);
        r3d_add_obstacle(ce, 0, 0, 0, 6000, 6000, 250);
        for (auto& u : utils)
            r3d_add_task(ce, 275, 3025, 1525, 5725, 3025, 1525, u[0], u[1]);
        Ctx cancel_ctx; cancel_ctx.cancel = true;
        check(r3d_route_multi_progress(ce, "longest", cb, &cancel_ctx) == R3D_OK,
              "route_multi_progress cancel returns OK");
        std::printf("[progress] cancel: phase1=%d (expect <5)\n", cancel_ctx.phase1);
        check(cancel_ctx.phase1 < 5, "cancel stops batch before all pipes complete");
        r3d_destroy(ce);
    }

    if (g_failures == 0) {
        std::printf("ALL CAPI TESTS PASSED\n");
        return 0;
    }
    std::printf("%d CAPI TEST(S) FAILED\n", g_failures);
    return 1;
}
