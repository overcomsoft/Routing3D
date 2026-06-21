// C ABI smoke tests for routing3d_capi.
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

    R3dEngine* e = r3d_create();
    check(e != nullptr, "r3d_create");
    if (!e) return 1;

    R3dGrid grid{50.0, 0.0, 0.0, 0.0, 120, 120, 60};
    check(r3d_set_grid(e, &grid) == R3D_OK, "r3d_set_grid");

    R3dParams params{50.0, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
    check(r3d_set_params(e, &params) == R3D_OK, "r3d_set_params");

    check(r3d_add_obstacle(e, 0, 0, 0, 6000, 6000, 250) == R3D_OK, "r3d_add_obstacle");

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

    {
        R3dEngine* be = r3d_create();
        check(be != nullptr, "corridor create");
        const double sc = 50.0;
        R3dGrid bg{sc, 0, 0, 0, 2000, 2000, 8};
        check(r3d_set_grid(be, &bg) == R3D_OK, "corridor set_grid");
        R3dParams bp{sc, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
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

    {
        struct Ctx { int phase1 = 0; int phase0 = 0; bool cancel = false; };
        auto cb = [](void* user, int32_t phase, int32_t, int32_t, int32_t, double, int32_t,
                     int64_t, double, int32_t, int32_t, double, const int32_t*, int32_t) -> int32_t {
            Ctx* c = static_cast<Ctx*>(user);
            if (phase == 0) ++c->phase0;
            if (phase == 1) ++c->phase1;
            return c->cancel ? 1 : 0;
        };
        R3dGrid pg{50.0, 0.0, 0.0, 0.0, 120, 120, 60};
        R3dParams pp{50.0, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};

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

    {
        R3dEngine* he = r3d_create();
        check(he != nullptr, "hpa_hier create");
        const double sc = 50.0;
        R3dGrid hg{sc, 0.0, 0.0, 0.0, 96, 96, 16};
        R3dParams hp{sc, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
        check(r3d_set_grid(he, &hg) == R3D_OK, "hpa_hier set_grid");
        check(r3d_set_params(he, &hp) == R3D_OK, "hpa_hier set_params");
        R3dRuntimeOptions ho{};
        ho.large_grid_threshold = 1;
        ho.max_expansions = 200000;
        ho.fallback_expansions = 200000;
        ho.hier_factor = 8;
        ho.hier_radius = 2;
        ho.hier_probe = 1;
        ho.ripup_enabled = 0;
        ho.cbs_expansions = 0;
        check(r3d_set_runtime_options(he, &ho) == R3D_OK, "hpa_hier runtime_options");
        int ti = r3d_add_task(he, 125.0, 125.0, 125.0, 4525.0, 4025.0, 125.0, "HPA", "Guide");
        check(ti == 0, "hpa_hier add_task");
        check(r3d_route_multi(he, "longest") == R3D_OK, "hpa_hier route_multi");
        R3dResult hr{};
        check(r3d_get_result(he, 0, &hr) == R3D_OK, "hpa_hier get_result");
        std::printf("[hpa_hier] success=%d length=%.0f path=%d expanded=%lld\n",
                    hr.success, hr.length_mm, hr.path_len, hr.expanded_nodes);
        check(hr.success != 0, "hpa_hier success via hierarchical route");
        check(hr.length_mm >= 8300.0 && hr.length_mm <= 9000.0, "hpa_hier length near manhattan");
        r3d_destroy(he);
    }
    {
        R3dEngine* se = r3d_create();
        check(se != nullptr, "route_split create");
        const double sc = 50.0;
        R3dGrid sg{sc, 0.0, 0.0, 0.0, 40, 40, 20};
        R3dParams sp{sc, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
        check(r3d_set_grid(se, &sg) == R3D_OK, "route_split set_grid");
        check(r3d_set_params(se, &sp) == R3D_OK, "route_split set_params");
        check(r3d_set_segment_astar(se, 1, 64) == R3D_OK, "route_split segment_astar");
        check(r3d_set_route_split(se, 1, 500.0) == R3D_OK, "route_split enable");
        int ti = r3d_add_task(se, 125.0, 125.0, 125.0, 1625.0, 1125.0, 125.0, "EXH", "Exhaust");
        check(ti == 0, "route_split add_task");
        check(r3d_route_multi(se, "longest") == R3D_OK, "route_split route_multi");
        R3dResult sr{};
        check(r3d_get_result(se, 0, &sr) == R3D_OK, "route_split get_result");
        std::vector<int> path(static_cast<size_t>(sr.path_len) * 3);
        int copied = sr.path_len > 0 ? r3d_copy_path(se, 0, path.data(), sr.path_len) : 0;
        bool touches_trunk = false;
        for (int p = 0; p < copied; ++p) {
            if (path[static_cast<size_t>(p) * 3 + 2] == 10) touches_trunk = true;
        }
        std::printf("[route_split] success=%d length=%.0f path=%d trunk_k10=%d expanded=%lld\n",
                    sr.success, sr.length_mm, sr.path_len, touches_trunk ? 1 : 0, sr.expanded_nodes);
        check(sr.success != 0, "route_split success");
        check(touches_trunk, "route_split path touches requested trunk z");
        check(sr.length_mm > 2500.0, "route_split length includes truck-in and terminal legs");
        r3d_destroy(se);
    }
    {
        R3dEngine* ae = r3d_create();
        check(ae != nullptr, "anytime create");
        const double sc = 50.0;
        R3dGrid ag{sc, 0.0, 0.0, 0.0, 80, 40, 10};
        R3dParams ap{sc, 500.0, 10.0, 0.0, 1.0, 0.0, 2, 6};
        check(r3d_set_grid(ae, &ag) == R3D_OK, "anytime set_grid");
        check(r3d_set_params(ae, &ap) == R3D_OK, "anytime set_params");
        int ti = r3d_add_task(ae, 125.0, 125.0, 125.0, 3625.0, 1625.0, 125.0, "ANY", "AStar");
        check(ti == 0, "anytime add_task");
        R3dResult ar{};
        int iters = 0, improves = 0;
        check(r3d_route_task_anytime(ae, 0, 3.0, 1.0, 1.0, 5000.0, -1, -1,
                                     &ar, &iters, &improves) == R3D_OK,
              "anytime route_task_anytime");
        std::printf("[anytime] success=%d length=%.0f cost=%.0f iterations=%d improvements=%d expanded=%lld\n",
                    ar.success, ar.length_mm, ar.cost_mm, iters, improves, ar.expanded_nodes);
        check(ar.success != 0, "anytime success");
        check(iters >= 3, "anytime ran scheduled weight passes");
        check(improves >= 1, "anytime recorded at least initial incumbent");
        check(ar.length_mm >= 5000.0 && ar.length_mm <= 5300.0, "anytime length near manhattan");
        r3d_destroy(ae);
    }
    if (g_failures == 0) {
        std::printf("ALL CAPI TESTS PASSED\n");
        return 0;
    }
    std::printf("%d CAPI TEST(S) FAILED\n", g_failures);
    return 1;
}
