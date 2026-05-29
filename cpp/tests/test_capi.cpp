// C ABI 스모크 테스트 (routing3d_capi) — Phase 3
// =============================================================================
// [이 파일이 하는 일]
//   DLL(routing3d_capi)을 경유해 엔진을 호출하고, 핸들 ABI 로 골든03(5배관 순차)을
//   재현(5/5 성공·총 28050mm)하는지, 문자열 ABI(scene.txt 왕복)가 동작하는지 검증한다.
//   ctest 이름: capi.
//
// [빌드/실행]  cmake --build cpp/build --config Release --target test_capi
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

    // ---------------------------------------------------------------- Level 2: 핸들 ABI
    R3dEngine* e = r3d_create();
    check(e != nullptr, "r3d_create");
    if (!e) return 1;

    R3dGrid grid{50.0, 0.0, 0.0, 0.0, 120, 120, 60};
    check(r3d_set_grid(e, &grid) == R3D_OK, "r3d_set_grid");

    R3dParams params{50.0, 500.0, 10.0, 2, 6};  // baseline
    check(r3d_set_params(e, &params) == R3D_OK, "r3d_set_params");

    // 바닥 슬래브.
    check(r3d_add_obstacle(e, 0, 0, 0, 6000, 6000, 250) == R3D_OK, "r3d_add_obstacle");

    // 같은 통로 5개 배관(골든03).
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
    check(std::fabs(total - 28050.0) < 1e-6, "golden03 total 28050mm");

    // ---------------------------------------------------------------- Level 1: 문자열 ABI
    char* scene = nullptr;
    check(r3d_dump_scene_text(e, &scene) == R3D_OK && scene != nullptr, "r3d_dump_scene_text");

    char* routed = nullptr;
    check(r3d_route_scene_text(scene, "multi", "longest", &routed) == R3D_OK && routed != nullptr,
          "r3d_route_scene_text");

    if (routed) {
        std::string rs(routed);
        int succ = 0;
        size_t pos = 0;
        const std::string needle = "success\t1";
        while ((pos = rs.find(needle, pos)) != std::string::npos) {
            ++succ;
            pos += needle.size();
        }
        std::printf("[string] multi: %d success markers\n", succ);
        check(succ == 5, "level1 multi 5 success");
    }

    r3d_free_string(scene);
    r3d_free_string(routed);
    r3d_destroy(e);

    if (g_failures == 0) {
        std::printf("ALL CAPI TESTS PASSED\n");
        return 0;
    }
    std::printf("%d CAPI TEST(S) FAILED\n", g_failures);
    return 1;
}
