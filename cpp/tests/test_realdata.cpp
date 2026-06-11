// 실데이터 회귀 골든 (test_realdata) — Phase D
// =============================================================================
// [이 파일이 하는 일]
//   DDW_AI_DB 에서 추출해 scene.txt 로 동결한 '실데이터 장면'(장애물+스텁 종단점 작업)을
//   C ABI(routing3d_capi) 의 실제 라우팅 경로(r3d_route_multi)로 돌려, ① 전 배관 성공 수
//   ② 총 길이(결정적 스냅샷) ③ 재현성(두 번 라우팅 동일) 를 검증한다. DB 비의존(픽스처 고정)
//   이라 CI/로컬에서 동일 재현된다. 엔진 동작이 의도치 않게 바뀌면(성공수 감소·비결정) 즉시 탐지.
//
//   픽스처 재생성(의도 변경 시): Routing3D.Viewer.exe 환경변수 R3D_EXPORT_SCENE=<path> 로
//     --dbroute 1 100 Exhaust 를 돌리면 라우팅 직전 엔진 씬을 그 경로에 덤프한다(DbRouteDiag).
//
// [빌드/실행]  cmake --build cpp/build --config Release --target test_realdata
//             ctest --test-dir cpp/build -C Release -R realdata --output-on-failure
// =============================================================================
#include "routing3d_capi.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int g_failures = 0;
static void check(bool cond, const char* msg) {
    if (!cond) { std::printf("FAIL: %s\n", msg); ++g_failures; }
}

// scene.txt 파일 전체를 문자열로 읽는다.
static std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// 씬을 로드해 route_multi(priority) 후 (성공수, 총길이) 를 반환. 실패 시 success=-1.
// min_straight>0 이면 r3d_set_min_straight(C2) 적용. out_turns 옵션(널 가능).
static bool route_scene(const std::string& scene_text, const char* priority,
                        int task_count, int& out_ok, double& out_len,
                        double min_straight = 0.0, long long* out_turns = nullptr) {
    out_ok = 0; out_len = 0.0;
    if (out_turns) *out_turns = 0;
    R3dEngine* e = r3d_create();
    if (!e) return false;
    if (r3d_load_scene_text(e, scene_text.c_str()) != R3D_OK) { r3d_destroy(e); return false; }
    if (min_straight > 0.0) r3d_set_min_straight(e, min_straight);
    if (r3d_route_multi(e, priority) != R3D_OK) { r3d_destroy(e); return false; }
    for (int t = 0; t < task_count; ++t) {
        R3dResult r{};
        if (r3d_get_result(e, t, &r) != R3D_OK) { r3d_destroy(e); return false; }
        if (r.success) { ++out_ok; out_len += r.length_mm; if (out_turns) *out_turns += r.turns; }
    }
    r3d_destroy(e);
    return true;
}

int main() {
    std::printf("r3d_version: %s\n", r3d_version());

    // 픽스처: project1 Exhaust, cell=100 (장애물 632·작업 20). 라우팅 직전 엔진 씬 동결.
    const std::string fixture =
        std::string(SCENE_FIXTURE_DIR) + "/realdata_proj1_exhaust_c100.scene.txt";
    const std::string scene = read_file(fixture);
    check(!scene.empty(), "fixture read (realdata_proj1_exhaust_c100.scene.txt)");
    if (scene.empty()) { std::printf("  (픽스처 없음 — 경로/이름 확인)\n"); return 1; }

    const int TASKS = 20;

    // ① 라우팅 — 전 배관 성공 + 총 길이.
    int ok1 = 0; double len1 = 0.0;
    check(route_scene(scene, "longest", TASKS, ok1, len1), "route_scene #1");
    std::printf("realdata: success %d/%d, totalLen %.0f mm\n", ok1, TASKS, len1);

    // 전 배관 연결 성공(제품 핵심 지표). 실데이터 장면이 막혀 성공수가 떨어지면 회귀.
    check(ok1 == TASKS, "all tasks routed (20/20)");

    // 총 길이 스냅샷(결정적). 의도적 알고리즘 개선 시 이 값을 갱신한다(주석으로 명시).
    //   허용 오차 ±0.5% — 부동소수/플랫폼 미세차 흡수, 실질 경로 변화는 탐지.
    const double EXPECTED_LEN = 67600.0;
    check(std::fabs(len1 - EXPECTED_LEN) <= EXPECTED_LEN * 0.005,
          "total length within 0.5% of snapshot (67600 mm)");

    // ② 재현성 — 같은 입력 → 같은 결과(결정성 A2/W1).
    int ok2 = 0; double len2 = 0.0;
    check(route_scene(scene, "longest", TASKS, ok2, len2), "route_scene #2");
    check(ok1 == ok2 && len1 == len2, "deterministic (identical re-route)");

    // ③ C2 코너 최소반경(Phase C) 실데이터 불변식 — min_straight=2 가 성공·총길이·총꺾임을 악화시키지
    //    않는다(비교란 최종 패스: 라우팅 점유 불변 → 다운스트림 영향 0, 길이·꺾임 단조 비증가).
    int okB = 0, okM = 0; double lenB = 0.0, lenM = 0.0; long long tB = 0, tM = 0;
    check(route_scene(scene, "longest", TASKS, okB, lenB, 0.0, &tB), "route_scene base(turns)");
    check(route_scene(scene, "longest", TASKS, okM, lenM, 2.0, &tM), "route_scene min_straight=2");
    std::printf("C2 min-straight: base ok=%d len=%.0f turns=%lld | ms2 ok=%d len=%.0f turns=%lld\n",
                okB, lenB, tB, okM, lenM, tM);
    check(okM >= okB, "C2: min-straight never reduces success (realdata)");
    check(lenM <= lenB, "C2: min-straight never increases total length (realdata)");
    check(tM <= tB, "C2: min-straight never increases total turns (realdata)");

    if (g_failures == 0) std::printf("ALL REALDATA CHECKS PASSED\n");
    else std::printf("%d REALDATA CHECK(S) FAILED\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
