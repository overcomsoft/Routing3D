// scene.txt I/O 교차검증 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.9)
// =============================================================================
// [이 파일이 하는 일]
//   1) format_repr_double 가 Python repr(float) 와 동일한 표기를 내는지(F4) 검증.
//   2) Python 이 만든 골든 픽스처(tests/fixtures/roundtrip.scene.txt)를 C++ 가
//      읽고(loads_scene) 다시 써서(dumps_scene) **원본 바이트와 동일**한지(F2) 검증.
//   3) self round-trip: write→read→write 가 동일(파서/라이터가 서로 역).
//   4) \N(None) vs ""(빈 문자열) 구분 보존(F3) + 점유맵 복원(occupancy_from_doc).
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
//
//   픽스처 재생성(알고리즘 의도 변경 시): cpp/tests/fixtures/_gen_fixture.py
// =============================================================================
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "routing3d/occupancy.hpp"
#include "routing3d/scene_io.hpp"

using namespace routing3d;

static int g_failures = 0;

static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

static std::string read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { std::printf("  [FAIL] 파일 열기 실패: %s\n", path.c_str()); ++g_failures; return {}; }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// 두 문자열의 첫 불일치 위치를 사람이 읽기 좋게 보고(디버깅용).
static void report_first_diff(const std::string& a, const std::string& b) {
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            std::printf("    첫 불일치 @%zu: 기대 0x%02X('%c') vs 실제 0x%02X('%c')\n",
                        i, (unsigned char)a[i], a[i], (unsigned char)b[i], b[i]);
            return;
        }
    }
    if (a.size() != b.size())
        std::printf("    길이 차이: 기대 %zu vs 실제 %zu\n", a.size(), b.size());
}

// ---------------------------------------------------------------- (1) 실수 표기
static void test_repr_float() {
    std::printf("=== format_repr_double (F4) ===\n");
    struct Case { double v; const char* want; };
    const Case cases[] = {
        {0.1, "0.1"}, {0.0001, "0.0001"}, {1e-05, "1e-05"}, {1e16, "1e+16"},
        {1e15, "1000000000000000.0"}, {2.5e-10, "2.5e-10"},
        {9999999999999998.0, "9999999999999998.0"}, {3.141592653589793, "3.141592653589793"},
        {-0.0, "-0.0"}, {0.05, "0.05"}, {0.123, "0.123"}, {470.5, "470.5"},
        {75.125, "75.125"}, {275.25, "275.25"}, {50.5, "50.5"}, {25.5, "25.5"},
        {6000.0, "6000.0"}, {250.0, "250.0"}, {100000.0, "100000.0"}, {-2150.5, "-2150.5"},
        {0.0, "0.0"}, {1.0, "1.0"},
    };
    for (const Case& c : cases) {
        std::string got = format_repr_double(c.v);
        bool ok = (got == c.want);
        if (!ok) std::printf("    repr(%g): 기대 '%s' vs 실제 '%s'\n", c.v, c.want, got.c_str());
        check(ok, std::string("repr == '") + c.want + "'");
    }
}

// ---------------------------------------------------------------- (2)(3) 왕복
static SceneDoc test_roundtrip() {
    std::printf("=== scene.txt round-trip (F2/F3) ===\n");
    const std::string fixture = std::string(SCENE_FIXTURE_DIR) + "/roundtrip.scene.txt";
    const std::string original = read_file(fixture);

    SceneDoc doc = loads_scene(original);
    std::string out1 = dumps_scene(doc);

    bool byte_id = (out1 == original);
    if (!byte_id) report_first_diff(original, out1);
    check(byte_id, "C++ 재출력이 Python 픽스처와 바이트 동일 (F2 교차검증)");

    // self round-trip: write→read→write 동일.
    SceneDoc doc2 = loads_scene(out1);
    std::string out2 = dumps_scene(doc2);
    check(out1 == out2, "self round-trip 동일 (write→read→write)");
    return doc;
}

// ---------------------------------------------------------------- (4) 구조/F3
static void test_structure(const SceneDoc& doc) {
    std::printf("=== 파싱 구조 / \\N vs \"\" 구분 (F3) ===\n");
    check(doc.cell_mm == 50.0, "cell_mm == 50.0");
    check(doc.shape == (Cell{120, 120, 60}), "shape == (120,120,60)");
    check(doc.obstacles.size() == 4, "obstacles == 4");
    check(doc.tasks.size() == 2, "tasks == 2");
    check(doc.params.w_tier.size() == 2 && doc.params.w_tier.at(3) == 50.5, "w_tier{1,3} 복원");

    // 장애물[1]: name="" (빈 문자열, 존재) / object_id=None / ddworks_type 존재.
    const Obstacle& o1 = doc.obstacles[1];
    check(o1.name.has_value() && o1.name->empty(), "obstacles[1].name == \"\" (빈문자열)");
    check(!o1.object_id.has_value(), "obstacles[1].object_id == None(\\N)");
    check(doc.obstacles[0].ddworks_type == std::nullopt, "obstacles[0].ddworks_type == None");
    // 유니코드/특수문자 보존.
    check(doc.obstacles[2].name.has_value() && *doc.obstacles[2].name == "한글 이름 / unicode",
          "유니코드/'//'/공백 이름 보존");
    check(doc.obstacles[2].object_id.has_value() && doc.obstacles[2].object_id->empty(),
          "obstacles[2].object_id == \"\" (빈문자열 ≠ None)");

    // 작업[1]: utility=None, utility_group="" (구분).
    const RouteTask& t1 = doc.tasks[1];
    check(!t1.utility.has_value(), "tasks[1].utility == None");
    check(t1.utility_group.has_value() && t1.utility_group->empty(), "tasks[1].utility_group == \"\"");

    // 결과: [0] 성공+경로+방문 / [1] 실패+경로·방문 없음.
    check(doc.results.size() == 2, "results 평행 길이 2");
    check(doc.results[0].has_value() && doc.results[0]->success, "results[0] 성공");
    check(doc.results[0]->path.has_value() && doc.results[0]->path->size() == 3, "results[0].path 3셀");
    check(doc.results[0]->visited.has_value() && doc.results[0]->visited->size() == 3,
          "results[0].visited 3셀");
    check(doc.results[1].has_value() && !doc.results[1]->success, "results[1] 실패");
    check(!doc.results[1]->path.has_value(), "results[1].path 없음(None)");
    check(!doc.results[1]->visited.has_value(), "results[1].visited 없음(None)");
}

// ---------------------------------------------------------------- (5) 점유맵 복원
static void test_occupancy(const SceneDoc& doc) {
    std::printf("=== occupancy_from_doc ===\n");
    DenseOccupancy occ = occupancy_from_doc(doc);
    check(occ.shape() == doc.shape, "복원 점유맵 shape 일치");
    check(occ.cell_mm() == doc.cell_mm, "복원 점유맵 cell_mm 일치");
    // 바닥 슬래브(z 0~250mm) → 점유 셀 다수.
    check(occ.count_blocked() > 0, "장애물 복셀화로 점유 셀 > 0");
}

int main() {
    test_repr_float();
    SceneDoc doc = test_roundtrip();
    test_structure(doc);
    test_occupancy(doc);
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
