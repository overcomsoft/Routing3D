// Tests for scene IO parsing and serialization compatibility.
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
    if (!f) { std::printf("  [FAIL] ?일 ?기 ?패: %s\n", path.c_str()); ++g_failures; return {}; }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void report_first_diff(const std::string& a, const std::string& b) {
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            std::printf("    ?불일?@%zu: 기? 0x%02X('%c') vs ?제 0x%02X('%c')\n",
                        i, (unsigned char)a[i], a[i], (unsigned char)b[i], b[i]);
            return;
        }
    }
    if (a.size() != b.size())
        std::printf("    길이 차이: 기? %zu vs ?제 %zu\n", a.size(), b.size());
}

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
        if (!ok) std::printf("    repr(%g): 기? '%s' vs ?제 '%s'\n", c.v, c.want, got.c_str());
        check(ok, std::string("repr == '") + c.want + "'");
    }
}

static SceneDoc test_roundtrip() {
    std::printf("=== scene.txt round-trip (F2/F3) ===\n");
    const std::string fixture = std::string(SCENE_FIXTURE_DIR) + "/roundtrip.scene.txt";
    const std::string original = read_file(fixture);

    SceneDoc doc = loads_scene(original);
    std::string out1 = dumps_scene(doc);

    check(out1.find("\"version\": 3") != std::string::npos,
          "legacy fixture loads and writes the current JSON v3 scene format");

    SceneDoc doc2 = loads_scene(out1);
    std::string out2 = dumps_scene(doc2);
    check(out1 == out2, "self round-trip ?일 (write?read?write)");
    return doc;
}

// ---------------------------------------------------------------- (4) 구조/F3
static void test_structure(const SceneDoc& doc) {
    std::printf("=== ?싱 구조 / \\N vs \"\" 구분 (F3) ===\n");
    check(doc.cell_mm == 50.0, "cell_mm == 50.0");
    check(doc.shape == (Cell{120, 120, 60}), "shape == (120,120,60)");
    check(doc.obstacles.size() == 4, "obstacles == 4");
    check(doc.tasks.size() == 2, "tasks == 2");
    check(doc.params.w_tier.size() == 2 && doc.params.w_tier.at(3) == 50.5, "w_tier{1,3} 복원");
    check(doc.params.w_corridor == 0.0 && doc.params.w_heur == 1.0 && doc.params.w_heur_near == 0.0,
          "v1 missing params use v2 defaults");

    const Obstacle& o1 = doc.obstacles[1];
    check(o1.name.has_value() && o1.name->empty(), "obstacles[1].name == \"\" (빈문?열)");
    check(!o1.object_id.has_value(), "obstacles[1].object_id == None(\\N)");
    check(doc.obstacles[0].ddworks_type == std::nullopt, "obstacles[0].ddworks_type == None");
    check(doc.obstacles[2].name.has_value() && !doc.obstacles[2].name->empty(),
          "obstacles[2].name preserves non-empty text");
    check(doc.obstacles[2].object_id.has_value() && doc.obstacles[2].object_id->empty(),
          "obstacles[2].object_id == \"\" (빈문?열 ??None)");

    const RouteTask& t1 = doc.tasks[1];
    check(!t1.utility.has_value(), "tasks[1].utility == None");
    check(t1.utility_group.has_value() && t1.utility_group->empty(), "tasks[1].utility_group == \"\"");

    check(doc.results.size() == 2, "results ?행 길이 2");
    check(doc.results[0].has_value() && doc.results[0]->success, "results[0] ?공");
    check(doc.results[0]->path.has_value() && doc.results[0]->path->size() == 3, "results[0].path 3?");
    check(doc.results[0]->visited.has_value() && doc.results[0]->visited->size() == 3,
          "results[0].visited 3?");
    check(doc.results[1].has_value() && !doc.results[1]->success, "results[1] ?패");
    check(!doc.results[1]->path.has_value(), "results[1].path ?음(None)");
    check(!doc.results[1]->visited.has_value(), "results[1].visited ?음(None)");
}

static void test_occupancy(const SceneDoc& doc) {
    std::printf("=== occupancy_from_doc ===\n");
    DenseOccupancy occ = occupancy_from_doc(doc);
    check(occ.shape() == doc.shape, "복원 ?유?shape ?치");
    check(occ.cell_mm() == doc.cell_mm, "복원 ?유?cell_mm ?치");
    check(occ.count_blocked() > 0, "?애?복??로 ?유 ? > 0");
}

int main() {
    test_repr_float();
    SceneDoc doc = test_roundtrip();
    test_structure(doc);
    test_occupancy(doc);
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
