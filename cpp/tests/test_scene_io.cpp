// scene.txt I/O êµì°¨ê²€ì¦??ŒìŠ¤????Routing3D C++ ?”ì§„ (Phase 3, Step 3.9)
// =============================================================================
// [???Œì¼???˜ëŠ” ??
//   1) format_repr_double ê°€ Python repr(float) ?€ ?™ì¼???œê¸°ë¥??´ëŠ”ì§€(F4) ê²€ì¦?
//   2) Python ??ë§Œë“  ê³¨ë“  ?½ìŠ¤ì²?tests/fixtures/roundtrip.scene.txt)ë¥?C++ ê°€
//      ?½ê³ (loads_scene) ?¤ì‹œ ?¨ì„œ(dumps_scene) **?ë³¸ ë°”ì´?¸ì? ?™ì¼**?œì?(F2) ê²€ì¦?
//   3) self round-trip: write?’read?’write ê°€ ?™ì¼(?Œì„œ/?¼ì´?°ê? ?œë¡œ ??.
//   4) \N(None) vs ""(ë¹?ë¬¸ì?? êµ¬ë¶„ ë³´ì¡´(F3) + ?ìœ ë§?ë³µì›(occupancy_from_doc).
//
// [ë¹Œë“œ/?¤í–‰]  (?„ë¡œ?íŠ¸ ë£¨íŠ¸?ì„œ)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release --output-on-failure
//
//   ?½ìŠ¤ì²??¬ìƒ???Œê³ ë¦¬ì¦˜ ?˜ë„ ë³€ê²???: cpp/tests/fixtures/_gen_fixture.py
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
    if (!f) { std::printf("  [FAIL] ?Œì¼ ?´ê¸° ?¤íŒ¨: %s\n", path.c_str()); ++g_failures; return {}; }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// ??ë¬¸ì?´ì˜ ì²?ë¶ˆì¼ì¹??„ì¹˜ë¥??¬ëŒ???½ê¸° ì¢‹ê²Œ ë³´ê³ (?”ë²„ê¹…ìš©).
static void report_first_diff(const std::string& a, const std::string& b) {
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            std::printf("    ì²?ë¶ˆì¼ì¹?@%zu: ê¸°ë? 0x%02X('%c') vs ?¤ì œ 0x%02X('%c')\n",
                        i, (unsigned char)a[i], a[i], (unsigned char)b[i], b[i]);
            return;
        }
    }
    if (a.size() != b.size())
        std::printf("    ê¸¸ì´ ì°¨ì´: ê¸°ë? %zu vs ?¤ì œ %zu\n", a.size(), b.size());
}

// ---------------------------------------------------------------- (1) ?¤ìˆ˜ ?œê¸°
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
        if (!ok) std::printf("    repr(%g): ê¸°ë? '%s' vs ?¤ì œ '%s'\n", c.v, c.want, got.c_str());
        check(ok, std::string("repr == '") + c.want + "'");
    }
}

// ---------------------------------------------------------------- (2)(3) ?•ë³µ
static SceneDoc test_roundtrip() {
    std::printf("=== scene.txt round-trip (F2/F3) ===\n");
    const std::string fixture = std::string(SCENE_FIXTURE_DIR) + "/roundtrip.scene.txt";
    const std::string original = read_file(fixture);

    SceneDoc doc = loads_scene(original);
    std::string out1 = dumps_scene(doc);

    check(out1.find("\"version\": 3") != std::string::npos,
          "legacy fixture loads and writes the current JSON v3 scene format");

    // self round-trip: write?’read?’write ?™ì¼.
    SceneDoc doc2 = loads_scene(out1);
    std::string out2 = dumps_scene(doc2);
    check(out1 == out2, "self round-trip ?™ì¼ (write?’read?’write)");
    return doc;
}

// ---------------------------------------------------------------- (4) êµ¬ì¡°/F3
static void test_structure(const SceneDoc& doc) {
    std::printf("=== ?Œì‹± êµ¬ì¡° / \\N vs \"\" êµ¬ë¶„ (F3) ===\n");
    check(doc.cell_mm == 50.0, "cell_mm == 50.0");
    check(doc.shape == (Cell{120, 120, 60}), "shape == (120,120,60)");
    check(doc.obstacles.size() == 4, "obstacles == 4");
    check(doc.tasks.size() == 2, "tasks == 2");
    check(doc.params.w_tier.size() == 2 && doc.params.w_tier.at(3) == 50.5, "w_tier{1,3} ë³µì›");
    check(doc.params.w_corridor == 0.0 && doc.params.w_heur == 1.0 && doc.params.w_heur_near == 0.0,
          "v1 missing params use v2 defaults");

    // ?¥ì• ë¬?1]: name="" (ë¹?ë¬¸ì?? ì¡´ì¬) / object_id=None / ddworks_type ì¡´ì¬.
    const Obstacle& o1 = doc.obstacles[1];
    check(o1.name.has_value() && o1.name->empty(), "obstacles[1].name == \"\" (ë¹ˆë¬¸?ì—´)");
    check(!o1.object_id.has_value(), "obstacles[1].object_id == None(\\N)");
    check(doc.obstacles[0].ddworks_type == std::nullopt, "obstacles[0].ddworks_type == None");
    // ? ë‹ˆì½”ë“œ/?¹ìˆ˜ë¬¸ì ë³´ì¡´.
    check(doc.obstacles[2].name.has_value() && !doc.obstacles[2].name->empty(),
          "obstacles[2].name preserves non-empty text");
    check(doc.obstacles[2].object_id.has_value() && doc.obstacles[2].object_id->empty(),
          "obstacles[2].object_id == \"\" (ë¹ˆë¬¸?ì—´ ??None)");

    // ?‘ì—…[1]: utility=None, utility_group="" (êµ¬ë¶„).
    const RouteTask& t1 = doc.tasks[1];
    check(!t1.utility.has_value(), "tasks[1].utility == None");
    check(t1.utility_group.has_value() && t1.utility_group->empty(), "tasks[1].utility_group == \"\"");

    // ê²°ê³¼: [0] ?±ê³µ+ê²½ë¡œ+ë°©ë¬¸ / [1] ?¤íŒ¨+ê²½ë¡œÂ·ë°©ë¬¸ ?†ìŒ.
    check(doc.results.size() == 2, "results ?‰í–‰ ê¸¸ì´ 2");
    check(doc.results[0].has_value() && doc.results[0]->success, "results[0] ?±ê³µ");
    check(doc.results[0]->path.has_value() && doc.results[0]->path->size() == 3, "results[0].path 3?€");
    check(doc.results[0]->visited.has_value() && doc.results[0]->visited->size() == 3,
          "results[0].visited 3?€");
    check(doc.results[1].has_value() && !doc.results[1]->success, "results[1] ?¤íŒ¨");
    check(!doc.results[1]->path.has_value(), "results[1].path ?†ìŒ(None)");
    check(!doc.results[1]->visited.has_value(), "results[1].visited ?†ìŒ(None)");
}

// ---------------------------------------------------------------- (5) ?ìœ ë§?ë³µì›
static void test_occupancy(const SceneDoc& doc) {
    std::printf("=== occupancy_from_doc ===\n");
    DenseOccupancy occ = occupancy_from_doc(doc);
    check(occ.shape() == doc.shape, "ë³µì› ?ìœ ë§?shape ?¼ì¹˜");
    check(occ.cell_mm() == doc.cell_mm, "ë³µì› ?ìœ ë§?cell_mm ?¼ì¹˜");
    // ë°”ë‹¥ ?¬ë˜ë¸?z 0~250mm) ???ìœ  ?€ ?¤ìˆ˜.
    check(occ.count_blocked() > 0, "?¥ì• ë¬?ë³µì??”ë¡œ ?ìœ  ?€ > 0");
}

int main() {
    test_repr_float();
    SceneDoc doc = test_roundtrip();
    test_structure(doc);
    test_occupancy(doc);
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
