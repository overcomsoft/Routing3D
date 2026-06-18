// Routing3D CLI ??C++ ?”ì§„ ?¤í–‰ ì§„ì…??(Phase 3)
// =============================================================================
// [???Œì¼???˜ëŠ” ??
//   ì§€ê¸ˆê¹Œì§€ êµ¬í˜„??C++ ?¼ìš°???”ì§„(occupancy/astar/multi_route/scene_io)??ëª…ë ¹ì¤„ì—??//   ë°”ë¡œ ?¤í–‰?œë‹¤. scene.json ë¥??½ì–´ ë°°ê????¼ìš°?…í•˜ê³?ê²°ê³¼(ê²½ë¡œ/ì§€??ë¥?scene.json ë¡?//   ?¤ì‹œ ?°ë©°, ?…ë ¥ ?Œì¼ ?†ì´???´ì¥ ?°ëª¨ ?¥ë©´???Œë ¤ë³????ˆë‹¤.
//
// [ë¹Œë“œ]  (?„ë¡œ?íŠ¸ ë£¨íŠ¸?ì„œ; ?¸ë? ?˜ì¡´??ë¶ˆí•„????ì½”ì–´ë§?
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release --target routing3d_cli
//
// [?¤í–‰]
//   # ???´ì¥ ?°ëª¨(ê³¨ë“ 03: ê°™ì? ?µë¡œ 5ê°?ë°°ê? ?œì°¨ ?¼ìš°?? ???…ë ¥ ë¶ˆí•„??//   cpp/build/Release/routing3d_cli.exe demo
//   cpp/build/Release/routing3d_cli.exe demo --out out.scene.json   # ê²°ê³¼ë¥?scene.json ë¡??€??//
//   # ??scene.json ë¥??½ì–´ ?¼ìš°????ê²°ê³¼ ?€??//   cpp/build/Release/routing3d_cli.exe route --in scene.json --out routed.scene.json --mode multi
//   cpp/build/Release/routing3d_cli.exe route --in scene.json --mode single
//
//   # ??scene.json ?”ì•½ë§?ì¶œë ¥
//   cpp/build/Release/routing3d_cli.exe summary --in scene.json
// =============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/scene_io.hpp"

using namespace routing3d;

namespace {

// AStarResult ??SceneResult(ì§ë ¬???¨ìœ„). ?±ê³µ ??ê²½ë¡œ ?ˆì´???¬í•¨.
SceneResult to_scene_result(const AStarResult& r) {
    SceneResult s;
    s.success = r.success;
    s.length_mm = r.length_mm;
    s.cost_mm = r.cost_mm;
    s.turns = r.turns;
    s.expanded_nodes = r.expanded_nodes;
    s.elapsed_ms = r.elapsed_ms;
    if (r.success) s.path = r.path;
    return s;
}

// ëª…ë ¹ì¤„ì—??--key value ?•íƒœ??ê°’ì„ ì°¾ëŠ”???†ìœ¼ë©?def).
std::string opt(int argc, char** argv, const std::string& key, const std::string& def = "") {
    for (int i = 2; i + 1 < argc; ++i)
        if (key == argv[i]) return argv[i + 1];
    return def;
}
bool has_flag(int argc, char** argv, const std::string& key) {
    for (int i = 2; i < argc; ++i)
        if (key == argv[i]) return true;
    return false;
}

void print_usage() {
    std::printf(
        "Routing3D CLI ???¬ìš©ë²?n"
        "  routing3d_cli demo [--out OUT.scene.json]\n"
        "      ?´ì¥ ?°ëª¨ ?¥ë©´(ê³¨ë“ 03: 5ê°?ë°°ê? ?œì°¨)???¼ìš°?…í•˜ê³??”ì•½ ì¶œë ¥.\n"
        "  routing3d_cli route --in IN.scene.json [--out OUT.scene.json] [--mode multi|single|ripup] [--priority longest]\n"
        "      scene.json ë¥??½ì–´ ?¼ìš°?…í•˜ê³?ê²°ê³¼ë¥?(ì§€???? ?€?? (ripup = ?œì°¨ ??rip-up&reroute)\n"
        "  routing3d_cli summary --in IN.scene.json\n"
        "      scene.json ?”ì•½ ì¶œë ¥.\n");
}

// ?¤ì¤‘ ?¼ìš°??ê²°ê³¼ë¥?doc ??ì±„ìš´??routed ?œì„œë¡?tasks/results ?¬êµ¬?????ê¸°?¼ê? scene.json).
template <class Occ>
void fill_multi(SceneDoc& doc, const Occ& occ, const std::string& priority) {
    auto mr = route_sequential(occ, doc.tasks, doc.params, priority);
    doc.tasks.clear();
    doc.results.clear();
    for (const PipeResult& p : mr.pipes) {
        doc.tasks.push_back(p.task);
        doc.results.push_back(to_scene_result(p.result));
    }
    std::printf("[?¤ì¤‘ë°°ê?/%s] %d/%zu ?±ê³µ (?¤íŒ¨ %d), ì´?ê¸¸ì´ %.0f mm\n", priority.c_str(),
                mr.success_count(), mr.pipes.size(), mr.fail_count(), mr.total_length_mm());
}

// rip-up & reroute(Step 3.8): ?œì°¨ ë² ì´?¤ë¼????ë§‰íŒ ë°°ê???blocker ??–´?´ê¸°ë¡??´ì†Œ.
template <class Occ>
void fill_ripup(SceneDoc& doc, const Occ& occ, const std::string& priority) {
    auto mr = route_ripup(occ, doc.tasks, doc.params, priority);
    doc.tasks.clear();
    doc.results.clear();
    for (const PipeResult& p : mr.pipes) {
        doc.tasks.push_back(p.task);
        doc.results.push_back(to_scene_result(p.result));
    }
    std::printf("[rip-up/%s] %d/%zu ?±ê³µ (?¤íŒ¨ %d), ì´?ê¸¸ì´ %.0f mm\n", priority.c_str(),
                mr.success_count(), mr.pipes.size(), mr.fail_count(), mr.total_length_mm());
}

// ?¨ì¼ ?¼ìš°?? ê°??‘ì—…???…ë¦½ A* ë¡??ë³¸ ?¥ì• ë¬??ìœ ë§?. results ??tasks ?€ ?‰í–‰.
template <class Occ>
void fill_single(SceneDoc& doc, const Occ& occ) {
    doc.results.assign(doc.tasks.size(), std::nullopt);
    int ok = 0;
    double total = 0.0;
    for (size_t i = 0; i < doc.tasks.size(); ++i) {
        const RouteTask& t = doc.tasks[i];
        AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm), doc.params);
        doc.results[i] = to_scene_result(r);
        if (r.success) { ++ok; total += r.length_mm; }
    }
    std::printf("[?¨ì¼ë°°ê?] %d/%zu ?±ê³µ, ì´?ê¸¸ì´ %.0f mm\n", ok, doc.tasks.size(), total);
}

void print_summary(const SceneDoc& doc) {
    long long blocked = occupancy_from_doc(doc).count_blocked();
    int with_res = 0, ok = 0;
    for (const auto& r : doc.results) {
        if (r.has_value()) { ++with_res; if (r->success) ++ok; }
    }
    std::printf("[scene] ê²©ì (%d,%d,%d) cell=%.0fmm origin=(%.0f,%.0f,%.0f) | ?¥ì• ë¬?%zu(?ìœ ?€ %lld) "
                "| ?‘ì—… %zu | ê²°ê³¼ %d/%d ?±ê³µ\n",
                doc.shape.i, doc.shape.j, doc.shape.k, doc.cell_mm, doc.origin.x, doc.origin.y,
                doc.origin.z, doc.obstacles.size(), blocked, doc.tasks.size(), ok, with_res);
}

// ?´ì¥ ?°ëª¨ ?¥ë©´(ê³¨ë“ 03): 120x120x60 ê²©ì, ë°”ë‹¥ ?¬ë˜ë¸? ê°™ì? ?µë¡œ 5ê°?ë°°ê?.
SceneDoc make_demo_doc() {
    SceneDoc doc;
    doc.cell_mm = 50.0;
    doc.origin = Vec3{0, 0, 0};
    doc.shape = Cell{120, 120, 60};
    doc.params = RouteParams{};  // baseline.
    Obstacle floor;
    floor.min_xyz = Vec3{0, 0, 0};
    floor.max_xyz = Vec3{6000, 6000, 250};
    floor.ost_type = "OST_Floors";
    doc.obstacles.push_back(floor);
    const char* utils[5][2] = {{"UPW_S", "UPW"}, {"NFW", "Waste Liquid"}, {"PA", "Gas"},
                               {"NW", "Water"}, {"ACID", "Exhaust"}};
    for (auto& u : utils) {
        RouteTask t;
        t.start_mm = Vec3{275, 3025, 1525};
        t.end_mm = Vec3{5725, 3025, 1525};
        t.utility = u[0];
        t.utility_group = u[1];
        doc.tasks.push_back(t);
    }
    return doc;
}

int cmd_demo(int argc, char** argv) {
    SceneDoc doc = make_demo_doc();
    std::printf("?´ì¥ ?°ëª¨ ?¥ë©´(ê³¨ë“ 03):\n");
    print_summary(doc);
    DenseOccupancy occ = occupancy_from_doc(doc);
    fill_multi(doc, occ, opt(argc, argv, "--priority", "longest"));
    const std::string out = opt(argc, argv, "--out");
    if (!out.empty()) {
        write_scene(out, doc);
        std::printf("[?€?? %s\n", out.c_str());
    }
    return 0;
}

int cmd_route(int argc, char** argv) {
    const std::string in = opt(argc, argv, "--in");
    if (in.empty()) { std::printf("?¤ë¥˜: --in ???„ìš”?©ë‹ˆ??\n"); print_usage(); return 2; }
    SceneDoc doc = read_scene(in);
    std::printf("[?…ë ¥] %s\n", in.c_str());
    print_summary(doc);
    DenseOccupancy occ = occupancy_from_doc(doc);

    const std::string mode = opt(argc, argv, "--mode", "multi");
    if (mode == "single") fill_single(doc, occ);
    else if (mode == "ripup") fill_ripup(doc, occ, opt(argc, argv, "--priority", "longest"));
    else fill_multi(doc, occ, opt(argc, argv, "--priority", "longest"));

    const std::string out = opt(argc, argv, "--out");
    if (!out.empty()) {
        write_scene(out, doc);
        std::printf("[?€?? %s\n", out.c_str());
    }
    return 0;
}

int cmd_summary(int argc, char** argv) {
    const std::string in = opt(argc, argv, "--in");
    if (in.empty()) { std::printf("?¤ë¥˜: --in ???„ìš”?©ë‹ˆ??\n"); print_usage(); return 2; }
    print_summary(read_scene(in));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // ?ˆë„??ì½˜ì†”?ì„œ UTF-8 ?œê? ì¶œë ¥(ì½”ë“œ?˜ì´ì§€ 65001).
#ifdef _WIN32
    std::system("chcp 65001 > nul");
#endif
    if (argc < 2) { print_usage(); return 1; }
    const std::string cmd = argv[1];
    try {
        if (cmd == "demo") return cmd_demo(argc, argv);
        if (cmd == "route") return cmd_route(argc, argv);
        if (cmd == "summary") return cmd_summary(argc, argv);
        if (cmd == "-h" || cmd == "--help" || cmd == "help") { print_usage(); return 0; }
    } catch (const std::exception& e) {
        std::printf("?¤ë¥˜: %s\n", e.what());
        return 3;
    }
    std::printf("?????†ëŠ” ëª…ë ¹: %s\n", cmd.c_str());
    print_usage();
    return 1;
}
