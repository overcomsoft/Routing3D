// Routing3D CLI entry point.
// Provides command-line scene loading, routing execution, and scene/result export.
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
        "Routing3D CLI ???용?n"
        "  routing3d_cli demo [--out OUT.scene.json]\n"
        "      ?장 ?모 ?면(골든03: 5?배? ?차)???우?하??약 출력.\n"
        "  routing3d_cli route --in IN.scene.json [--out OUT.scene.json] [--mode multi|single|ripup] [--priority longest]\n"
        "      scene.json ??어 ?우?하?결과?(지???? ??? (ripup = ?차 ??rip-up&reroute)\n"
        "  routing3d_cli summary --in IN.scene.json\n"
        "      scene.json ?약 출력.\n");
}

template <class Occ>
void fill_multi(SceneDoc& doc, const Occ& occ, const std::string& priority) {
    auto mr = route_sequential(occ, doc.tasks, doc.params, priority);
    doc.tasks.clear();
    doc.results.clear();
    for (const PipeResult& p : mr.pipes) {
        doc.tasks.push_back(p.task);
        doc.results.push_back(to_scene_result(p.result));
    }
    std::printf("[?중배?/%s] %d/%zu ?공 (?패 %d), ?길이 %.0f mm\n", priority.c_str(),
                mr.success_count(), mr.pipes.size(), mr.fail_count(), mr.total_length_mm());
}

template <class Occ>
void fill_ripup(SceneDoc& doc, const Occ& occ, const std::string& priority) {
    auto mr = route_ripup(occ, doc.tasks, doc.params, priority);
    doc.tasks.clear();
    doc.results.clear();
    for (const PipeResult& p : mr.pipes) {
        doc.tasks.push_back(p.task);
        doc.results.push_back(to_scene_result(p.result));
    }
    std::printf("[rip-up/%s] %d/%zu ?공 (?패 %d), ?길이 %.0f mm\n", priority.c_str(),
                mr.success_count(), mr.pipes.size(), mr.fail_count(), mr.total_length_mm());
}

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
    std::printf("[?일배?] %d/%zu ?공, ?길이 %.0f mm\n", ok, doc.tasks.size(), total);
}

void print_summary(const SceneDoc& doc) {
    long long blocked = occupancy_from_doc(doc).count_blocked();
    int with_res = 0, ok = 0;
    for (const auto& r : doc.results) {
        if (r.has_value()) { ++with_res; if (r->success) ++ok; }
    }
    std::printf("[scene] 격자 (%d,%d,%d) cell=%.0fmm origin=(%.0f,%.0f,%.0f) | ?애?%zu(?유? %lld) "
                "| ?업 %zu | 결과 %d/%d ?공\n",
                doc.shape.i, doc.shape.j, doc.shape.k, doc.cell_mm, doc.origin.x, doc.origin.y,
                doc.origin.z, doc.obstacles.size(), blocked, doc.tasks.size(), ok, with_res);
}

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
    std::printf("?장 ?모 ?면(골든03):\n");
    print_summary(doc);
    DenseOccupancy occ = occupancy_from_doc(doc);
    fill_multi(doc, occ, opt(argc, argv, "--priority", "longest"));
    const std::string out = opt(argc, argv, "--out");
    if (!out.empty()) {
        write_scene(out, doc);
        std::printf("[??? %s\n", out.c_str());
    }
    return 0;
}

int cmd_route(int argc, char** argv) {
    const std::string in = opt(argc, argv, "--in");
    if (in.empty()) { std::printf("?류: --in ???요?니??\n"); print_usage(); return 2; }
    SceneDoc doc = read_scene(in);
    std::printf("[?력] %s\n", in.c_str());
    print_summary(doc);
    DenseOccupancy occ = occupancy_from_doc(doc);

    const std::string mode = opt(argc, argv, "--mode", "multi");
    if (mode == "single") fill_single(doc, occ);
    else if (mode == "ripup") fill_ripup(doc, occ, opt(argc, argv, "--priority", "longest"));
    else fill_multi(doc, occ, opt(argc, argv, "--priority", "longest"));

    const std::string out = opt(argc, argv, "--out");
    if (!out.empty()) {
        write_scene(out, doc);
        std::printf("[??? %s\n", out.c_str());
    }
    return 0;
}

int cmd_summary(int argc, char** argv) {
    const std::string in = opt(argc, argv, "--in");
    if (in.empty()) { std::printf("?류: --in ???요?니??\n"); print_usage(); return 2; }
    print_summary(read_scene(in));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
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
        std::printf("?류: %s\n", e.what());
        return 3;
    }
    std::printf("?????는 명령: %s\n", cmd.c_str());
    print_usage();
    return 1;
}
