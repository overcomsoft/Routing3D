// Routing3D CLI — C++ 엔진 실행 진입점 (Phase 3)
// =============================================================================
// [이 파일이 하는 일]
//   지금까지 구현한 C++ 라우팅 엔진(occupancy/astar/multi_route/scene_io)을 명령줄에서
//   바로 실행한다. scene.txt 를 읽어 배관을 라우팅하고 결과(경로/지표)를 scene.txt 로
//   다시 쓰며, 입력 파일 없이도 내장 데모 장면을 돌려볼 수 있다.
//
// [빌드]  (프로젝트 루트에서; 외부 의존성 불필요 — 코어만)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release --target routing3d_cli
//
// [실행]
//   # ① 내장 데모(골든03: 같은 통로 5개 배관 순차 라우팅) — 입력 불필요
//   cpp/build/Release/routing3d_cli.exe demo
//   cpp/build/Release/routing3d_cli.exe demo --out out.scene.txt   # 결과를 scene.txt 로 저장
//
//   # ② scene.txt 를 읽어 라우팅 후 결과 저장
//   cpp/build/Release/routing3d_cli.exe route --in scene.txt --out routed.scene.txt --mode multi
//   cpp/build/Release/routing3d_cli.exe route --in scene.txt --mode single
//
//   # ③ scene.txt 요약만 출력
//   cpp/build/Release/routing3d_cli.exe summary --in scene.txt
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

// AStarResult → SceneResult(직렬화 단위). 성공 시 경로 레이어 포함.
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

// 명령줄에서 --key value 형태의 값을 찾는다(없으면 def).
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
        "Routing3D CLI — 사용법\n"
        "  routing3d_cli demo [--out OUT.scene.txt]\n"
        "      내장 데모 장면(골든03: 5개 배관 순차)을 라우팅하고 요약 출력.\n"
        "  routing3d_cli route --in IN.scene.txt [--out OUT.scene.txt] [--mode multi|single] [--priority longest]\n"
        "      scene.txt 를 읽어 라우팅하고 결과를 (지정 시) 저장.\n"
        "  routing3d_cli summary --in IN.scene.txt\n"
        "      scene.txt 요약 출력.\n");
}

// 다중 라우팅 결과를 doc 에 채운다(routed 순서로 tasks/results 재구성 → 자기일관 scene.txt).
template <class Occ>
void fill_multi(SceneDoc& doc, const Occ& occ, const std::string& priority) {
    auto mr = route_sequential(occ, doc.tasks, doc.params, priority);
    doc.tasks.clear();
    doc.results.clear();
    for (const PipeResult& p : mr.pipes) {
        doc.tasks.push_back(p.task);
        doc.results.push_back(to_scene_result(p.result));
    }
    std::printf("[다중배관/%s] %d/%zu 성공 (실패 %d), 총 길이 %.0f mm\n", priority.c_str(),
                mr.success_count(), mr.pipes.size(), mr.fail_count(), mr.total_length_mm());
}

// 단일 라우팅: 각 작업을 독립 A* 로(원본 장애물 점유맵). results 는 tasks 와 평행.
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
    std::printf("[단일배관] %d/%zu 성공, 총 길이 %.0f mm\n", ok, doc.tasks.size(), total);
}

void print_summary(const SceneDoc& doc) {
    long long blocked = occupancy_from_doc(doc).count_blocked();
    int with_res = 0, ok = 0;
    for (const auto& r : doc.results) {
        if (r.has_value()) { ++with_res; if (r->success) ++ok; }
    }
    std::printf("[scene] 격자 (%d,%d,%d) cell=%.0fmm origin=(%.0f,%.0f,%.0f) | 장애물 %zu(점유셀 %lld) "
                "| 작업 %zu | 결과 %d/%d 성공\n",
                doc.shape.i, doc.shape.j, doc.shape.k, doc.cell_mm, doc.origin.x, doc.origin.y,
                doc.origin.z, doc.obstacles.size(), blocked, doc.tasks.size(), ok, with_res);
}

// 내장 데모 장면(골든03): 120x120x60 격자, 바닥 슬래브, 같은 통로 5개 배관.
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
    std::printf("내장 데모 장면(골든03):\n");
    print_summary(doc);
    DenseOccupancy occ = occupancy_from_doc(doc);
    fill_multi(doc, occ, opt(argc, argv, "--priority", "longest"));
    const std::string out = opt(argc, argv, "--out");
    if (!out.empty()) {
        write_scene(out, doc);
        std::printf("[저장] %s\n", out.c_str());
    }
    return 0;
}

int cmd_route(int argc, char** argv) {
    const std::string in = opt(argc, argv, "--in");
    if (in.empty()) { std::printf("오류: --in 이 필요합니다.\n"); print_usage(); return 2; }
    SceneDoc doc = read_scene(in);
    std::printf("[입력] %s\n", in.c_str());
    print_summary(doc);
    DenseOccupancy occ = occupancy_from_doc(doc);

    const std::string mode = opt(argc, argv, "--mode", "multi");
    if (mode == "single") fill_single(doc, occ);
    else fill_multi(doc, occ, opt(argc, argv, "--priority", "longest"));

    const std::string out = opt(argc, argv, "--out");
    if (!out.empty()) {
        write_scene(out, doc);
        std::printf("[저장] %s\n", out.c_str());
    }
    return 0;
}

int cmd_summary(int argc, char** argv) {
    const std::string in = opt(argc, argv, "--in");
    if (in.empty()) { std::printf("오류: --in 이 필요합니다.\n"); print_usage(); return 2; }
    print_summary(read_scene(in));
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // 윈도우 콘솔에서 UTF-8 한글 출력(코드페이지 65001).
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
        std::printf("오류: %s\n", e.what());
        return 3;
    }
    std::printf("알 수 없는 명령: %s\n", cmd.c_str());
    print_usage();
    return 1;
}
