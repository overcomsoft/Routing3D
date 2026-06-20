// Routing3D live C ABI implementation (routing3d_capi) - Phase 3
// =============================================================================
// Provides an exception-safe C facade over the C++ Routing3D engine.
// The opaque R3dEngine handle owns SceneDoc and runtime options; exported APIs
// translate C/PInvoke calls into occupancy, A*, multi-route, octree, and scene IO
// operations while returning R3dStatus instead of throwing across the ABI boundary.
//
// Build check:
//   cmake --build cpp/build --config Release --target routing3d_capi
//   ctest --test-dir cpp/build -C Release -R capi --output-on-failure
// =============================================================================
#ifndef ROUTING3D_CAPI_EXPORTS
#define ROUTING3D_CAPI_EXPORTS
#endif
#include "routing3d_capi.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/corridor.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/octree_occupancy.hpp"
#include "routing3d/scene_io.hpp"
#ifdef ROUTING3D_USE_OPENVDB
#include "routing3d/vdb_occupancy.hpp"
#endif

using namespace routing3d;

namespace {

std::string json_escape(const std::string& s) {
    std::ostringstream os;
    for (unsigned char ch : s) {
        switch (ch) {
        case '\\': os << "\\\\"; break;
        case '"':  os << "\\\""; break;
        case '\n': os << "\\n"; break;
        case '\r': os << "\\r"; break;
        case '\t': os << "\\t"; break;
        default:
            if (ch < 0x20) os << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch);
            else os << ch;
        }
    }
    return os.str();
}

std::string cell_json(const Cell& c) {
    return "[" + std::to_string(c.i) + "," + std::to_string(c.j) + "," + std::to_string(c.k) + "]";
}

std::string cell_array_json(const std::vector<Cell>& cells) {
    std::string s;
    s.reserve(cells.size() * 18 + 2);
    s.push_back('[');
    for (size_t i = 0; i < cells.size(); ++i) {
        if (i) s.push_back(',');
        s += cell_json(cells[i]);
    }
    s.push_back(']');
    return s;
}

std::string vec_json(const Vec3& v) {
    std::ostringstream os;
    os << std::setprecision(12) << "[" << v.x << "," << v.y << "," << v.z << "]";
    return os.str();
}

struct TraceWriter {
    R3dTraceOptions opt{};
    std::string path;
    std::ofstream out;
    int current_task = -1;
    int events_this_task = 0;
    bool warned_task_limit = false;

    bool enabled() const { return opt.enabled != 0 && out.good(); }
    int sample_every() const { return opt.sample_every > 0 ? opt.sample_every : 1000; }
    int max_events_per_task() const { return opt.max_events_per_task > 0 ? opt.max_events_per_task : 20000; }

    bool open(const std::string& p) {
        path = p;
        out.close();
        out.clear();
        out.open(path, std::ios::out | std::ios::trunc);
        return out.good();
    }

    void write_raw(const std::string& json) {
        if (!enabled()) return;
        if (current_task >= 0 && events_this_task >= max_events_per_task()) {
            if (!warned_task_limit) {
                out << "{\"type\":\"trace_limit\",\"task\":" << current_task
                    << ",\"max_events\":" << max_events_per_task() << "}\n";
                warned_task_limit = true;
            }
            return;
        }
        out << json << '\n';
        if (current_task >= 0) ++events_this_task;
    }

    void task_scope(int task) {
        if (task != current_task) {
            current_task = task;
            events_this_task = 0;
            warned_task_limit = false;
        }
    }

    void flush() {
        if (out.good()) out.flush();
    }
};

}  // namespace

struct R3dEngine {
    SceneDoc doc;
    // OFF by default to avoid large memory growth; callers can opt in via r3d_set_collect_visited(1).
    bool collect_visited = false;
    // Optional learned corridor seed cells used when w_corridor > 0.
    std::vector<Cell> corridor_seed;
    // Global pipe dilation radius in cells for multi-route collision avoidance.
    int pipe_radius = 0;
    bool per_task_radius = false;
    // Additional centerline spacing between routed pipes in millimeters.
    double pipe_gap_mm = 0.0;
    // CBS-lite recursion depth. 0 disables negotiated congestion resolution.
    int cbs_depth = 0;
    // Post-process minimum straight-run multiplier, relative to pipe diameter.
    double min_straight_mult = 0.0;
    // 코너 최소직선(절대 mm, 하드 제약). >0 이면 A* 가 '한 번 꺾인 뒤 이 길이만큼 직진하기 전엔 다시 꺾지
    //   못하도록' 강제한다(상태에 진행 셀 수 run 추가). min_straight_mult(관경 배수·후처리 흡수)와 달리
    //   탐색 단계의 하드 보장이며 관경 무관·전 배관 적용(목표 직전 마지막 구간은 면제). 셀로는
    //   ceil(min_straight_mm/cell)→params.min_straight_cells. 0=OFF(골든 불변). r3d_set_min_straight_mm.
    double min_straight_mm = 0.0;
    R3dRuntimeOptions runtime{};
    R3dTraceOptions trace_options{};
    TraceWriter trace;
};

namespace {

long long env_ll(const char* name, long long def) {
    if (const char* s = std::getenv(name)) {
        char* end = nullptr;
        long long v = std::strtoll(s, &end, 10);
        if (end != s && v > 0) return v;
    }
    return def;
}

long long large_grid_cap() { return env_ll("R3D_MAX_EXP", 48000000LL); }

// 코너 최소직선(절대 mm) → A* 상태 제약용 셀 수로 변환해 doc.params 에 반영. 라우팅 직전에 호출한다
//   (cell_mm 이 set_params 로 확정된 뒤). env R3D_MIN_STRAIGHT_MM(>0)이 있으면 우선. 0/미설정이면 0(OFF,
//   골든 불변). route_multi_impl 의 params_for 가 doc.params 를 복사하므로 main·rip-up·CBS 전부에 전파된다.
void apply_min_straight_cells(R3dEngine* e) {
    double mm = e->min_straight_mm;
    if (const char* s = std::getenv("R3D_MIN_STRAIGHT_MM")) {
        char* end = nullptr; double v = std::strtod(s, &end);
        if (end != s && v >= 0.0) mm = v;
    }
    const double cell = e->doc.params.cell_mm;
    e->doc.params.min_straight_cells =
        (mm > 0.0 && cell > 0.0) ? static_cast<int>(std::ceil(mm / cell)) : 0;
}
long long opt_or_default(long long value, long long dflt) { return value > 0 ? value : dflt; }
int opt_or_default_i(int value, int dflt) { return value > 0 ? value : dflt; }

template <class Occ>
long long safe_blocked_count(const Occ& occ) {
    try { return occ.count_blocked(); }
    catch (...) { return -1; }
}

void trace_header(TraceWriter* tr, const SceneDoc& doc, const std::string& priority) {
    if (!tr || !tr->enabled()) return;
    tr->current_task = -1;
    tr->write_raw("{\"type\":\"trace_header\",\"version\":1,\"engine\":\"routing3d_capi\","
                  "\"priority\":\"" + json_escape(priority) + "\","
                  "\"cell_mm\":" + std::to_string(doc.cell_mm) + ","
                  "\"origin\":" + vec_json(doc.origin) + ","
                  "\"shape\":" + cell_json(doc.shape) + ","
                  "\"task_count\":" + std::to_string(doc.tasks.size()) + ","
                  "\"obstacle_count\":" + std::to_string(doc.obstacles.size()) + "}");
}

template <class Occ>
void trace_occupancy_summary(TraceWriter* tr, const Occ& occ) {
    if (!tr || !tr->enabled() || !tr->opt.include_occupancy) return;
    tr->current_task = -1;
    tr->write_raw("{\"type\":\"occupancy_summary\",\"blocked_count\":" +
                  std::to_string(safe_blocked_count(occ)) + "}");
}

std::vector<Cell> sample_obstacle_cells(const SceneDoc& doc,
                                        const std::vector<Obstacle>& boxes,
                                        int max_cells) {
    std::vector<CellRange> ranges;
    ranges.reserve(boxes.size());
    long long total = 0;

    for (const auto& ob : boxes) {
        CellRange r = grid_box_range(AABB(ob.min_xyz, ob.max_xyz), doc.origin, doc.cell_mm, doc.shape);
        long long n = static_cast<long long>(r.hi.i - r.lo.i) *
                      static_cast<long long>(r.hi.j - r.lo.j) *
                      static_cast<long long>(r.hi.k - r.lo.k);
        if (n <= 0) continue;
        ranges.push_back(r);
        total += n;
    }

    std::vector<Cell> cells;
    if (total <= 0 || max_cells <= 0) return cells;
    const long long take = std::min<long long>(total, max_cells);
    cells.reserve(static_cast<size_t>(take));

    long long ordinal = 0;
    long long picked = 0;
    long long next_pick = 0;
    for (const auto& r : ranges) {
        for (int k = r.lo.k; k < r.hi.k; ++k) {
            for (int j = r.lo.j; j < r.hi.j; ++j) {
                for (int i = r.lo.i; i < r.hi.i; ++i, ++ordinal) {
                    if (picked >= take) return cells;
                    if (ordinal < next_pick) continue;
                    cells.push_back(Cell{i, j, k});
                    ++picked;
                    next_pick = (picked * total) / take;
                }
            }
        }
    }
    return cells;
}

void trace_cell_sample(TraceWriter* tr, const std::string& type,
                       const std::vector<Cell>& cells, long long total) {
    if (!tr || !tr->enabled() || !tr->opt.include_occupancy) return;
    tr->current_task = -1;
    tr->write_raw("{\"type\":\"" + json_escape(type) + "\",\"total\":" + std::to_string(total) +
                  ",\"sampled\":" + std::to_string(cells.size()) +
                  ",\"cells\":" + cell_array_json(cells) + "}");
}

void trace_static_occupancy_samples(TraceWriter* tr, const SceneDoc& doc) {
    if (!tr || !tr->enabled() || !tr->opt.include_occupancy) return;
    int max_occ = 12000;
    if (const char* s = std::getenv("R3D_TRACE_OCC_CELLS")) {
        char* end = nullptr;
        long v = std::strtol(s, &end, 10);
        if (end != s && v > 0) max_occ = static_cast<int>(std::min<long>(v, 100000));
    }
    const int max_pass = std::max(1000, max_occ / 3);

    auto count_cells = [&](const std::vector<Obstacle>& boxes) {
        long long total = 0;
        for (const auto& ob : boxes) {
            CellRange r = grid_box_range(AABB(ob.min_xyz, ob.max_xyz), doc.origin, doc.cell_mm, doc.shape);
            total += static_cast<long long>(r.hi.i - r.lo.i) *
                     static_cast<long long>(r.hi.j - r.lo.j) *
                     static_cast<long long>(r.hi.k - r.lo.k);
        }
        return total;
    };

    trace_cell_sample(tr, "occupancy_sample",
                      sample_obstacle_cells(doc, doc.obstacles, max_occ),
                      count_cells(doc.obstacles));
    trace_cell_sample(tr, "passthrough_sample",
                      sample_obstacle_cells(doc, doc.passthrough, max_pass),
                      count_cells(doc.passthrough));
}

void trace_task_begin(TraceWriter* tr, int order_index, int task_index, const RouteTask& task,
                      const Cell& start_cell, const Cell& goal_cell,
                      const Cell& snapped_start, const Cell& snapped_goal) {
    if (!tr || !tr->enabled()) return;
    tr->task_scope(task_index);
    tr->write_raw("{\"type\":\"task_begin\",\"order\":" + std::to_string(order_index) +
                  ",\"task\":" + std::to_string(task_index) +
                  ",\"source_world\":" + vec_json(task.start_mm) +
                  ",\"target_world\":" + vec_json(task.end_mm) +
                  ",\"source_cell\":" + cell_json(start_cell) +
                  ",\"target_cell\":" + cell_json(goal_cell) +
                  ",\"snapped_source\":" + cell_json(snapped_start) +
                  ",\"snapped_target\":" + cell_json(snapped_goal) +
                  ",\"utility\":\"" + json_escape(task.utility.value_or("")) +
                  "\",\"group\":\"" + json_escape(task.utility_group.value_or("")) + "\"}");
    if (!(start_cell == snapped_start))
        tr->write_raw("{\"type\":\"snap\",\"task\":" + std::to_string(task_index) +
                      ",\"kind\":\"start\",\"from\":" + cell_json(start_cell) +
                      ",\"to\":" + cell_json(snapped_start) + "}");
    if (!(goal_cell == snapped_goal))
        tr->write_raw("{\"type\":\"snap\",\"task\":" + std::to_string(task_index) +
                      ",\"kind\":\"goal\",\"from\":" + cell_json(goal_cell) +
                      ",\"to\":" + cell_json(snapped_goal) + "}");
}

void trace_expand(TraceWriter* tr, int order_index, int task_index, long long expanded, double progress) {
    if (!tr || !tr->enabled() || expanded <= 0) return;
    const int sample = tr->sample_every();
    if (expanded % sample != 0) return;
    tr->task_scope(task_index);
    std::ostringstream os;
    os << "{\"type\":\"expand\",\"order\":" << order_index
       << ",\"task\":" << task_index
       << ",\"expanded_nodes\":" << expanded
       << ",\"progress01\":" << std::setprecision(8) << progress << "}";
    tr->write_raw(os.str());
}

const char* trace_reject_reason(const char* event) {
    if (!event) return "unknown";
    if (std::strcmp(event, "candidate_reject_out_of_bounds") == 0) return "out_of_bounds";
    if (std::strcmp(event, "candidate_reject_blocked") == 0) return "blocked";
    if (std::strcmp(event, "candidate_reject_corridor_gate") == 0) return "corridor_gate";
    if (std::strcmp(event, "candidate_reject_min_straight") == 0) return "min_straight";
    return event;
}

void trace_search_cell(TraceWriter* tr, int order_index, int task_index, const char* event,
                       const Cell& from, const Cell& to, long long expanded,
                       int dir, int run, int required) {
    if (!tr || !tr->enabled() || expanded <= 0 || !event) return;
    const int sample = tr->sample_every();
    if (expanded > 5 && expanded % sample != 0) return;

    tr->task_scope(task_index);
    std::ostringstream os;
    if (std::strcmp(event, "expand_cell") == 0) {
        os << "{\"type\":\"expand_cell\",\"order\":" << order_index
           << ",\"task\":" << task_index
           << ",\"expanded_nodes\":" << expanded
           << ",\"cell\":" << cell_json(to)
           << ",\"dir\":" << dir
           << ",\"run\":" << run
           << ",\"required\":" << required << "}";
        tr->write_raw(os.str());
        return;
    }

    if (!tr->opt.include_rejects) return;
    os << "{\"type\":\"candidate_reject\",\"order\":" << order_index
       << ",\"task\":" << task_index
       << ",\"expanded_nodes\":" << expanded
       << ",\"reason\":\"" << trace_reject_reason(event) << "\""
       << ",\"from\":" << cell_json(from)
       << ",\"to\":" << cell_json(to)
       << ",\"dir\":" << dir
       << ",\"run\":" << run
       << ",\"required\":" << required << "}";
    tr->write_raw(os.str());
}

void trace_task_end(TraceWriter* tr, int order_index, int task_index, const AStarResult& res,
                    bool ok, bool aborted) {
    if (!tr || !tr->enabled()) return;
    tr->task_scope(task_index);
    std::ostringstream os;
    os << "{\"type\":\"task_end\",\"order\":" << order_index
       << ",\"task\":" << task_index
       << ",\"success\":" << (ok ? "true" : "false")
       << ",\"aborted\":" << (aborted ? "true" : "false")
       << ",\"fail_reason\":" << static_cast<int>(res.fail)
       << ",\"length_mm\":" << std::setprecision(12) << res.length_mm
       << ",\"turns\":" << res.turns
       << ",\"expanded_nodes\":" << res.expanded_nodes
       << ",\"elapsed_ms\":" << res.elapsed_ms
       << ",\"path_len\":" << (res.path.empty() ? 0 : static_cast<int>(res.path.size())) << "}";
    tr->write_raw(os.str());
}

void trace_postprocess(TraceWriter* tr, int task_index, const std::string& stage,
                       const std::vector<Cell>& before, const std::vector<Cell>& after,
                       int min_run = 0) {
    if (!tr || !tr->enabled() || !tr->opt.include_postprocess) return;
    tr->task_scope(task_index);
    tr->write_raw("{\"type\":\"postprocess\",\"task\":" + std::to_string(task_index) +
                  ",\"stage\":\"" + json_escape(stage) + "\","
                  "\"min_run_cells\":" + std::to_string(min_run) + ","
                  "\"before_points\":" + std::to_string(before.size()) + ","
                  "\"after_points\":" + std::to_string(after.size()) + ","
                  "\"before_turns\":" + std::to_string(count_turns(before)) + ","
                  "\"after_turns\":" + std::to_string(count_turns(after)) + "}");
}

char* dup_string(const std::string& s) {
    char* p = static_cast<char*>(std::malloc(s.size() + 1));
    if (!p) return nullptr;
    std::memcpy(p, s.c_str(), s.size() + 1);
    return p;
}

std::optional<std::string> opt_str(const char* s) {
    if (!s) return std::nullopt;
    return std::string(s);
}

SceneResult to_scene_result(const AStarResult& r) {
    SceneResult s;
    s.success = r.success;
    s.length_mm = r.length_mm;
    s.cost_mm = r.cost_mm;
    s.turns = r.turns;
    s.expanded_nodes = r.expanded_nodes;
    s.elapsed_ms = r.elapsed_ms;
    s.fail = static_cast<int>(r.fail);
    if (r.success) s.path = r.path;
    if (!r.visited.empty()) s.visited = r.visited;
    return s;
}

void fill_result(R3dResult& o, const std::optional<SceneResult>& r) {
    o = R3dResult{};
    if (!r) return;
    o.success = r->success ? 1 : 0;
    o.length_mm = r->length_mm;
    o.cost_mm = r->cost_mm;
    o.turns = r->turns;
    o.expanded_nodes = r->expanded_nodes;
    o.elapsed_ms = r->elapsed_ms;
    o.path_len = r->path ? static_cast<int32_t>(r->path->size()) : 0;
    o.visited_len = r->visited ? static_cast<int32_t>(r->visited->size()) : 0;
    o.fail_reason = static_cast<int32_t>(r->fail);
}

ImplicitOccupancy implicit_from_doc(const SceneDoc& doc) {
    ImplicitOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;
        }
    }
    return occ;
}

#ifdef ROUTING3D_USE_OPENVDB
// OpenVDB production occupancy. It keeps large filled regions tile-compressed while exposing
// the same query contract used by Dense/Implicit routing algorithms.
VdbOccupancy vdb_from_doc(const SceneDoc& doc) {
    VdbOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;
        }
    }
    return occ;
}
#endif
ImplicitOccupancy coarse_implicit_from_doc(const SceneDoc& doc, int factor) {
    Cell cs{(doc.shape.i + factor - 1) / factor, (doc.shape.j + factor - 1) / factor,
            (doc.shape.k + factor - 1) / factor};
    ImplicitOccupancy occ(cs, doc.origin, doc.cell_mm * factor);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;
        }
    }
    return occ;
}

template <class Occ>
bool route_hier(const Occ& work, const ImplicitOccupancy& coarse, int factor, int radius,
                Cell s, Cell g, const RouteParams& params, long long max_exp,
                bool collect_visited, AStarResult& out) {
    auto to_coarse = [factor](const Cell& c) {
        return Cell{c.i / factor, c.j / factor, c.k / factor};
    };
    Cell cs0 = to_coarse(s), cg0 = to_coarse(g);
    Cell cs = snap_to_free_cell(coarse, cs0, 4);
    Cell cgl = snap_to_free_cell(coarse, cg0, 4);
    RouteParams cp = params;
    cp.cell_mm = coarse.cell_mm();
    // 적응형 해상도 핵심(Tier-3 Stage 1): coarse 골격은 '최적(표준 A*)'으로 푼다. coarse 격자는 fine 의
    //   1/factor³(예 8³=512배 작음 → 수십만 셀)이라 w_heur=1.0(최적)·동적가중 OFF 로도 저렴하고, 이 골격이
    //   fine corridor 의 길잡이가 되므로 골격이 최적이어야 fine 경로가 짧다. 정적 greedy(w=2.0) coarse 는
    //   골격 자체가 우회해 fine 이 그 튜브에 갇혀 3~4× 우회의 주원인이었다. min_straight 도 coarse 엔 불필요
    //   (fine 에서 강제) → 상태폭발 방지.
    cp.w_heur = 1.0;
    cp.w_heur_near = 0.0;
    cp.min_straight_cells = 0;
    AStarResult cg = astar_weighted(coarse, cs, cgl, cp, 2000000LL, false);
    if (!cg.success || cg.path.empty()) return false;

    // 적응형 corridor 확장 재시도(Tier-3 Stage 2): 실패 시 반경 r 을 ×2 로 키워 최대 3회 재시도 → 좁은
    //   튜브 미스로 전체탐색 fall-through(메모리 폭발·상한 실패) 대신 넓힌 튜브에서 재탐색(메모리 한계
    //   실패 직접 감소). 탐색은 항상 corridor 로 바운드되므로 최종 실패해도 메모리 안전.
    for (int attempt = 0, r = radius; attempt < 3; ++attempt, r *= 2) {
    auto corr = std::make_shared<std::unordered_set<uint64_t>>();
    corr->reserve(cg.path.size() * static_cast<size_t>((2 * r + 1) * (2 * r + 1) * (2 * r + 1)) + 64);
    auto add_dilated = [&](const Cell& c) {
        for (int di = -r; di <= r; ++di)
            for (int dj = -r; dj <= r; ++dj)
                for (int dk = -r; dk <= r; ++dk)
                    corr->insert(pack20(Cell{c.i + di, c.j + dj, c.k + dk}));
    };
    for (const Cell& c : cg.path) add_dilated(c);
    auto add_box = [&](const Cell& a, const Cell& b) {
        int i0 = std::min(a.i, b.i) - r, i1 = std::max(a.i, b.i) + r;
        int j0 = std::min(a.j, b.j) - r, j1 = std::max(a.j, b.j) + r;
        int k0 = std::min(a.k, b.k) - r, k1 = std::max(a.k, b.k) + r;
        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                for (int k = k0; k <= k1; ++k) corr->insert(pack20(Cell{i, j, k}));
    };
    add_box(cs0, cs);
    add_box(cg0, cgl);

    auto in_corr = [corr, factor](const Cell& fc) {
        return corr->count(pack20(Cell{fc.i / factor, fc.j / factor, fc.k / factor})) > 0;
    };
    AStarResult fr = astar_weighted(work, s, g, params, max_exp, collect_visited,
                                    nullptr, nullptr, 0, in_corr);
    if (fr.success && !fr.path.empty()) { out = std::move(fr); return true; }
        // 예산 소진(ExpansionLimit) 실패는 튜브를 넓혀도 탐색만 더 폭발 → 재시도 무의미(중단). '튜브가 좁아
        //   경로 없음'(NoPath)·'시작/목표가 튜브 밖'(CorridorMiss)일 때만 반경을 키워 재시도한다.
        if (fr.fail == RouteFail::ExpansionLimit) break;
    }   // 적응형 corridor 확장 재시도 루프 끝(반경 ×2).
    return false;   // 넓힌 튜브로도 실패 → 호출자가 fb_exp 전체탐색 폴백(드묾).
}

//   ?몄옄: phase, order_index, task_index, success, length_mm, turns, expanded_nodes, elapsed_ms,
using ProgressCb = std::function<int(int, int, int, bool, double, int, long long, double, int, int,
                                     double, const std::vector<Cell>*)>;

inline std::vector<Cell> walk_order(Cell A, Cell B, const int (&axisOrder)[3]) {
    std::vector<Cell> v;
    v.push_back(A);
    Cell c = A;
    for (int oi = 0; oi < 3; ++oi) {
        int ax = axisOrder[oi];
        int target = (ax == 0) ? B.i : (ax == 1) ? B.j : B.k;
        while (true) {
            int cur = (ax == 0) ? c.i : (ax == 1) ? c.j : c.k;
            if (cur == target) break;
            int s = (target > cur) ? 1 : -1;
            if (ax == 0) c.i += s; else if (ax == 1) c.j += s; else c.k += s;
            v.push_back(c);
        }
    }
    return v;
}

template <class Occ>
bool ortho_connect(const Occ& occ, Cell A, Cell B, std::vector<Cell>& out) {
    int axes = (A.i != B.i ? 1 : 0) + (A.j != B.j ? 1 : 0) + (A.k != B.k ? 1 : 0);
    if (axes > 3) return false;
    static const int orders[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    bool found = false;
    int bestTurns = std::numeric_limits<int>::max();
    int bestSteps = std::numeric_limits<int>::max();
    for (const auto& ord : orders) {
        std::vector<Cell> v = walk_order(A, B, ord);
        bool clear = true;
        for (const Cell& c : v) if (occ.is_blocked(c)) { clear = false; break; }
        if (!clear) continue;
        const int turns = count_turns(v);
        const int steps = static_cast<int>(v.size()) - 1;
        if (!found || turns < bestTurns || (turns == bestTurns && steps < bestSteps)) {
            out = std::move(v);
            bestTurns = turns;
            bestSteps = steps;
            found = true;
        }
    }
    return found;
}

template <class Occ>
std::vector<Cell> unkink_path(const Occ& occ, const std::vector<Cell>& path) {
    const int n = static_cast<int>(path.size());
    if (n < 4) return path;
    std::vector<Cell> out;
    out.reserve(static_cast<size_t>(n));
    out.push_back(path[0]);
    int a = 0;
    while (a < n - 1) {
        int best = -1;
        std::vector<Cell> bestSeg;
        for (int j = n - 1; j >= a + 2; --j) {
            std::vector<Cell> seg;
            if (!ortho_connect(occ, path[static_cast<size_t>(a)], path[static_cast<size_t>(j)], seg))
                continue;
            const int segSteps = static_cast<int>(seg.size()) - 1, origSteps = j - a;
            if (segSteps > origSteps) continue;
            if (segSteps == origSteps) {
                std::vector<Cell> slice(path.begin() + a, path.begin() + j + 1);
                if (count_turns(seg) >= count_turns(slice)) continue;
            }
            best = j;
            bestSeg = std::move(seg);
            break;
        }
        if (best < 0) { out.push_back(path[static_cast<size_t>(a + 1)]); a = a + 1; }
        else { for (size_t s = 1; s < bestSeg.size(); ++s) out.push_back(bestSeg[s]); a = best; }
    }
    return out;
}

template <class Occ>
std::vector<Cell> enforce_min_straight(const Occ& occ, const std::vector<Cell>& path, int min_run_cells) {
    if (min_run_cells <= 1 || path.size() < 5) return path;
    std::vector<Cell> cur = path;
    bool changed = true;
    int guard = 0;
    while (changed && guard++ < 64) {
        changed = false;
        std::vector<int> corners;
        corners.push_back(0);
        for (size_t m = 1; m + 1 < cur.size(); ++m) {
            Cell d0{cur[m].i - cur[m - 1].i, cur[m].j - cur[m - 1].j, cur[m].k - cur[m - 1].k};
            Cell d1{cur[m + 1].i - cur[m].i, cur[m + 1].j - cur[m].j, cur[m + 1].k - cur[m].k};
            if (!(d0 == d1)) corners.push_back(static_cast<int>(m));
        }
        corners.push_back(static_cast<int>(cur.size()) - 1);
        for (size_t ci = 1; ci + 1 < corners.size(); ++ci) {
            const int runNext = corners[ci + 1] - corners[ci];      // ci~ci+1 ???? ??.
            const int runPrev = corners[ci] - corners[ci - 1];      // ci-1~ci ???? ??.
            if (runNext >= min_run_cells && runPrev >= min_run_cells) continue;
            const int a = corners[ci - 1], b = corners[ci + 1];
            std::vector<Cell> seg;
            if (!ortho_connect(occ, cur[static_cast<size_t>(a)], cur[static_cast<size_t>(b)], seg))
                continue;
            std::vector<Cell> slice(cur.begin() + a, cur.begin() + b + 1);
            if (count_turns(seg) > count_turns(slice)) continue;
            if (static_cast<int>(seg.size()) - 1 > b - a) continue;  // 湲몄??利앷? 湲덉?.
            std::vector<Cell> next(cur.begin(), cur.begin() + a);    // [0..a) + seg + (b..end].
            for (const Cell& c : seg) next.push_back(c);
            for (size_t t = static_cast<size_t>(b) + 1; t < cur.size(); ++t) next.push_back(cur[t]);
            cur = std::move(next);
            changed = true;
            break;
        }
    }
    return cur;
}

// Shared multi-pipe routing pipeline for Dense, Implicit, and OpenVDB occupancy backends.
template <class Occ>
void route_multi_impl(SceneDoc& doc, Occ occ, const std::string& priority, bool collect_visited,
                      const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr,
                      int pipe_radius = 0, bool per_task_radius = false,
                      int cbs_depth = 0, double min_straight_mult = 0.0,
                      double pipe_gap_mm = 0.0,
                      const R3dRuntimeOptions* runtime = nullptr,
                      TraceWriter* trace = nullptr) {
    const long long large_threshold = runtime ? opt_or_default(runtime->large_grid_threshold, 5000000LL) : 5000000LL;
    const long long configured_max_exp = runtime ? runtime->max_expansions : 0;
    const long long configured_fallback_exp = runtime ? runtime->fallback_expansions : 0;
    const int configured_hier_factor = runtime ? runtime->hier_factor : 0;
    const int configured_hier_radius = runtime ? runtime->hier_radius : 0;
    const long long configured_hier_probe = runtime ? runtime->hier_probe : 0;
    const int configured_ripup_enabled = runtime ? runtime->ripup_enabled : -1;
    const long long configured_cbs_exp = runtime ? runtime->cbs_expansions : 0;

    Occ work = occ.copy();
    trace_header(trace, doc, priority);
    trace_static_occupancy_samples(trace, doc);
    trace_occupancy_summary(trace, work);
    int eff_cbs_depth = cbs_depth < 0 ? 0 : cbs_depth;
    if (const char* cs = std::getenv("R3D_CBS")) {
        char* end = nullptr; long v = std::strtol(cs, &end, 10);
        if (end != cs && v >= 0) eff_cbs_depth = static_cast<int>(v);
    }
    if (eff_cbs_depth > 3) eff_cbs_depth = 3;
    double eff_min_straight = min_straight_mult < 0.0 ? 0.0 : min_straight_mult;
    if (const char* ms = std::getenv("R3D_MIN_STRAIGHT")) {
        char* end = nullptr; double v = std::strtod(ms, &end);
        if (end != ms && v >= 0.0) eff_min_straight = v;
    }
    int eff_pipe_radius = pipe_radius < 0 ? 0 : pipe_radius;
    if (const char* pr = std::getenv("R3D_PIPE_RADIUS")) {
        char* end = nullptr;
        long v = std::strtol(pr, &end, 10);
        if (end != pr && v >= 0) eff_pipe_radius = static_cast<int>(v);
    }
    bool eff_per_task = per_task_radius;
    if (const char* pt = std::getenv("R3D_PER_TASK_RADIUS"))
        eff_per_task = !(pt[0] == '0' || pt[0] == '\0');
    const int PIPE_RADIUS_MAX = 8;
    const double cell_for_r = doc.params.cell_mm;
    auto radius_of = [&](int task_idx) -> int {
        if (eff_per_task && task_idx >= 0 && task_idx < static_cast<int>(doc.tasks.size())) {
            double d = doc.tasks[static_cast<size_t>(task_idx)].diameter_mm;
            if (d > 0.0 && cell_for_r > 0.0) {
                int r = static_cast<int>(std::ceil(d / cell_for_r)) - 1;
                if (r < 0) r = 0; if (r > PIPE_RADIUS_MAX) r = PIPE_RADIUS_MAX;
                return r;
            }
        }
        return eff_pipe_radius;
    };
    double eff_gap_mm = pipe_gap_mm < 0.0 ? 0.0 : pipe_gap_mm;
    if (const char* pg = std::getenv("R3D_PIPE_GAP")) {
        char* end = nullptr; double v = std::strtod(pg, &end);
        if (end != pg && v >= 0.0) eff_gap_mm = v;
    }
    const bool use_gap = eff_gap_mm > 0.0 && cell_for_r > 0.0;
    const int PAIR_RADIUS_MAX = 24;
    auto rmm_of = [&](int ti) -> double {
        if (eff_per_task && ti >= 0 && ti < static_cast<int>(doc.tasks.size())) {
            double d = doc.tasks[static_cast<size_t>(ti)].diameter_mm;
            if (d > 0.0) return d * 0.5;
        }
        return eff_pipe_radius * cell_for_r;
    };
    auto pair_radius = [&](int a, int b) -> int {
        double sep = rmm_of(a) + rmm_of(b) + eff_gap_mm;
        int r = static_cast<int>(std::ceil(sep / cell_for_r));
        if (r < 0) r = 0; if (r > PAIR_RADIUS_MAX) r = PAIR_RADIUS_MAX;
        return r;
    };
    auto params_for = [&](int task_idx) -> RouteParams {
        RouteParams tp = doc.params;
        if (eff_per_task && tp.w_clear > 0.0) {
            int r = radius_of(task_idx);
            if (r > tp.clearance_radius) tp.clearance_radius = r;
        }
        return tp;
    };

    const int n = static_cast<int>(doc.tasks.size());
    std::vector<int> order(static_cast<size_t>(n));
    std::iota(order.begin(), order.end(), 0);

    auto dist = [&](int t) {
        return manhattan(work.to_cell(doc.tasks[static_cast<size_t>(t)].start_mm),
                         work.to_cell(doc.tasks[static_cast<size_t>(t)].end_mm));
    };
    auto dia = [&](int t) { return doc.tasks[static_cast<size_t>(t)].diameter_mm; };
    if (priority == "original") {
    } else if (priority == "shortest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) < dist(b); });
    } else if (priority == "longest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) > dist(b); });
    } else if (priority == "diameter") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            if (dia(a) != dia(b)) return dia(a) > dia(b);
            return dist(a) > dist(b);
        });
    } else if (priority == "utility") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            const std::string la = doc.tasks[static_cast<size_t>(a)].utility_label();
            const std::string lb = doc.tasks[static_cast<size_t>(b)].utility_label();
            if (la != lb) return la < lb;
            if (dia(a) != dia(b)) return dia(a) > dia(b);
            return dist(a) > dist(b);
        });
    } else {
        throw std::invalid_argument("unknown priority: " + priority);
    }

    doc.results.assign(static_cast<size_t>(n), std::nullopt);
    std::unordered_set<long long> corridor;
    const bool use_corridor = doc.params.w_corridor > 0.0;
    const int corridor_radius = doc.params.corridor_radius > 0 ? doc.params.corridor_radius : 1;
    if (use_corridor && seed) {
        for (const Cell& c : *seed)
            if (work.in_bounds(c)) corridor.insert(static_cast<long long>(work.lin(c)));
    }
    const long long max_exp = (occ.size() > large_threshold)
        ? opt_or_default(configured_max_exp, large_grid_cap()) : -1;
    long long eff_cbs_exp = configured_cbs_exp > 0 ? configured_cbs_exp
        : ((max_exp > 0) ? max_exp : 2000000LL);
    if (const char* ce = std::getenv("R3D_CBS_EXP")) {
        char* end = nullptr;
        long long v = std::strtoll(ce, &end, 10);
        if (end != ce && v > 0) eff_cbs_exp = v;
    }
    const long long progress_every = on_pipe ? 50000LL : 0;
    const bool large_grid = occ.size() > large_threshold;
    const bool use_hier = large_grid;
    // 적응형 coarse factor(Tier-3 Stage 3b): 미설정이면 coarse 셀이 ~200mm 가 되도록 fine 셀 크기에 맞춰
    //   factor 를 정한다(목표 200mm/cell, [4,32] 클램프). 이러면 coarse 격자 셀 수가 fine 해상도와 무관하게
    //   거의 일정 → coarse 골격 솔브 비용이 10mm 같은 초미세격자에서도 폭발하지 않는다. cell=25 → factor 8
    //   (기존과 동일), cell=10 → 20, cell=50 → 4. 명시 설정(configured>0)이면 그대로.
    int adaptive_factor = 8;
    {
        const double cm = doc.params.cell_mm;
        if (cm > 0.0) {
            int f = static_cast<int>(200.0 / cm + 0.5);
            adaptive_factor = f < 4 ? 4 : (f > 32 ? 32 : f);
        }
    }
    const int HIER_FACTOR = opt_or_default_i(configured_hier_factor, adaptive_factor);
    const int HIER_RADIUS = opt_or_default_i(configured_hier_radius, 2);
    const long long HIER_PROBE = opt_or_default(configured_hier_probe, 300000LL);
    long long fallback_exp = configured_fallback_exp > 0 ? configured_fallback_exp : max_exp;
    if (const char* fe = std::getenv("R3D_FALLBACK_EXP")) {
        char* end = nullptr; long long v = std::strtoll(fe, &end, 10);
        if (end != fe && v > 0) fallback_exp = (max_exp > 0) ? std::min(v, max_exp) : v;
    }
    std::optional<ImplicitOccupancy> coarse;
    int done = 0;
    bool aborted = false;
    std::map<int, std::vector<Cell>> placed;
    for (int oidx = 0; oidx < static_cast<int>(order.size()); ++oidx) {
        const int oi = order[static_cast<size_t>(oidx)];
        const RouteTask& t = doc.tasks[static_cast<size_t>(oi)];
        if (use_gap) {
            work = occ.copy();
            for (const auto& kv : placed) mark_pipe(work, kv.second, pair_radius(kv.first, oi));
        }
        const int snap_r = use_gap ? 2 + pair_radius(oi, oi) : 2 + radius_of(oi);
        Cell raw_s = work.to_cell(t.start_mm);
        Cell raw_g = work.to_cell(t.end_mm);
        Cell s = snap_to_free_cell(work, raw_s, snap_r);
        Cell g = snap_to_free_cell(work, raw_g, snap_r);
        trace_task_begin(trace, oidx, oi, t, raw_s, raw_g, s, g);
        const RouteParams tp = params_for(oi);

        std::function<bool(long long, double)> intra;
        if (on_pipe) {
            intra = [&](long long expanded, double prog) -> bool {
                trace_expand(trace, oidx, oi, expanded, prog);
                if (on_pipe(0, oidx, oi, false, 0.0, 0, expanded, 0.0, done, n, prog, nullptr) != 0)
                    aborted = true;
                return aborted;
            };
        } else if (trace && trace->enabled()) {
            intra = [&](long long expanded, double prog) -> bool {
                trace_expand(trace, oidx, oi, expanded, prog);
                return aborted;
            };
        }
        AStarTraceFn astar_trace;
        if (trace && trace->enabled() && trace->opt.level >= 1) {
            astar_trace = [trace, oidx, oi](const char* event, const Cell& from, const Cell& to,
                                            long long expanded, int dir, int run, int required) {
                trace_search_cell(trace, oidx, oi, event, from, to, expanded, dir, run, required);
            };
        }
        AStarResult res;
        bool routed = false;
        long long fb_exp = max_exp;
        if (use_hier) {
            const long long probe = (max_exp > 0) ? std::min(HIER_PROBE, max_exp) : HIER_PROBE;
            res = astar_weighted(work, s, g, tp, probe, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 intra ? &intra : nullptr, progress_every, AllowAll{}, t.goal_dir,
                                 astar_trace ? &astar_trace : nullptr);
            if (res.success && !res.path.empty()) {
                routed = true;
            } else if (res.expanded_nodes >= probe) {
                if (!coarse) coarse.emplace(coarse_implicit_from_doc(doc, HIER_FACTOR));
                if (route_hier(work, *coarse, HIER_FACTOR, HIER_RADIUS, s, g, tp,
                               max_exp, collect_visited, res))
                    routed = true;
                else
                    fb_exp = fallback_exp;
            } else {
                routed = true;
            }
        }
        if (!routed)
            res = astar_weighted(work, s, g, tp, fb_exp, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 intra ? &intra : nullptr, progress_every, AllowAll{}, t.goal_dir,
                                 astar_trace ? &astar_trace : nullptr);
        bool ok = res.success && !res.path.empty();
        if (!ok && t.goal_dir >= 0 && !aborted) {
            res = astar_weighted(work, s, g, tp, fb_exp, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 intra ? &intra : nullptr, progress_every, AllowAll{}, -1,
                                 astar_trace ? &astar_trace : nullptr);
            ok = res.success && !res.path.empty();
        }
        std::vector<Cell> path = res.path;
        if (ok && doc.params.w_heur > 1.0 && path.size() >= 4) {
            std::vector<Cell> up = unkink_path(work, path);
            if (up.size() < path.size()) {
                trace_postprocess(trace, oi, "unkink", path, up);
                path = std::move(up);
                res.path = path;
                res.length_mm = (path.size() - 1) * doc.params.cell_mm;
                res.turns = count_turns(path);
            }
        }
        doc.results[static_cast<size_t>(oi)] = to_scene_result(res);
        if (ok) {
            mark_pipe(work, path, radius_of(oi));
            if (trace && trace->enabled()) {
                trace->task_scope(oi);
                trace->write_raw("{\"type\":\"route_mark\",\"task\":" + std::to_string(oi) +
                                 ",\"path_points\":" + std::to_string(path.size()) +
                                 ",\"radius_cells\":" + std::to_string(radius_of(oi)) + "}");
                trace->write_raw("{\"type\":\"route_path\",\"task\":" + std::to_string(oi) +
                                 ",\"path_points\":" + std::to_string(path.size()) +
                                 ",\"cells\":" + cell_array_json(path) + "}");
            }
            if (use_corridor) add_corridor_cells(work, corridor, path, corridor_radius);
            placed[oi] = path;
        }
        trace_task_end(trace, oidx, oi, res, ok, aborted);
        ++done;
        if (on_pipe) {
            if (on_pipe(1, oidx, oi, ok, res.length_mm, res.turns, res.expanded_nodes, res.elapsed_ms,
                        done, n, 1.0, ok ? &path : nullptr) != 0)
                aborted = true;
        }
        if (aborted) break;
    }

    bool ripup_on = configured_ripup_enabled >= 0 ? (configured_ripup_enabled != 0) : true;
    if (configured_ripup_enabled < 0)
        if (const char* rs = std::getenv("R3D_RIPUP")) ripup_on = !(rs[0] == 'o' && rs[1] == 'f');
    auto has_fail = [&]() {
        for (int i = 0; i < n; ++i)
            if (!doc.results[static_cast<size_t>(i)] || !doc.results[static_cast<size_t>(i)]->success)
                return true;
        return false;
    };
    if (ripup_on && !aborted && large_grid && has_fail()) {
        const int RIPUP_ROUNDS = 6, MAX_RIPUP = 4;
        auto pack = [](const Cell& c) -> uint64_t {
            return (static_cast<uint64_t>(static_cast<uint32_t>(c.i)) << 42) |
                   (static_cast<uint64_t>(static_cast<uint32_t>(c.j)) << 21) |
                   static_cast<uint64_t>(static_cast<uint32_t>(c.k));
        };
        auto route_on = [&](const Occ& w, int ti) -> AStarResult {
            const RouteTask& tt = doc.tasks[static_cast<size_t>(ti)];
            const int snap_r = 2 + radius_of(ti);
            const RouteParams tpr = params_for(ti);
            Cell ss = snap_to_free_cell(w, w.to_cell(tt.start_mm), snap_r);
            Cell gg = snap_to_free_cell(w, w.to_cell(tt.end_mm), snap_r);
            AStarResult r = astar_weighted(w, ss, gg, tpr, max_exp, false,
                                           nullptr, nullptr, 0, AllowAll{}, tt.goal_dir);
            if ((!r.success || r.path.empty()) && tt.goal_dir >= 0)
                r = astar_weighted(w, ss, gg, tpr, max_exp, false);
            return r;
        };
        auto build_work = [&](const std::map<int, std::vector<Cell>>& paths) -> Occ {
            Occ w = occ.copy();
            for (const auto& kv : paths) mark_pipe(w, kv.second, radius_of(kv.first));
            return w;
        };
        for (int round = 0; round < RIPUP_ROUNDS; ++round) {
            std::vector<int> failed;
            for (int i = 0; i < n; ++i)
                if (!doc.results[static_cast<size_t>(i)] || !doc.results[static_cast<size_t>(i)]->success)
                    failed.push_back(i);
            if (failed.empty()) break;
            bool changed = false;
            for (int f : failed) {
                AStarResult ideal = route_on(occ, f);
                if (!(ideal.success && !ideal.path.empty())) continue;
                std::unordered_set<uint64_t> cs;
                cs.reserve(ideal.path.size() * 2);
                for (const Cell& c : ideal.path) cs.insert(pack(c));
                std::vector<int> blockers;
                for (const auto& kv : placed) {
                    for (const Cell& c : kv.second)
                        if (cs.count(pack(c))) { blockers.push_back(kv.first); break; }
                }
                if (blockers.empty() || static_cast<int>(blockers.size()) > MAX_RIPUP) continue;
                std::map<int, std::vector<Cell>> trial = placed;
                for (int b : blockers) trial.erase(b);
                Occ wt = build_work(trial);
                AStarResult rf = route_on(wt, f);
                if (!(rf.success && !rf.path.empty())) continue;
                mark_pipe(wt, rf.path, radius_of(f));
                trial[f] = rf.path;
                std::vector<AStarResult> rbs(blockers.size());
                bool all_ok = true;
                for (size_t bi = 0; bi < blockers.size(); ++bi) {
                    AStarResult rb = route_on(wt, blockers[bi]);   // ??? blocker ????고똿.
                    if (rb.success && !rb.path.empty()) {
                        mark_pipe(wt, rb.path, radius_of(blockers[bi]));
                        trial[blockers[bi]] = rb.path;
                    } else {
                        all_ok = false;
                    }
                    rbs[bi] = std::move(rb);
                }
                if (!all_ok) continue;
                placed = std::move(trial);
                doc.results[static_cast<size_t>(f)] = to_scene_result(rf);
                for (size_t bi = 0; bi < blockers.size(); ++bi)
                    doc.results[static_cast<size_t>(blockers[bi])] = to_scene_result(rbs[bi]);
                changed = true;
                if (on_pipe)
                    on_pipe(1, -1, f, true, rf.length_mm, rf.turns, rf.expanded_nodes, rf.elapsed_ms,
                            done, n, 1.0, &placed[f]);
            }
            if (!changed) break;
        }
    }

    // CBS-lite: recursively move blockers when a failed route can be recovered by negotiation.
    if (eff_cbs_depth > 0 && !aborted && has_fail()) {
        const int CBS_MAXBLK = 4;
        auto pack = [](const Cell& c) -> uint64_t {
            return (static_cast<uint64_t>(static_cast<uint32_t>(c.i)) << 42) |
                   (static_cast<uint64_t>(static_cast<uint32_t>(c.j)) << 21) |
                   static_cast<uint64_t>(static_cast<uint32_t>(c.k));
        };
        auto route_on = [&](const Occ& w, int ti) -> AStarResult {
            const RouteTask& tt = doc.tasks[static_cast<size_t>(ti)];
            const int snap_r = 2 + radius_of(ti);
            const RouteParams tpr = params_for(ti);
            Cell ss = snap_to_free_cell(w, w.to_cell(tt.start_mm), snap_r);
            Cell gg = snap_to_free_cell(w, w.to_cell(tt.end_mm), snap_r);
            AStarResult r = astar_weighted(w, ss, gg, tpr, eff_cbs_exp, false,
                                           nullptr, nullptr, 0, AllowAll{}, tt.goal_dir);
            if ((!r.success || r.path.empty()) && tt.goal_dir >= 0)
                r = astar_weighted(w, ss, gg, tpr, eff_cbs_exp, false);
            return r;
        };
        auto build_work = [&](const std::map<int, std::vector<Cell>>& paths) -> Occ {
            Occ w = occ.copy();
            for (const auto& kv : paths) mark_pipe(w, kv.second, radius_of(kv.first));
            return w;
        };

        std::function<bool(int, const std::map<int, std::vector<Cell>>&, int,
                           std::map<int, std::vector<Cell>>&,
                           std::map<int, AStarResult>&)> resolve;
        resolve = [&](int target, const std::map<int, std::vector<Cell>>& state, int depth,
                      std::map<int, std::vector<Cell>>& out,
                      std::map<int, AStarResult>& out_results) -> bool {
            AStarResult ideal = route_on(occ, target);
            if (!(ideal.success && !ideal.path.empty())) return false;
            std::unordered_set<uint64_t> cs;
            cs.reserve(ideal.path.size() * 2);
            for (const Cell& c : ideal.path) cs.insert(pack(c));
            std::vector<int> blockers;
            for (const auto& kv : state) {
                if (kv.first == target) continue;
                for (const Cell& c : kv.second)
                    if (cs.count(pack(c))) { blockers.push_back(kv.first); break; }
            }
            if (blockers.empty() || static_cast<int>(blockers.size()) > CBS_MAXBLK) return false;

            std::map<int, std::vector<Cell>> trial = state;
            for (int b : blockers) trial.erase(b);
            AStarResult rf = route_on(build_work(trial), target);
            if (!(rf.success && !rf.path.empty())) return false;
            trial[target] = rf.path;
            out_results[target] = rf;

            for (int b : blockers) {
                AStarResult rb = route_on(build_work(trial), b);
                if (rb.success && !rb.path.empty()) {
                    trial[b] = rb.path;
                    out_results[b] = rb;
                    continue;
                }
                if (depth <= 0) return false;
                std::map<int, std::vector<Cell>> sub;
                std::map<int, AStarResult> sub_results;
                if (!resolve(b, trial, depth - 1, sub, sub_results)) return false;
                trial = std::move(sub);
                out_results.insert(sub_results.begin(), sub_results.end());
            }
            out = std::move(trial);
            return true;
        };

        for (int round = 0; round < 4; ++round) {
            std::vector<int> failed;
            for (int i = 0; i < n; ++i)
                if (!doc.results[static_cast<size_t>(i)] || !doc.results[static_cast<size_t>(i)]->success)
                    failed.push_back(i);
            if (failed.empty()) break;
            bool changed = false;
            for (int f : failed) {
                std::map<int, std::vector<Cell>> out;
                std::map<int, AStarResult> out_results;
                if (!resolve(f, placed, eff_cbs_depth, out, out_results)) continue;
                for (const auto& kv : out) {
                    auto it = placed.find(kv.first);
                    if (it == placed.end() || it->second != kv.second) {
                        auto ri = out_results.find(kv.first);
                        if (ri != out_results.end())
                            doc.results[static_cast<size_t>(kv.first)] = to_scene_result(ri->second);
                    }
                }
                placed = std::move(out);
                changed = true;
                if (on_pipe) {
                    auto ri = out_results.find(f);
                    const AStarResult* rr = (ri != out_results.end()) ? &ri->second : nullptr;
                    on_pipe(1, -1, f, true,
                            rr ? rr->length_mm : (placed[f].size() - 1) * doc.params.cell_mm,
                            rr ? rr->turns : count_turns(placed[f]),
                            rr ? rr->expanded_nodes : 0,
                            rr ? rr->elapsed_ms : 0.0,
                            done, n, 1.0, &placed[f]);
                }
            }
            if (!changed) break;
        }
    }
    const int abs_min_run = doc.params.min_straight_cells > 1 ? doc.params.min_straight_cells : 0;
    if ((eff_min_straight > 0.0 || abs_min_run > 1) && cell_for_r > 0.0 && !aborted && !placed.empty()) {
        for (auto& kv : placed) {
            const int pi = kv.first;
            std::vector<Cell>& path = kv.second;
            const double d = doc.tasks[static_cast<size_t>(pi)].diameter_mm;
            const int dia_min_run = (eff_min_straight > 0.0 && d > 0.0)
                ? static_cast<int>(std::ceil(eff_min_straight * d / cell_for_r)) : 0;
            const int min_run = std::max(abs_min_run, dia_min_run);
            if (min_run <= 1 || path.size() < 5) continue;
            Occ chk = occ.copy();
            for (const auto& other : placed)
                if (other.first != pi) mark_pipe(chk, other.second, radius_of(other.first));
            std::vector<Cell> sp = unkink_path(chk, path);
            sp = enforce_min_straight(chk, sp, min_run);
            if (sp.size() < path.size() ||
                (sp.size() == path.size() && count_turns(sp) < count_turns(path))) {
                if (count_turns(sp) <= count_turns(path)) {
                    trace_postprocess(trace, pi, "min_straight", path, sp, min_run);
                    AStarResult nr;
                    nr.success = true; nr.path = sp;
                    nr.length_mm = (sp.size() - 1) * doc.params.cell_mm;
                    nr.turns = count_turns(sp);
                    doc.results[static_cast<size_t>(pi)] = to_scene_result(nr);
                    path = std::move(sp);
                    if (on_pipe)
                        on_pipe(1, -1, pi, true, (path.size() - 1) * doc.params.cell_mm,
                                count_turns(path), 0, 0.0, done, n, 1.0, &path);
                }
            }
        }
    }
    if (trace && trace->enabled()) {
        trace->current_task = -1;
        trace->write_raw("{\"type\":\"trace_end\",\"task_count\":" + std::to_string(n) +
                         ",\"aborted\":" + std::string(aborted ? "true" : "false") + "}");
        trace->flush();
    }
}

void route_multi_into_doc(SceneDoc& doc, const std::string& priority, bool collect_visited,
                          const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr,
                          int pipe_radius = 0, bool per_task_radius = false,
                          int cbs_depth = 0, double min_straight_mult = 0.0,
                          double pipe_gap_mm = 0.0,
                          const R3dRuntimeOptions* runtime = nullptr,
                          TraceWriter* trace = nullptr) {
#ifdef ROUTING3D_USE_OPENVDB
    route_multi_impl(doc, vdb_from_doc(doc), priority, collect_visited, on_pipe, seed,
                     pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm, runtime, trace);
#else
    const long long cells =
        static_cast<long long>(doc.shape.i) * doc.shape.j * doc.shape.k;
    const long long large_threshold = runtime ? opt_or_default(runtime->large_grid_threshold, 5000000LL) : 5000000LL;
    if (cells > large_threshold) {
        route_multi_impl(doc, implicit_from_doc(doc), priority, collect_visited, on_pipe, seed,
                         pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm, runtime, trace);
    } else {
        route_multi_impl(doc, occupancy_from_doc(doc), priority, collect_visited, on_pipe, seed,
                         pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm, runtime, trace);
    }
#endif
}

}  // namespace

// ============================================================================
extern "C" const char* r3d_version(void) {
    return "routing3d_capi 0.1 (engine Phase 3)";
}

extern "C" void r3d_free_string(char* s) {
    std::free(s);
}

// ============================================================================ Level 1
extern "C" R3dStatus r3d_route_scene_text(const char* scene_text, const char* mode,
                                          const char* priority, char** out_scene_text) {
    if (!scene_text || !out_scene_text) return R3D_ERR_ARG;
    *out_scene_text = nullptr;
    SceneDoc doc;
    try {
        doc = loads_scene(scene_text);
    } catch (...) {
        return R3D_ERR_PARSE;
    }
    try {
        const std::string m = mode ? mode : "multi";
        if (m == "single") {
            doc.results.assign(doc.tasks.size(), std::nullopt);
#ifdef ROUTING3D_USE_OPENVDB
            VdbOccupancy occ = vdb_from_doc(doc);
            const long long max_exp = occ.size() > 5000000LL ? large_grid_cap() : -1;
#else
            ImplicitOccupancy occ = implicit_from_doc(doc);
            const long long max_exp =
                (static_cast<long long>(doc.shape.i) * doc.shape.j * doc.shape.k) > 5000000LL
                    ? large_grid_cap()
                    : -1;
#endif
            for (size_t i = 0; i < doc.tasks.size(); ++i) {
                const RouteTask& t = doc.tasks[i];
                AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                               doc.params, max_exp, true);
                doc.results[i] = to_scene_result(r);
            }
        } else {
            route_multi_into_doc(doc, priority ? priority : "longest", true);
        }
        char* p = dup_string(dumps_scene(doc));
        if (!p) return R3D_ERR_RUNTIME;
        *out_scene_text = p;
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// ============================================================================ Level 2
extern "C" R3dEngine* r3d_create(void) {
    try {
        return new R3dEngine();
    } catch (...) {
        return nullptr;
    }
}

extern "C" void r3d_destroy(R3dEngine* e) {
    delete e;
}

extern "C" R3dStatus r3d_load_scene_text(R3dEngine* e, const char* scene_text) {
    if (!e || !scene_text) return R3D_ERR_ARG;
    try {
        e->doc = loads_scene(scene_text);
        if (e->doc.results.size() < e->doc.tasks.size()) e->doc.results.resize(e->doc.tasks.size());
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_PARSE;
    }
}

extern "C" R3dStatus r3d_set_grid(R3dEngine* e, const R3dGrid* g) {
    if (!e || !g) return R3D_ERR_ARG;
    if (g->cell_mm <= 0.0) return R3D_ERR_ARG;
    if (g->nx <= 0 || g->ny <= 0 || g->nz <= 0) return R3D_ERR_ARG;
    // corridor.hpp pack20 은 축당 20비트(최대 2^20-1 = 1,048,575).
    // 초과 시 64비트 키 충돌 → 즉시 R3D_ERR_RANGE 로 차단.
    constexpr int kPack20Max = (1 << 20) - 1;
    if (g->nx > kPack20Max || g->ny > kPack20Max || g->nz > kPack20Max) return R3D_ERR_RANGE;
    e->doc.cell_mm = g->cell_mm;
    e->doc.origin = Vec3{g->ox, g->oy, g->oz};
    e->doc.shape = Cell{g->nx, g->ny, g->nz};
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_params(R3dEngine* e, const R3dParams* p) {
    if (!e || !p) return R3D_ERR_ARG;
    if (p->cell_mm <= 0.0) return R3D_ERR_ARG;
    if (p->w_turn < 0.0 || p->w_clear < 0.0 || p->w_corridor < 0.0) return R3D_ERR_ARG;
    if (p->w_heur < 0.0 || p->w_heur_near < 0.0) return R3D_ERR_ARG;
    if (p->clearance_radius < 0) return R3D_ERR_ARG;
    // clearance_connectivity: 0=기본값(6으로 처리), 그 외 6 또는 26 만 허용.
    if (p->clearance_connectivity != 0 && p->clearance_connectivity != 6 && p->clearance_connectivity != 26)
        return R3D_ERR_ARG;
    e->doc.params.cell_mm = p->cell_mm;
    e->doc.params.w_turn = p->w_turn;
    e->doc.params.w_clear = p->w_clear;
    e->doc.params.clearance_radius = p->clearance_radius;
    // 0 은 기본값으로 6(면 이웃 BFS)으로 처리 — clearance_map 이 6/26 만 허용하므로 변환.
    e->doc.params.clearance_connectivity = (p->clearance_connectivity == 0) ? 6 : p->clearance_connectivity;
    e->doc.params.w_corridor = p->w_corridor;
    e->doc.params.w_heur = p->w_heur;
    e->doc.params.w_heur_near = p->w_heur_near;
    e->doc.params.corridor_radius = p->corridor_radius > 0 ? p->corridor_radius : 1;
    e->doc.params.rack_levels.clear();
    {
        int rc = p->rack_level_count;
        if (rc < 0) rc = 0;
        if (rc > 8) rc = 8;
        for (int i = 0; i < rc; ++i) e->doc.params.rack_levels.push_back(p->rack_levels[i]);
    }
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_runtime_options(R3dEngine* e, const R3dRuntimeOptions* opt) {
    if (!e || !opt) return R3D_ERR_ARG;
    e->runtime = *opt;
    if (e->runtime.large_grid_threshold < 0) e->runtime.large_grid_threshold = 0;
    if (e->runtime.max_expansions < 0) e->runtime.max_expansions = 0;
    if (e->runtime.fallback_expansions < 0) e->runtime.fallback_expansions = 0;
    if (e->runtime.hier_factor < 0) e->runtime.hier_factor = 0;
    if (e->runtime.hier_factor > 128) e->runtime.hier_factor = 128;
    if (e->runtime.hier_radius < 0) e->runtime.hier_radius = 0;
    if (e->runtime.hier_radius > 64) e->runtime.hier_radius = 64;
    if (e->runtime.hier_probe < 0) e->runtime.hier_probe = 0;
    if (e->runtime.ripup_enabled < -1) e->runtime.ripup_enabled = -1;
    if (e->runtime.ripup_enabled > 1) e->runtime.ripup_enabled = 1;
    if (e->runtime.cbs_expansions < 0) e->runtime.cbs_expansions = 0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_trace_options(R3dEngine* e, const R3dTraceOptions* opt) {
    if (!e || !opt) return R3D_ERR_ARG;
    e->trace_options = *opt;
    if (e->trace_options.sample_every <= 0) e->trace_options.sample_every = 1000;
    if (e->trace_options.max_events_per_task <= 0) e->trace_options.max_events_per_task = 20000;
    e->trace.opt = e->trace_options;
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_trace_file(R3dEngine* e, const char* path_utf8) {
    if (!e || !path_utf8 || !*path_utf8) return R3D_ERR_ARG;
    return e->trace.open(path_utf8) ? R3D_OK : R3D_ERR_RUNTIME;
}

extern "C" R3dStatus r3d_flush_trace(R3dEngine* e) {
    if (!e) return R3D_ERR_ARG;
    e->trace.flush();
    return R3D_OK;
}

extern "C" R3dStatus r3d_add_obstacle(R3dEngine* e, double minx, double miny, double minz,
                                      double maxx, double maxy, double maxz) {
    if (!e) return R3D_ERR_ARG;
    try {
        Obstacle o;
        o.min_xyz = Vec3{minx, miny, minz};
        o.max_xyz = Vec3{maxx, maxy, maxz};
        e->doc.obstacles.push_back(std::move(o));
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_add_passthrough(R3dEngine* e, double minx, double miny, double minz,
                                         double maxx, double maxy, double maxz) {
    if (!e) return R3D_ERR_ARG;
    try {
        Obstacle o;
        o.min_xyz = Vec3{minx, miny, minz};
        o.max_xyz = Vec3{maxx, maxy, maxz};
        e->doc.passthrough.push_back(std::move(o));
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" int32_t r3d_add_task(R3dEngine* e, double sx, double sy, double sz, double gx, double gy,
                                double gz, const char* utility, const char* utility_group) {
    if (!e) return -1;
    try {
        RouteTask t;
        t.start_mm = Vec3{sx, sy, sz};
        t.end_mm = Vec3{gx, gy, gz};
        t.utility = opt_str(utility);
        t.utility_group = opt_str(utility_group);
        e->doc.tasks.push_back(std::move(t));
        if (e->doc.results.size() < e->doc.tasks.size()) e->doc.results.resize(e->doc.tasks.size());
        return static_cast<int32_t>(e->doc.tasks.size()) - 1;
    } catch (...) {
        return -1;
    }
}

extern "C" R3dStatus r3d_set_task_endpoints(R3dEngine* e, int32_t task, double sx, double sy,
                                            double sz, double gx, double gy, double gz) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    RouteTask& t = e->doc.tasks[static_cast<size_t>(task)];
    t.start_mm = Vec3{sx, sy, sz};
    t.end_mm = Vec3{gx, gy, gz};
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_task_diameter(R3dEngine* e, int32_t task, double diameter_mm) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    e->doc.tasks[static_cast<size_t>(task)].diameter_mm = diameter_mm > 0.0 ? diameter_mm : 0.0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_task_goal_dir(R3dEngine* e, int32_t task, int32_t axis) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    e->doc.tasks[static_cast<size_t>(task)].goal_dir = (axis >= 0 && axis <= 5) ? axis : -1;
    return R3D_OK;
}

extern "C" R3dStatus r3d_route_multi(R3dEngine* e, const char* priority) {
    if (!e) return R3D_ERR_ARG;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        const std::vector<Cell>* seed = e->corridor_seed.empty() ? nullptr : &e->corridor_seed;
        route_multi_into_doc(e->doc, priority ? priority : "longest", e->collect_visited, {}, seed,
                             e->pipe_radius, e->per_task_radius, e->cbs_depth, e->min_straight_mult,
                             e->pipe_gap_mm, &e->runtime, &e->trace);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_set_per_task_radius(R3dEngine* e, int32_t enabled) {
    if (!e) return R3D_ERR_ARG;
    e->per_task_radius = enabled != 0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_cbs_depth(R3dEngine* e, int32_t depth) {
    if (!e) return R3D_ERR_ARG;
    e->cbs_depth = depth < 0 ? 0 : (depth > 3 ? 3 : depth);
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_min_straight(R3dEngine* e, double mult) {
    if (!e) return R3D_ERR_ARG;
    e->min_straight_mult = mult > 0.0 ? mult : 0.0;
    return R3D_OK;
}

// 코너 최소직선(절대 mm, 하드 제약) 설정. A* 가 '꺾인 뒤 이 길이만큼 직진 전엔 못 꺾도록' 강제한다.
//   라우팅 직전 apply_min_straight_cells 가 ceil(mm/cell)→params.min_straight_cells 로 환산. 0=OFF(골든 불변).
extern "C" R3dStatus r3d_set_min_straight_mm(R3dEngine* e, double mm) {
    if (!e) return R3D_ERR_ARG;
    e->min_straight_mm = mm > 0.0 ? mm : 0.0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_pipe_gap(R3dEngine* e, double gap_mm) {
    if (!e) return R3D_ERR_ARG;
    e->pipe_gap_mm = gap_mm > 0.0 ? gap_mm : 0.0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                              void* user) {
    if (!e) return R3D_ERR_ARG;
    try {
        ProgressCb on_pipe;
        if (cb) {
            on_pipe = [cb, user](int phase, int oi, int ti, bool ok, double len, int turns,
                                 long long exp, double ms, int done, int total, double prog,
                                 const std::vector<Cell>* path) -> int {
                const int32_t* pptr = nullptr;
                int32_t plen = 0;
                std::vector<int32_t> buf;
                if (path && !path->empty()) {
                    buf.reserve(path->size() * 3);
                    for (const Cell& c : *path) {
                        buf.push_back(c.i);
                        buf.push_back(c.j);
                        buf.push_back(c.k);
                    }
                    pptr = buf.data();
                    plen = static_cast<int32_t>(path->size());
                }
                return cb(user, phase, oi, ti, ok ? 1 : 0, len, turns, exp, ms, done, total, prog,
                          pptr, plen);
            };
        }
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        const std::vector<Cell>* seed = e->corridor_seed.empty() ? nullptr : &e->corridor_seed;
        route_multi_into_doc(e->doc, priority ? priority : "longest", e->collect_visited, on_pipe, seed,
                             e->pipe_radius, e->per_task_radius, e->cbs_depth, e->min_straight_mult,
                             e->pipe_gap_mm, &e->runtime, &e->trace);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_set_corridor_cells(R3dEngine* e, const int32_t* ijk, int32_t n) {
    if (!e) return R3D_ERR_ARG;
    try {
        e->corridor_seed.clear();
        if (ijk && n > 0) {
            e->corridor_seed.reserve(static_cast<size_t>(n));
            for (int32_t t = 0; t < n; ++t)
                e->corridor_seed.push_back(Cell{ijk[3 * t], ijk[3 * t + 1], ijk[3 * t + 2]});
        }
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_set_collect_visited(R3dEngine* e, int32_t enabled) {
    if (!e) return R3D_ERR_ARG;
    e->collect_visited = (enabled != 0);
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_pipe_radius(R3dEngine* e, int32_t radius_cells) {
    if (!e) return R3D_ERR_ARG;
    e->pipe_radius = radius_cells > 0 ? radius_cells : 0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_route_ripup(R3dEngine* e, const char* priority, int32_t max_rounds,
                                     int32_t max_ripup) {
    if (!e) return R3D_ERR_ARG;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        SceneDoc& doc = e->doc;
        const std::string prio = priority ? priority : "longest";
        auto run = [&](auto&& occ) {
            std::vector<int> order = order_indices(occ, doc.tasks, prio);
            auto mr = route_ripup(occ, doc.tasks, doc.params, prio, 0, 2, -1,
                                  max_rounds > 0 ? max_rounds : 10, max_ripup > 0 ? max_ripup : 4,
                                  e->collect_visited);
            doc.results.assign(doc.tasks.size(), std::nullopt);
            for (size_t pos = 0; pos < mr.pipes.size(); ++pos)
                doc.results[static_cast<size_t>(order[pos])] = to_scene_result(mr.pipes[pos].result);
        };
        #ifdef ROUTING3D_USE_OPENVDB
        run(vdb_from_doc(doc));
#else
        const long long cells = (long long)doc.shape.i * doc.shape.j * doc.shape.k;
        if (cells > 5000000LL) run(implicit_from_doc(doc));
        else run(occupancy_from_doc(doc));
#endif
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_route_corridor(R3dEngine* e, int32_t factor, int32_t radius) {
    if (!e) return R3D_ERR_ARG;
    if (factor < 1 || radius < 0) return R3D_ERR_ARG;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        SceneDoc& doc = e->doc;

        SparseOccupancy fine(doc.shape, doc.origin, doc.cell_mm);
        Cell cshape{(doc.shape.i + factor - 1) / factor, (doc.shape.j + factor - 1) / factor,
                    (doc.shape.k + factor - 1) / factor};
        SparseOccupancy coarse(cshape, doc.origin, doc.cell_mm * factor);
        for (const Obstacle& o : doc.obstacles) {
            try {
                AABB box(o.min_xyz, o.max_xyz);
                fine.add_box(box);
                coarse.add_box(box);
            } catch (...) {
            }
        }

        doc.results.assign(doc.tasks.size(), std::nullopt);
        for (size_t i = 0; i < doc.tasks.size(); ++i) {
            const RouteTask& t = doc.tasks[i];
            Cell s = snap_to_free_cell(fine, fine.to_cell(t.start_mm), 2);
            Cell g = snap_to_free_cell(fine, fine.to_cell(t.end_mm), 2);
            CorridorRoute cr = route_corridor(fine, coarse, s, g, factor, radius, -1);
            doc.results[i] = to_scene_result(cr.fine);
        }
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_route_corridor_multi(R3dEngine* e, int32_t factor, int32_t radius,
                                              const char* priority, int32_t pipe_radius) {
    if (!e) return R3D_ERR_ARG;
    if (factor < 1 || radius < 0) return R3D_ERR_ARG;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        SceneDoc& doc = e->doc;

        SparseOccupancy fine(doc.shape, doc.origin, doc.cell_mm);
        Cell cshape{(doc.shape.i + factor - 1) / factor, (doc.shape.j + factor - 1) / factor,
                    (doc.shape.k + factor - 1) / factor};
        SparseOccupancy coarse(cshape, doc.origin, doc.cell_mm * factor);
        for (const Obstacle& o : doc.obstacles) {
            try {
                AABB box(o.min_xyz, o.max_xyz);
                fine.add_box(box);
                coarse.add_box(box);
            } catch (...) {
            }
        }

        const std::string prio = priority ? priority : "longest";
        const std::vector<int> order = order_indices(fine, doc.tasks, prio);
        const int pr = pipe_radius > 0 ? pipe_radius : 0;

        doc.results.assign(doc.tasks.size(), std::nullopt);
        for (int idx : order) {
            const RouteTask& t = doc.tasks[static_cast<size_t>(idx)];
            Cell s = snap_to_free_cell(fine, fine.to_cell(t.start_mm), 2);
            Cell g = snap_to_free_cell(fine, fine.to_cell(t.end_mm), 2);
            CorridorRoute cr = route_corridor(fine, coarse, s, g, factor, radius, -1);
            if (cr.fine.success) {
                mark_pipe(fine, cr.fine.path, pr);
            }
            doc.results[static_cast<size_t>(idx)] = to_scene_result(cr.fine);
        }
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_route_task(R3dEngine* e, int32_t task, R3dResult* out) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        const RouteTask& t = e->doc.tasks[static_cast<size_t>(task)];
        const long long cells =
            static_cast<long long>(e->doc.shape.i) * e->doc.shape.j * e->doc.shape.k;
        SceneResult sr;
#ifdef ROUTING3D_USE_OPENVDB
        {
            VdbOccupancy occ = vdb_from_doc(e->doc);
            AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                           e->doc.params, large_grid_cap(), e->collect_visited);
            sr = to_scene_result(r);
        }
#else
        if (cells > 5000000LL) {
            ImplicitOccupancy occ = implicit_from_doc(e->doc);
            AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                           e->doc.params, large_grid_cap(), e->collect_visited);
            sr = to_scene_result(r);
        } else {
            DenseOccupancy occ = occupancy_from_doc(e->doc);
            AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                           e->doc.params, -1, e->collect_visited);
            sr = to_scene_result(r);
        }
#endif
        if (e->doc.results.size() != e->doc.tasks.size())
            e->doc.results.resize(e->doc.tasks.size());
        e->doc.results[static_cast<size_t>(task)] = sr;
        if (out) fill_result(*out, e->doc.results[static_cast<size_t>(task)]);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_get_result(const R3dEngine* e, int32_t task, R3dResult* out) {
    if (!e || !out) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    if (task >= static_cast<int32_t>(e->doc.results.size()) ||
        !e->doc.results[static_cast<size_t>(task)]) {
        *out = R3dResult{};
        return R3D_ERR_RUNTIME;
    }
    fill_result(*out, e->doc.results[static_cast<size_t>(task)]);
    return R3D_OK;
}

extern "C" int32_t r3d_copy_path(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells) {
    if (!e || !buf || buf_cells <= 0) return 0;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.results.size())) return 0;
    const std::optional<SceneResult>& r = e->doc.results[static_cast<size_t>(task)];
    if (!r || !r->path) return 0;
    const std::vector<Cell>& path = *r->path;
    int32_t n = std::min<int32_t>(buf_cells, static_cast<int32_t>(path.size()));
    for (int32_t i = 0; i < n; ++i) {
        buf[3 * i + 0] = path[static_cast<size_t>(i)].i;
        buf[3 * i + 1] = path[static_cast<size_t>(i)].j;
        buf[3 * i + 2] = path[static_cast<size_t>(i)].k;
    }
    return n;
}

extern "C" int32_t r3d_copy_visited(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells) {
    if (!e || !buf || buf_cells <= 0) return 0;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.results.size())) return 0;
    const std::optional<SceneResult>& r = e->doc.results[static_cast<size_t>(task)];
    if (!r || !r->visited) return 0;
    const std::vector<Cell>& vs = *r->visited;
    int32_t n = std::min<int32_t>(buf_cells, static_cast<int32_t>(vs.size()));
    for (int32_t i = 0; i < n; ++i) {
        buf[3 * i + 0] = vs[static_cast<size_t>(i)].i;
        buf[3 * i + 1] = vs[static_cast<size_t>(i)].j;
        buf[3 * i + 2] = vs[static_cast<size_t>(i)].k;
    }
    return n;
}

extern "C" int32_t r3d_copy_blocked(const R3dEngine* e, int32_t* buf, int32_t buf_cells) {
    if (!e) return 0;
    try {
        const Cell& shape = e->doc.shape;
        bool size_only = (buf == nullptr || buf_cells <= 0);
        auto scan = [&](auto&& occ) -> int32_t {
            int32_t written = 0;
            for (int i = 0; i < shape.i && (size_only || written < buf_cells); ++i)
                for (int j = 0; j < shape.j && (size_only || written < buf_cells); ++j)
                    for (int k = 0; k < shape.k && (size_only || written < buf_cells); ++k) {
                        Cell c{i, j, k};
                        if (!occ.is_blocked(c)) continue;
                        if (!size_only) {
                            buf[3 * written + 0] = i;
                            buf[3 * written + 1] = j;
                            buf[3 * written + 2] = k;
                        }
                        ++written;
                    }
            return written;
        };
        const long long cells = (long long)shape.i * shape.j * shape.k;
#ifdef ROUTING3D_USE_OPENVDB
        {
            VdbOccupancy occ = vdb_from_doc(e->doc);
            std::vector<Cell> cells_v = occ.blocked_cells();
            if (size_only) return static_cast<int32_t>(cells_v.size());
            int32_t n = std::min<int32_t>(buf_cells, static_cast<int32_t>(cells_v.size()));
            for (int32_t idx = 0; idx < n; ++idx) {
                const Cell& c = cells_v[static_cast<size_t>(idx)];
                buf[3 * idx + 0] = c.i;
                buf[3 * idx + 1] = c.j;
                buf[3 * idx + 2] = c.k;
            }
            return n;
        }
#else
        if (cells > 5000000LL) {
            ImplicitOccupancy occ = implicit_from_doc(e->doc);
            std::vector<Cell> cells_v = occ.blocked_cells();
            if (size_only) return static_cast<int32_t>(cells_v.size());
            int32_t n = std::min<int32_t>(buf_cells, static_cast<int32_t>(cells_v.size()));
            for (int32_t idx = 0; idx < n; ++idx) {
                const Cell& c = cells_v[static_cast<size_t>(idx)];
                buf[3 * idx + 0] = c.i;
                buf[3 * idx + 1] = c.j;
                buf[3 * idx + 2] = c.k;
            }
            return n;
        }
        return scan(occupancy_from_doc(e->doc));
#endif
    } catch (...) {
        return 0;
    }
}

// 대형 격자 시각화용 — blocked cell 을 최대 max_cells 개 균일 샘플링해 buf 에 복사.
// 수억 셀 격자에서 r3d_copy_blocked 전체 요청 대신 UI 미리보기용 대표 셀만 받는다.
// 반환=실제 복사 셀 수(<=max_cells). max_cells<=0 또는 buf=NULL 이면 0 반환.
// Copy a deterministic preview sample of blocked cells without materializing the full set.
extern "C" int32_t r3d_copy_blocked_sampled(const R3dEngine* e, int32_t max_cells, int32_t* buf) {
    if (!e || max_cells <= 0 || !buf) return 0;
    try {
#ifdef ROUTING3D_USE_OPENVDB
        std::vector<Cell> cells = vdb_from_doc(e->doc).blocked_cells_sampled(max_cells);
        for (int32_t idx = 0; idx < static_cast<int32_t>(cells.size()); ++idx) {
            buf[3 * idx + 0] = cells[static_cast<size_t>(idx)].i;
            buf[3 * idx + 1] = cells[static_cast<size_t>(idx)].j;
            buf[3 * idx + 2] = cells[static_cast<size_t>(idx)].k;
        }
        return static_cast<int32_t>(cells.size());
#else
        const Cell& shape = e->doc.shape;
        if (shape.i <= 0 || shape.j <= 0 || shape.k <= 0) return 0;
        struct RangeInfo { CellRange r; long long count; };
        std::vector<RangeInfo> ranges;
        ranges.reserve(e->doc.obstacles.size());
        long long total = 0;
        for (const auto& ob : e->doc.obstacles) {
            CellRange r = grid_box_range(AABB{ob.min_xyz, ob.max_xyz}, e->doc.origin, e->doc.cell_mm, shape);
            if (r.empty()) continue;
            const long long dx = static_cast<long long>(r.hi.i - r.lo.i);
            const long long dy = static_cast<long long>(r.hi.j - r.lo.j);
            const long long dz = static_cast<long long>(r.hi.k - r.lo.k);
            const long long cnt = dx * dy * dz;
            if (cnt <= 0) continue;
            ranges.push_back(RangeInfo{r, cnt});
            total += cnt;
        }
        if (total <= 0) return 0;
        const long long take = std::min<long long>(total, max_cells);
        long long ordinal = 0;
        long long picked = 0;
        long long next_pick = 0;
        auto emit = [&](const Cell& c) {
            buf[3 * picked + 0] = c.i;
            buf[3 * picked + 1] = c.j;
            buf[3 * picked + 2] = c.k;
            ++picked;
            next_pick = (picked * total) / take;
        };
        for (const RangeInfo& ri : ranges) {
            const CellRange& r = ri.r;
            for (int i = r.lo.i; i < r.hi.i && picked < take; ++i)
                for (int j = r.lo.j; j < r.hi.j && picked < take; ++j)
                    for (int k = r.lo.k; k < r.hi.k && picked < take; ++k, ++ordinal)
                        if (ordinal >= next_pick) emit(Cell{i, j, k});
        }
        return static_cast<int32_t>(picked);
#endif
    } catch (...) {
        return 0;
    }
}

// 통과 객체 점유 셀을 buf 에 복사(가시화 '통과 점유맵'). r3d_copy_blocked 와 동일 규약.
extern "C" int32_t r3d_copy_passthrough(const R3dEngine* e, int32_t* buf, int32_t buf_cells) {
    if (!e) return 0;
    try {
        DenseOccupancy occ = occupancy_from_passthrough(e->doc);
        const Cell& shape = e->doc.shape;
        bool size_only = (buf == nullptr || buf_cells <= 0);
        int32_t written = 0;
        for (int i = 0; i < shape.i && (size_only || written < buf_cells); ++i)
            for (int j = 0; j < shape.j && (size_only || written < buf_cells); ++j)
                for (int k = 0; k < shape.k && (size_only || written < buf_cells); ++k) {
                    Cell c{i, j, k};
                    if (!occ.is_blocked(c)) continue;
                    if (!size_only) {
                        buf[3 * written + 0] = i;
                        buf[3 * written + 1] = j;
                        buf[3 * written + 2] = k;
                    }
                    ++written;
                }
        return written;
    } catch (...) {
        return 0;
    }
}

extern "C" R3dStatus r3d_dump_scene_text(const R3dEngine* e, char** out_text) {
    if (!e || !out_text) return R3D_ERR_ARG;
    *out_text = nullptr;
    try {
        char* p = dup_string(dumps_scene(e->doc));
        if (!p) return R3D_ERR_RUNTIME;
        *out_text = p;
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// ============================================================================ 가변셀 옥트리 라우팅
extern "C" R3dStatus r3d_route_task_octree(R3dEngine* e, int32_t task,
                                           int64_t max_exp, int32_t goal_dir,
                                           R3dResult* out) {
    if (!e || !out) return R3D_ERR_ARG;
    const int n = (int)e->doc.tasks.size();
    if (task < 0 || task >= n) return R3D_ERR_RANGE;
    *out = R3dResult{};
    try {
        apply_min_straight_cells(e);
        // 옥트리 빌드
        OctreeOccupancy occ;
        occ.build(e->doc);

        const RouteTask& t = e->doc.tasks[task];
        Cell start = occ.to_cell(t.start_mm);
        Cell goal  = occ.to_cell(t.end_mm);

        // 격자 범위 스냅
        auto snap = [&](Cell c) -> Cell {
            c.i = std::clamp(c.i, 0, occ.nx-1);
            c.j = std::clamp(c.j, 0, occ.ny-1);
            c.k = std::clamp(c.k, 0, occ.nz-1);
            return c;
        };
        start = snap(start); goal = snap(goal);

        RouteParams params = e->doc.params;
        long long mx = (max_exp > 0) ? max_exp : env_ll("R3D_MAX_EXP", 0LL);

        AStarResult res = astar_octree(occ, start, goal, params, mx, goal_dir,
                                       e->collect_visited);

        // 결과 저장
        if ((int)e->doc.results.size() <= task)
            e->doc.results.resize(n, std::nullopt);
        e->doc.results[task] = to_scene_result(res);

        out->success       = res.success ? 1 : 0;
        out->length_mm     = res.length_mm;
        out->cost_mm       = res.cost_mm;
        out->turns         = res.turns;
        out->expanded_nodes = res.expanded_nodes;
        out->elapsed_ms    = res.elapsed_ms;
        out->path_len      = res.success ? (int32_t)res.path.size() : 0;
        out->visited_len   = e->collect_visited ? (int32_t)res.visited.size() : 0;
        out->fail_reason   = (int32_t)res.fail;
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// ============================================================================ 옥트리 리프 열거 (3D 가시화)
// Enumerate octree leaves. buf == nullptr and maxCount == 0 performs a size query.
extern "C" R3dStatus r3d_enum_octree_leaves(R3dEngine* e,
                                             R3dOctreeLeaf* buf, int32_t maxCount,
                                             int32_t* out_count) {
    if (!e || !out_count) return R3D_ERR_ARG;
    if (maxCount < 0) return R3D_ERR_ARG;
    if (!buf && maxCount > 0) return R3D_ERR_ARG;
    if (e->doc.shape.i <= 0) return R3D_ERR_ARG;   // scene 미로드
    *out_count = 0;
    try {
        OctreeOccupancy occ;
        occ.build(e->doc);

        int total = 0;
        const double cell = e->doc.cell_mm;
        const Vec3&  ori  = e->doc.origin;

        for (const auto& node : occ.nodes) {
            if (!node.is_leaf()) continue;
            ++total;
        }
        *out_count = total;
        if (!buf || maxCount == 0 || total == 0) return R3D_OK;

        const int target = std::min<int>(maxCount, total);
        int written = 0;
        int leaf_index = 0;

        auto write_leaf = [&](const auto& node) {
            buf[written].x0_mm  = (float)(ori.x + node.x0 * cell);
            buf[written].y0_mm  = (float)(ori.y + node.y0 * cell);
            buf[written].z0_mm  = (float)(ori.z + node.z0 * cell);
            buf[written].size_mm = (float)(node.side * cell);
            buf[written].state  = (int32_t)node.state;
            ++written;
        };

        for (const auto& node : occ.nodes) {
            if (!node.is_leaf()) continue;

            if (target == total) {
                write_leaf(node);
            } else {
                const int wanted = (int)(((int64_t)written * total) / target);
                if (leaf_index == wanted) write_leaf(node);
            }

            ++leaf_index;
            if (written >= target) break;
        }
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}
