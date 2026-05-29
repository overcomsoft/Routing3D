// Routing3D 네이티브 C ABI 구현 (routing3d_capi) — Phase 3
// =============================================================================
// [이 파일이 하는 일]
//   routing3d_capi.h 의 C ABI 를 C++ 코어 엔진 위에 얇게 구현한다. 모든 export 함수는
//   예외를 경계 밖으로 내보내지 않도록 try/catch 로 감싸 상태 코드로 보고한다.
//   엔진 상태(R3dEngine)는 SceneDoc 하나로 표현하고, 라우팅 시 점유맵을 즉석 구성한다.
//   설계: docs/csharp_helix_interop_design.md, 헤더: capi/routing3d_capi.h.
//
// [빌드/검증]  (프로젝트 루트에서)
//   cmake --build cpp/build --config Release --target routing3d_capi
//   ctest --test-dir cpp/build -C Release -R capi --output-on-failure
// =============================================================================
#define ROUTING3D_CAPI_EXPORTS
#include "routing3d_capi.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/corridor.hpp"
#include "routing3d/cost.hpp"
#include "routing3d/multi_route.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/scene_io.hpp"

using namespace routing3d;

// 불투명 핸들의 실제 정의: 씬 문서 하나(격자/파라미터/장애물/작업/결과)를 보유.
struct R3dEngine {
    SceneDoc doc;
};

namespace {

// std::string → malloc 버퍼(콜리 할당). r3d_free_string 으로 해제.
char* dup_string(const std::string& s) {
    char* p = static_cast<char*>(std::malloc(s.size() + 1));
    if (!p) return nullptr;
    std::memcpy(p, s.c_str(), s.size() + 1);
    return p;
}

// const char* → optional<string>. 널이면 None(=\N), 아니면 문자열(빈문자열 허용).
std::optional<std::string> opt_str(const char* s) {
    if (!s) return std::nullopt;
    return std::string(s);
}

// AStarResult → SceneResult(엔진 결과 저장 단위). 성공 시 경로 포함.
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

// optional<SceneResult> → R3dResult(POD). 없으면 0으로.
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
}

// 다중 배관 순차 라우팅을 '원본 작업 인덱스' 기준으로 doc.results 에 채운다.
// route_sequential 과 동일한 빌딩블록(order/snap/astar_weighted/mark_pipe)을 쓰되,
// 결과를 원래 인덱스에 저장해 핸들 API(get_result(task))의 매핑을 보존한다.
void route_multi_into_doc(SceneDoc& doc, const std::string& priority) {
    DenseOccupancy occ = occupancy_from_doc(doc);
    DenseOccupancy work = occ.copy();  // 원본 점유 불변.

    const int n = static_cast<int>(doc.tasks.size());
    std::vector<int> order(static_cast<size_t>(n));
    std::iota(order.begin(), order.end(), 0);

    auto dist = [&](int t) {
        return manhattan(work.to_cell(doc.tasks[static_cast<size_t>(t)].start_mm),
                         work.to_cell(doc.tasks[static_cast<size_t>(t)].end_mm));
    };
    if (priority == "original") {
        // 입력 순서 유지.
    } else if (priority == "shortest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) < dist(b); });
    } else if (priority == "longest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) > dist(b); });
    } else if (priority == "utility") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            const std::string la = doc.tasks[static_cast<size_t>(a)].utility_label();
            const std::string lb = doc.tasks[static_cast<size_t>(b)].utility_label();
            if (la != lb) return la < lb;
            return dist(a) > dist(b);
        });
    } else {
        throw std::invalid_argument("unknown priority: " + priority);
    }

    doc.results.assign(static_cast<size_t>(n), std::nullopt);
    for (int oi : order) {
        const RouteTask& t = doc.tasks[static_cast<size_t>(oi)];
        Cell s = snap_to_free_cell(work, work.to_cell(t.start_mm), 2);
        Cell g = snap_to_free_cell(work, work.to_cell(t.end_mm), 2);
        AStarResult res = astar_weighted(work, s, g, doc.params, -1);
        bool ok = res.success && !res.path.empty();
        std::vector<Cell> path = res.path;
        doc.results[static_cast<size_t>(oi)] = to_scene_result(res);
        if (ok) mark_pipe(work, path, 0);  // 깔린 경로를 점유로 추가(다음 배관 회피).
    }
}

}  // namespace

// ============================================================================ 공통
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
            DenseOccupancy occ = occupancy_from_doc(doc);
            doc.results.assign(doc.tasks.size(), std::nullopt);
            for (size_t i = 0; i < doc.tasks.size(); ++i) {
                const RouteTask& t = doc.tasks[i];
                AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                               doc.params, -1);
                doc.results[i] = to_scene_result(r);
            }
        } else {
            route_multi_into_doc(doc, priority ? priority : "longest");
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
    e->doc.cell_mm = g->cell_mm;
    e->doc.origin = Vec3{g->ox, g->oy, g->oz};
    e->doc.shape = Cell{g->nx, g->ny, g->nz};
    return R3D_OK;
}

extern "C" R3dStatus r3d_set_params(R3dEngine* e, const R3dParams* p) {
    if (!e || !p) return R3D_ERR_ARG;
    e->doc.params.cell_mm = p->cell_mm;
    e->doc.params.w_turn = p->w_turn;
    e->doc.params.w_clear = p->w_clear;
    e->doc.params.clearance_radius = p->clearance_radius;
    e->doc.params.clearance_connectivity = p->clearance_connectivity;
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

extern "C" R3dStatus r3d_route_multi(R3dEngine* e, const char* priority) {
    if (!e) return R3D_ERR_ARG;
    try {
        route_multi_into_doc(e->doc, priority ? priority : "longest");
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// 대형 장면용 corridor 라우팅: 장애물을 fine/coarse Sparse 점유로 만들고 작업별 route_corridor.
// Sparse + astar_hashed 라 occ.size() 배열을 잡지 않으므로 초대형 격자도 동작(메모리=점유 셀).
extern "C" R3dStatus r3d_route_corridor(R3dEngine* e, int32_t factor, int32_t radius) {
    if (!e) return R3D_ERR_ARG;
    if (factor < 1 || radius < 0) return R3D_ERR_ARG;
    try {
        SceneDoc& doc = e->doc;

        // fine/coarse 희소 점유맵(장애물만). coarse 셀 = fine 셀 × factor.
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
                // 퇴화 박스 무시.
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

extern "C" R3dStatus r3d_route_task(R3dEngine* e, int32_t task, R3dResult* out) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    try {
        DenseOccupancy occ = occupancy_from_doc(e->doc);
        const RouteTask& t = e->doc.tasks[static_cast<size_t>(task)];
        AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                       e->doc.params, -1);
        if (e->doc.results.size() != e->doc.tasks.size())
            e->doc.results.resize(e->doc.tasks.size());
        e->doc.results[static_cast<size_t>(task)] = to_scene_result(r);
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
        return R3D_ERR_RUNTIME;  // 아직 라우팅 안 됨.
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
