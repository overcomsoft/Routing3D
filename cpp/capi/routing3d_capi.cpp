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
#include <functional>
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
    bool collect_visited = true;  // 기본 on — 뷰어 '방문맵' 즉시 사용. set_collect_visited 로 끔.
    // 학습된 회랑 셀(ijk) — w_corridor>0 일 때 route_multi 가 시드로 사용해 배관을 그 곁으로 유도(L2b).
    // r3d_set_corridor_cells 로 설정/초기화. 비어 있으면 기존 동작(깔린 배관 곁 번들링만).
    std::vector<Cell> corridor_seed;
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

// AStarResult → SceneResult(엔진 결과 저장 단위). 성공 시 경로 포함. visited 가 비어있지
// 않으면 함께 복사(가시화 '방문맵' / scene.txt [visited] 섹션).
SceneResult to_scene_result(const AStarResult& r) {
    SceneResult s;
    s.success = r.success;
    s.length_mm = r.length_mm;
    s.cost_mm = r.cost_mm;
    s.turns = r.turns;
    s.expanded_nodes = r.expanded_nodes;
    s.elapsed_ms = r.elapsed_ms;
    if (r.success) s.path = r.path;
    if (!r.visited.empty()) s.visited = r.visited;
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
    o.visited_len = r->visited ? static_cast<int32_t>(r->visited->size()) : 0;
}

// 거대 격자에서 복셀화 없이(O(장애물 수)) 점유를 표현하는 ImplicitOccupancy 를 doc 로부터 구성.
// 셀 크기와 무관한 메모리 → 25mm/10mm 등 정밀 격자의 저장 폭발/오버플로를 근본 해소(S3).
ImplicitOccupancy implicit_from_doc(const SceneDoc& doc) {
    ImplicitOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;  // 두께 0(퇴화) 박스는 건너뛴다(occupancy_from_doc 과 동일).
        }
    }
    return occ;
}

// 진행 콜백 타입(내부용). phase=0(탐색 진행)/1(배관 완료).
//   인자: phase, order_index, task_index, success, length_mm, turns, expanded_nodes, elapsed_ms,
//         done, total, progress01, path(완료·성공 시 경로 셀, 아니면 nullptr).
using ProgressCb = std::function<void(int, int, int, bool, double, int, long long, double, int, int,
                                      double, const std::vector<Cell>*)>;

// 다중 배관 순차 라우팅의 백엔드 무관 본체(Occ = Dense/Implicit). order/snap/astar/mark_pipe 동일.
// 결과를 '원본 작업 인덱스' 에 저장해 핸들 API(get_result(task)) 매핑을 보존한다.
// on_pipe 가 유효하면 배관마다 호출(진행 다이얼로그용) — 결과/순서에는 영향 없음.
template <class Occ>
void route_multi_impl(SceneDoc& doc, Occ occ, const std::string& priority, bool collect_visited,
                      const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr) {
    Occ work = occ.copy();  // 원본 점유 불변(M2).

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

    // 회랑 인력(params.w_corridor>0)이면 깔린 배관 곁을 회랑으로 키워 다음 배관을 끌어모은다
    // → 기존 설계처럼 공용 랙으로 뭉치고 굴곡/길이가 늘어난다. 0이면 기존 동작과 동일.
    doc.results.assign(static_cast<size_t>(n), std::nullopt);
    std::unordered_set<long long> corridor;
    const bool use_corridor = doc.params.w_corridor > 0.0;
    const int corridor_radius = doc.params.corridor_radius > 0 ? doc.params.corridor_radius : 1;
    // 학습된 회랑 시드(L2b) — w_corridor>0 일 때 외부 주입 셀(seed)을 회랑에 미리 넣어, 배관이
    // 그 곁을 '싸게'(w_corridor 면제) 지나가도록 유도(기존설계 스텁/랙 형상 따라가기). 0이면 무시.
    if (use_corridor && seed) {
        for (const Cell& c : *seed)
            if (work.in_bounds(c)) corridor.insert(static_cast<long long>(work.lin(c)));
    }
    // 대형 격자(예 25mm·1.3억 셀)에서는 경로가 없는/막힌 배관이 도달 가능한 셀을 전부 확장해
    // g/came 맵이 수 GB 로 폭증 → 메모리 고갈(0xC0000005). 탐색 상한을 둬 그런 배관을 조기 종료한다.
    // 작은 격자(골든 등)는 상한 없음(-1) 으로 기존 동작·결정성 보존.
    const long long max_exp = (occ.size() > 5000000LL) ? 12000000LL : -1;
    // 탐색 진행율 보고 간격(확장 수). 너무 잦으면 콜백 폭주 → 5만마다(배관당 수십 회).
    const long long progress_every = on_pipe ? 50000LL : 0;
    int done = 0;
    for (int oidx = 0; oidx < static_cast<int>(order.size()); ++oidx) {
        const int oi = order[static_cast<size_t>(oidx)];
        const RouteTask& t = doc.tasks[static_cast<size_t>(oi)];
        Cell s = snap_to_free_cell(work, work.to_cell(t.start_mm), 2);
        Cell g = snap_to_free_cell(work, work.to_cell(t.end_mm), 2);

        // 탐색 중 진행율(처리상태 %) 콜백 — phase=0. 현재 배관의 order/task 인덱스로 행을 찾는다.
        std::function<void(long long, double)> intra;
        if (on_pipe) {
            intra = [&](long long expanded, double prog) {
                on_pipe(0, oidx, oi, false, 0.0, 0, expanded, 0.0, done, n, prog, nullptr);
            };
        }
        AStarResult res = astar_weighted(work, s, g, doc.params, max_exp, collect_visited,
                                         use_corridor ? &corridor : nullptr,
                                         on_pipe ? &intra : nullptr, progress_every);
        bool ok = res.success && !res.path.empty();
        std::vector<Cell> path = res.path;
        doc.results[static_cast<size_t>(oi)] = to_scene_result(res);
        if (ok) {
            mark_pipe(work, path, 0);  // 깔린 경로를 점유로 추가(다음 배관 회피).
            if (use_corridor) add_corridor_cells(work, corridor, path, corridor_radius);
        }
        ++done;
        if (on_pipe) {  // phase=1 완료 — 지표 + (성공 시) 경로 셀.
            on_pipe(1, oidx, oi, ok, res.length_mm, res.turns, res.expanded_nodes, res.elapsed_ms,
                    done, n, 1.0, ok ? &path : nullptr);
        }
    }
}

// 격자 크기로 백엔드 선택(sparse 해법):
//   작은 격자(≤5M 셀, 골든 등) → DenseOccupancy: 기존 동작·바이트 결과 완전 불변.
//   거대 격자(>5M 셀, 25mm/10mm) → ImplicitOccupancy: 복셀화 없는 O(장애물) 저장 + 64비트 키 +
//   온디맨드 클리어런스 → 130MB/2GB 배열·520MB 거리변환·int 오버플로를 모두 회피.
void route_multi_into_doc(SceneDoc& doc, const std::string& priority, bool collect_visited,
                          const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr) {
    const long long cells =
        static_cast<long long>(doc.shape.i) * doc.shape.j * doc.shape.k;
    if (cells > 5000000LL) {
        route_multi_impl(doc, implicit_from_doc(doc), priority, collect_visited, on_pipe, seed);
    } else {
        route_multi_impl(doc, occupancy_from_doc(doc), priority, collect_visited, on_pipe, seed);
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
        // Level 1(문자열) API 는 핸들 없이 호출되므로 visited 수집 기본 on.
        if (m == "single") {
            DenseOccupancy occ = occupancy_from_doc(doc);
            doc.results.assign(doc.tasks.size(), std::nullopt);
            for (size_t i = 0; i < doc.tasks.size(); ++i) {
                const RouteTask& t = doc.tasks[i];
                AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                               doc.params, -1, true);
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
    e->doc.params.w_corridor = p->w_corridor;
    e->doc.params.w_heur = p->w_heur;
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

// 통과(pass-through) 객체 추가 — 점유맵 가시화용, 경로탐색 충돌 대상 아님(doc.passthrough).
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

extern "C" R3dStatus r3d_route_multi(R3dEngine* e, const char* priority) {
    if (!e) return R3D_ERR_ARG;
    try {
        const std::vector<Cell>* seed = e->corridor_seed.empty() ? nullptr : &e->corridor_seed;
        route_multi_into_doc(e->doc, priority ? priority : "longest", e->collect_visited, {}, seed);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

extern "C" R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                              void* user) {
    if (!e) return R3D_ERR_ARG;
    try {
        ProgressCb on_pipe;  // cb 가 널이면 비활성(콜백 없는 route_multi 와 동일).
        if (cb) {
            on_pipe = [cb, user](int phase, int oi, int ti, bool ok, double len, int turns,
                                 long long exp, double ms, int done, int total, double prog,
                                 const std::vector<Cell>* path) {
                // 경로 셀(i,j,k) 를 임시 int 배열로 펴서 콜백에 전달(포인터는 호출 동안만 유효).
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
                cb(user, phase, oi, ti, ok ? 1 : 0, len, turns, exp, ms, done, total, prog, pptr,
                   plen);
            };
        }
        const std::vector<Cell>* seed = e->corridor_seed.empty() ? nullptr : &e->corridor_seed;
        route_multi_into_doc(e->doc, priority ? priority : "longest", e->collect_visited, on_pipe, seed);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// 학습된 회랑 셀(ijk 삼중항 배열, 길이 n)을 엔진에 설정한다(L2b). w_corridor>0 일 때 route_multi 가
// 이 셀들을 회랑 시드로 삼아 배관을 그 곁으로 유도한다. n<=0 또는 ijk==null 이면 회랑을 비운다.
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

// rip-up & reroute(Step 3.8): 헤더 route_ripup 을 호출하되, 결과를 '원본 작업 인덱스'로
// 되돌려 doc.results 에 저장(get_result 매핑 보존, doc.tasks 불변). 우선순위 순열은
// order_indices 로 재현(route_ripup 내부 order_tasks 와 동일 안정 정렬 → 위치 일치).
extern "C" R3dStatus r3d_route_ripup(R3dEngine* e, const char* priority, int32_t max_rounds,
                                     int32_t max_ripup) {
    if (!e) return R3D_ERR_ARG;
    try {
        SceneDoc& doc = e->doc;
        const std::string prio = priority ? priority : "longest";
        DenseOccupancy occ = occupancy_from_doc(doc);
        std::vector<int> order = order_indices(occ, doc.tasks, prio);
        auto mr = route_ripup(occ, doc.tasks, doc.params, prio, 0, 2, -1,
                              max_rounds > 0 ? max_rounds : 10, max_ripup > 0 ? max_ripup : 4,
                              e->collect_visited);
        doc.results.assign(doc.tasks.size(), std::nullopt);
        for (size_t pos = 0; pos < mr.pipes.size(); ++pos)
            doc.results[static_cast<size_t>(order[pos])] = to_scene_result(mr.pipes[pos].result);
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

// 순차 계층 corridor: r3d_route_corridor 와 같은 Sparse + astar_hashed 이되, priority 순서로
// 한 배관씩 라우팅하고 성공 경로를 fine 점유에 mark_pipe 로 추가해 다음 배관이 피하게 한다(충돌 0).
extern "C" R3dStatus r3d_route_corridor_multi(R3dEngine* e, int32_t factor, int32_t radius,
                                              const char* priority, int32_t pipe_radius) {
    if (!e) return R3D_ERR_ARG;
    if (factor < 1 || radius < 0) return R3D_ERR_ARG;
    try {
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
                // 퇴화 박스 무시.
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
                // 다음 배관이 피하도록 fine 점유에 경로(+반경)를 추가. coarse 는 가이드라 미표시.
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
        DenseOccupancy occ = occupancy_from_doc(e->doc);
        const RouteTask& t = e->doc.tasks[static_cast<size_t>(task)];
        AStarResult r = astar_weighted(occ, occ.to_cell(t.start_mm), occ.to_cell(t.end_mm),
                                       e->doc.params, -1, e->collect_visited);
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

// 방문(확장) 셀 복사 — 가시화 '방문맵' 용. copy_path 와 동일 형식.
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

// 점유맵(블록된 셀) 인덱스 복사 — 가시화 '점유맵' 용. 현재 doc 의 obstacles 로 즉석 voxelize.
// buf=NULL, buf_cells=0 이면 총 셀 수만 반환(사이즈 조회). 부분 복사 시 처음 buf_cells 개.
extern "C" int32_t r3d_copy_blocked(const R3dEngine* e, int32_t* buf, int32_t buf_cells) {
    if (!e) return 0;
    try {
        DenseOccupancy occ = occupancy_from_doc(e->doc);
        const Cell& shape = e->doc.shape;
        // 사이즈 조회 모드: buf 미지정.
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
        return written;  // size_only=true 면 전체 카운트, false 면 실제 복사한 셀 수.
    } catch (...) {
        return 0;
    }
}

// 통과 객체 점유 셀 인덱스 복사 — 가시화 '통과 점유맵'. r3d_copy_blocked 와 동일 규약.
extern "C" int32_t r3d_copy_passthrough(const R3dEngine* e, int32_t* buf, int32_t buf_cells) {
    if (!e) return 0;
    try {
        DenseOccupancy occ = occupancy_from_passthrough(e->doc);
        const Cell& shape = e->doc.shape;
        // 사이즈 조회 모드: buf 미지정.
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
        return written;  // size_only=true 면 전체 카운트, false 면 실제 복사한 셀 수.
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
