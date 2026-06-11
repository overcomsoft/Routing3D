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
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_set>
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
    bool collect_visited = false;  // 기본 OFF(A2) — 대형 장면 메모리/복사 비용 보호. 방문맵·단계 애니가
                                   // 필요할 때만 호출자가 r3d_set_collect_visited(1) 로 opt-in.
    // 학습된 회랑 셀(ijk) — w_corridor>0 일 때 route_multi 가 시드로 사용해 배관을 그 곁으로 유도(L2b).
    // r3d_set_corridor_cells 로 설정/초기화. 비어 있으면 기존 동작(깔린 배관 곁 번들링만).
    std::vector<Cell> corridor_seed;
    // 배관-배관 충돌 회피(옵션1): 깔린 배관을 점유로 추가할 때 팽창 반경(셀). 0=경로 셀만(기존).
    // >0 이면 mark_pipe 가 경로 ±radius 6-이웃을 막아 다음 배관 중심선을 그만큼 띄운다 → 실제 관경으로
    // 렌더해도 표면이 겹치지 않는다(시각/물리 충돌 해소). r3d_set_pipe_radius 로 설정. 관경/셀 기반 산출은
    // 호출자(뷰어 BuildEngineForRows)가 수행. env R3D_PIPE_RADIUS 로도 재정의 가능(헤드리스 A/B).
    int pipe_radius = 0;
    bool per_task_radius = false;  // B1 — ON 이면 route_multi 가 각 배관 diameter_mm 로 반경 자동 산출.
    // 배관-배관 이격(mm) — 두 배관 센터선 거리 ≥ r1 + r2 + pipe_gap_mm 보장. 0=기존 동작(표면 맞닿음·골든 불변).
    //   >0 이면 메인 루프가 깔린 배관을 routing 배관 기준 쌍 반경(ceil((r_a+r_b+gap)/cell))으로 막는다(per-pipe
    //   재구성). r3d_set_pipe_gap. 규격: 표면 사이 최소 60mm 띄움. env R3D_PIPE_GAP.
    double pipe_gap_mm = 0.0;
    // C1 negotiated-congestion(CBS-lite, Phase C) — 연쇄(재귀) rip-up 최대 깊이. 0=OFF(평면 rip-up만,
    //   기존 동작·골든 불변). >0 이면 평면 rip-up 후 남은 실패 배관을, blocker 가 재배치 못 하면 그 blocker 의
    //   blocker 까지 이 깊이만큼 재귀적으로 양보시켜 해소(무손실·결정적). r3d_set_cbs_depth / env R3D_CBS.
    int cbs_depth = 0;
    // C2 코너 최소반경(Phase C) — 엘보 사이 직선(런)이 (mult × 관경) 미만이면 제작 불가 → 경로(셀) 단계에서
    //   양옆 코너를 충돌없는 직교 연결로 흡수해 없앤다(충돌검사 통과·꺾임 비증가일 때만, 양 끝점 고정). 0=OFF
    //   (기존 동작·골든 불변). 권장 2.0(엘보 간 직선 ≥ 2×관경). r3d_set_min_straight / env R3D_MIN_STRAIGHT.
    double min_straight_mult = 0.0;
};

namespace {

// 환경변수에서 양의 long long 한도를 읽는다(미설정/0이하/파싱실패면 def). 거대격자(25mm 등) 탐색 상한을
// 32GB+ 서버에서 키워 어려운 배관 커버리지를 높이는 용도 — 메모리는 확장 노드 해시맵(g/came/closed)에
// 비례하므로 RAM 여유가 있을 때만 올린다. 작은 격자(골든)는 애초에 무제한(-1)이라 영향 없음.
long long env_ll(const char* name, long long def) {
    if (const char* s = std::getenv(name)) {
        char* end = nullptr;
        long long v = std::strtoll(s, &end, 10);
        if (end != s && v > 0) return v;
    }
    return def;
}

// 거대격자 탐색 상한(메모리/런어웨이 보호). 기본 48M(구 12M, 32GB+ 서버 기준 상향 — 25mm 정밀 격자의
// 막힌/혼잡 배관을 더 깊이 탐색해 성공수↑; 한 번에 한 배관만 탐색하므로 피크 메모리=이 한도분의 해시맵).
// env R3D_MAX_EXP 로 추가 재정의. (12M 은 짧은 거리[#146 2,277mm·91셀]인데도 혼잡 종단 포켓을 못 뚫고
// 도달하던 한계였다 — 25mm + pipe_radius 팽창으로 마지막 배관 진입로가 좁아진 경우.)
long long large_grid_cap() { return env_ll("R3D_MAX_EXP", 48000000LL); }

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
    s.fail = static_cast<int>(r.fail);   // 실패 사유(A1) 전달.
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
    o.fail_reason = static_cast<int32_t>(r->fail);   // 실패 사유(A1, RouteFail). 성공=0.
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

// 거대격자 장거리 배관 가속용 coarse 점유맵(factor 배 셀). 동일 origin·박스, 셀만 factor 배 굵게.
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

// 계층 corridor 라우팅 — coarse 가이드로 fine 탐색을 tube(coarse 경로 ±radius 팽창)로 하드 제한한다.
// 거대격자 장거리 배관의 탐색량을 크게 줄인다. **비용모델은 fine 과 동일**(weighted A* + 클리어런스 +
// 턴 페널티)이라 경로 품질 보존. 가이드 실패/튜브 내 경로 없음 → false 반환(호출자가 무제한 fine 으로
// 폴백 → 성공 수 회귀 0). work=깔린배관 포함 fine 점유(충돌회피 유지).
template <class Occ>
bool route_hier(const Occ& work, const ImplicitOccupancy& coarse, int factor, int radius,
                Cell s, Cell g, const RouteParams& params, long long max_exp,
                bool collect_visited, AStarResult& out) {
    auto to_coarse = [factor](const Cell& c) {
        return Cell{c.i / factor, c.j / factor, c.k / factor};  // i>=0 → 바닥 나눗셈.
    };
    // fine 종단의 coarse 셀은 장비/덕트 근처라 coarse(굵은) 해상도에서 막혀 있을 수 있다 → 자유 coarse
    // 셀로 스냅해 가이드가 시작/도착하게 한다. 스냅으로 생긴 종단 갭은 아래 연결 박스로 튜브에 포함.
    Cell cs0 = to_coarse(s), cg0 = to_coarse(g);
    Cell cs = snap_to_free_cell(coarse, cs0, 4);
    Cell cgl = snap_to_free_cell(coarse, cg0, 4);
    RouteParams cp = params;
    cp.cell_mm = coarse.cell_mm();
    AStarResult cg = astar_weighted(coarse, cs, cgl, cp, 2000000LL, false);
    if (!cg.success || cg.path.empty()) return false;

    // 2) 튜브(coarse 셀 키 집합) = coarse 경로 ±radius 팽창 + 양 끝(실제 fine 종단↔스냅 coarse) 연결 박스.
    auto corr = std::make_shared<std::unordered_set<uint64_t>>();
    corr->reserve(cg.path.size() * static_cast<size_t>((2 * radius + 1) * (2 * radius + 1) * (2 * radius + 1)) + 64);
    auto add_dilated = [&](const Cell& c) {
        for (int di = -radius; di <= radius; ++di)
            for (int dj = -radius; dj <= radius; ++dj)
                for (int dk = -radius; dk <= radius; ++dk)
                    corr->insert(pack20(Cell{c.i + di, c.j + dj, c.k + dk}));
    };
    for (const Cell& c : cg.path) add_dilated(c);
    // 종단 연결 박스(to_coarse(종단)↔스냅 coarse, ±radius) — fine 종단 셀이 반드시 튜브에 들도록.
    auto add_box = [&](const Cell& a, const Cell& b) {
        int i0 = std::min(a.i, b.i) - radius, i1 = std::max(a.i, b.i) + radius;
        int j0 = std::min(a.j, b.j) - radius, j1 = std::max(a.j, b.j) + radius;
        int k0 = std::min(a.k, b.k) - radius, k1 = std::max(a.k, b.k) + radius;
        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                for (int k = k0; k <= k1; ++k) corr->insert(pack20(Cell{i, j, k}));
    };
    add_box(cs0, cs);
    add_box(cg0, cgl);

    // 3) fine A* — fine 셀의 coarse 셀이 튜브에 있을 때만 확장(하드 제한). 비용모델 동일(품질 보존).
    auto in_corr = [corr, factor](const Cell& fc) {
        return corr->count(pack20(Cell{fc.i / factor, fc.j / factor, fc.k / factor})) > 0;
    };
    AStarResult fr = astar_weighted(work, s, g, params, max_exp, collect_visited,
                                    nullptr, nullptr, 0, in_corr);
    if (!fr.success || fr.path.empty()) return false;
    out = std::move(fr);
    return true;
}

// 진행 콜백 타입(내부용). phase=0(탐색 진행)/1(배관 완료). 반환=0 계속, 0아님=취소(abort).
//   인자: phase, order_index, task_index, success, length_mm, turns, expanded_nodes, elapsed_ms,
//         done, total, progress01, path(완료·성공 시 경로 셀, 아니면 nullptr).
using ProgressCb = std::function<int(int, int, int, bool, double, int, long long, double, int, int,
                                     double, const std::vector<Cell>*)>;

// ---------------------------------------------------------------- 경로 후처리: 킨크/역주행 제거
// A→B 를 직교(직선 또는 단일 엘보)로 잇는 셀열을 axisOrder 순서로 생성(끝점 포함).
// 안 다른 축은 자연히 건너뛴다(이동량 0).
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

// A 와 B 를 직교 ≤1엘보(축차 ≤2) 로 잇는 충돌없는 셀열을 찾는다(축순서 후보 전수). 3축 차이는 실패.
template <class Occ>
bool ortho_connect(const Occ& occ, Cell A, Cell B, std::vector<Cell>& out) {
    int axes = (A.i != B.i ? 1 : 0) + (A.j != B.j ? 1 : 0) + (A.k != B.k ? 1 : 0);
    if (axes > 2) return false;                       // 2엘보(3축)는 단축 대상에서 제외(보수적).
    static const int orders[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    for (const auto& ord : orders) {
        std::vector<Cell> v = walk_order(A, B, ord);
        bool clear = true;
        for (const Cell& c : v) if (occ.is_blocked(c)) { clear = false; break; }
        if (clear) { out = std::move(v); return true; }
    }
    return false;
}

// 경로에서 역주행/킨크 제거: 떨어진 두 경로점을 '더 짧은 직교 연결(충돌없음)'로 대체하는 그리디 단축.
// occ(장애물+이미 깔린 배관) 충돌만 검사 → 결과는 항상 물리적 유효. mark_pipe 전에 적용하므로 후속
// 배관이 단축경로를 회피(M1/M2 보존). 결정적(가장 먼 j 우선·고정 축순서). 길이가 줄 때만 대체(무한루프 차단).
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
        for (int j = n - 1; j >= a + 2; --j) {   // 가장 먼 j 우선(최대 단축).
            std::vector<Cell> seg;
            if (!ortho_connect(occ, path[static_cast<size_t>(a)], path[static_cast<size_t>(j)], seg))
                continue;
            const int segSteps = static_cast<int>(seg.size()) - 1, origSteps = j - a;
            if (segSteps > origSteps) continue;   // 길어지면 기각.
            if (segSteps == origSteps) {
                // 같은 길이면 꺾임이 더 적을 때만 대체(평면 톱니/지그재그 정리).
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

// 코너 최소반경(C2, Phase C): 엘보 사이 직선(런)이 min_run_cells 미만이면 제작 불가(짧은 단관) → 그 짧은
// 런을 가로질러 양옆 코너를 '충돌없는 직교(≤1엘보) 연결'(ortho_connect)로 흡수해 없앤다. occ(장애물+이미
// 깔린 배관) 충돌검사 통과 + 꺾임 비증가 + 길이 비증가일 때만 대체(무손실·물리유효). 양 끝점(PoC/스텁)은
// 고정(코너 후보에서 제외). 결정적(코너 오름차순·한 번에 하나, 변경 시 코너 재계산). min_run_cells<=1 이면
// 원본 그대로 반환(골든/기존 동작 불변). PathRectifier(렌더 레벨, 되돌림)와 달리 경로 셀 단계라 충돌 안전.
template <class Occ>
std::vector<Cell> enforce_min_straight(const Occ& occ, const std::vector<Cell>& path, int min_run_cells) {
    if (min_run_cells <= 1 || path.size() < 5) return path;
    std::vector<Cell> cur = path;
    bool changed = true;
    int guard = 0;
    while (changed && guard++ < 64) {   // guard=무한루프 차단(매 반복 1개 흡수 → 최대 코너 수만큼).
        changed = false;
        // 코너 인덱스(방향 전환점) 수집 — [0, 전환점들…, n-1]. 양 끝(0,n-1)은 고정점.
        std::vector<int> corners;
        corners.push_back(0);
        for (size_t m = 1; m + 1 < cur.size(); ++m) {
            Cell d0{cur[m].i - cur[m - 1].i, cur[m].j - cur[m - 1].j, cur[m].k - cur[m - 1].k};
            Cell d1{cur[m + 1].i - cur[m].i, cur[m + 1].j - cur[m].j, cur[m + 1].k - cur[m].k};
            if (!(d0 == d1)) corners.push_back(static_cast<int>(m));
        }
        corners.push_back(static_cast<int>(cur.size()) - 1);
        // 인접 코너쌍 런 길이를 보고, 짧은 내부 런을 양옆 코너(ci-1, ci+1) 직교연결로 흡수.
        for (size_t ci = 1; ci + 1 < corners.size(); ++ci) {
            const int runNext = corners[ci + 1] - corners[ci];      // ci~ci+1 런(셀 수).
            const int runPrev = corners[ci] - corners[ci - 1];      // ci-1~ci 런(셀 수).
            if (runNext >= min_run_cells && runPrev >= min_run_cells) continue;  // 둘 다 충분.
            const int a = corners[ci - 1], b = corners[ci + 1];
            std::vector<Cell> seg;
            if (!ortho_connect(occ, cur[static_cast<size_t>(a)], cur[static_cast<size_t>(b)], seg))
                continue;                                            // 충돌 또는 3축차 → 흡수 불가.
            std::vector<Cell> slice(cur.begin() + a, cur.begin() + b + 1);
            if (count_turns(seg) > count_turns(slice)) continue;     // 꺾임 증가 금지.
            if (static_cast<int>(seg.size()) - 1 > b - a) continue;  // 길이 증가 금지.
            std::vector<Cell> next(cur.begin(), cur.begin() + a);    // [0..a) + seg + (b..end].
            for (const Cell& c : seg) next.push_back(c);
            for (size_t t = static_cast<size_t>(b) + 1; t < cur.size(); ++t) next.push_back(cur[t]);
            cur = std::move(next);
            changed = true;
            break;   // 코너 재계산(흡수로 인덱스가 바뀜).
        }
    }
    return cur;
}

// 다중 배관 순차 라우팅의 백엔드 무관 본체(Occ = Dense/Implicit). order/snap/astar/mark_pipe 동일.
// 결과를 '원본 작업 인덱스' 에 저장해 핸들 API(get_result(task)) 매핑을 보존한다.
// on_pipe 가 유효하면 배관마다 호출(진행 다이얼로그용) — 결과/순서에는 영향 없음.
template <class Occ>
void route_multi_impl(SceneDoc& doc, Occ occ, const std::string& priority, bool collect_visited,
                      const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr,
                      int pipe_radius = 0, bool per_task_radius = false,
                      int cbs_depth = 0, double min_straight_mult = 0.0,
                      double pipe_gap_mm = 0.0) {
    Occ work = occ.copy();  // 원본 점유 불변(M2).
    // C1 CBS 깊이(연쇄 rip-up) — 인자 우선, env R3D_CBS(>=0) 가 있으면 재정의. 0=OFF(평면 rip-up만·골든 불변).
    int eff_cbs_depth = cbs_depth < 0 ? 0 : cbs_depth;
    if (const char* cs = std::getenv("R3D_CBS")) {
        char* end = nullptr; long v = std::strtol(cs, &end, 10);
        if (end != cs && v >= 0) eff_cbs_depth = static_cast<int>(v);
    }
    if (eff_cbs_depth > 3) eff_cbs_depth = 3;   // 분기 폭발 차단(분기 ≤ (MAXBLK+1)^(depth+1)).
    // C2 코너 최소반경 배수(엘보 간 직선 ≥ mult×관경) — 인자 우선, env R3D_MIN_STRAIGHT(>=0) 재정의. 0=OFF.
    double eff_min_straight = min_straight_mult < 0.0 ? 0.0 : min_straight_mult;
    if (const char* ms = std::getenv("R3D_MIN_STRAIGHT")) {
        char* end = nullptr; double v = std::strtod(ms, &end);
        if (end != ms && v >= 0.0) eff_min_straight = v;
    }
    // 배관 점유 팽창 반경(옵션1, 배관-배관 충돌 회피). 인자 우선, env R3D_PIPE_RADIUS(>=0) 가 있으면 재정의
    // (헤드리스 --dbroute A/B 용). 0=경로 셀만(기존 동작·골든 불변).
    int eff_pipe_radius = pipe_radius < 0 ? 0 : pipe_radius;
    if (const char* pr = std::getenv("R3D_PIPE_RADIUS")) {
        char* end = nullptr;
        long v = std::strtol(pr, &end, 10);
        if (end != pr && v >= 0) eff_pipe_radius = static_cast<int>(v);
    }
    // per-task 관경 반경(B1) — ON 이면 각 배관의 diameter_mm 로 반경을 자동 산출(호출자 책임 제거, 가는 배관
    //   과패킹 해소). OFF(기본) 또는 관경 미상이면 글로벌 eff_pipe_radius 폴백 → 기존 동작·골든 불변.
    //   env R3D_PER_TASK_RADIUS 로도 켤 수 있다(헤드리스 A/B).
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
        return eff_pipe_radius;   // OFF/관경 미상 → 글로벌.
    };
    // ── 배관-배관 이격(센터선 거리 ≥ r1 + r2 + gap, 기본 60mm) ──
    // 기존 마킹(radius_of ≈ ceil(d/cell)-1)은 센터선을 '약 관경(d)'만큼만 띄워 두 배관 '표면이 딱 붙는다'
    // (gap=0). 규격은 두 배관 반경합 + 여유(60mm). gap>0 이면 **메인 루프에서 깔린 배관을 routing 배관 기준
    // 쌍(pairwise) 반경 = ceil((r_a + r_b + gap)/cell) 으로 막아** 다음 배관 센터선을 정확히 r_a+r_b+gap 만큼
    // 띄운다(per-pipe 재구성). gap=0(기본)이면 기존 증분 마킹(골든/기존 동작 불변). 인자 우선·env R3D_PIPE_GAP.
    double eff_gap_mm = pipe_gap_mm < 0.0 ? 0.0 : pipe_gap_mm;
    if (const char* pg = std::getenv("R3D_PIPE_GAP")) {
        char* end = nullptr; double v = std::strtod(pg, &end);
        if (end != pg && v >= 0.0) eff_gap_mm = v;
    }
    const bool use_gap = eff_gap_mm > 0.0 && cell_for_r > 0.0;
    const int PAIR_RADIUS_MAX = 24;   // 쌍 반경 상한(거대 관경+gap 폭주 차단).
    // 관경 반경(mm) — per_task & 관경 알면 d/2, 아니면 글로벌 반경(셀)을 mm 로 환산.
    auto rmm_of = [&](int ti) -> double {
        if (eff_per_task && ti >= 0 && ti < static_cast<int>(doc.tasks.size())) {
            double d = doc.tasks[static_cast<size_t>(ti)].diameter_mm;
            if (d > 0.0) return d * 0.5;
        }
        return eff_pipe_radius * cell_for_r;
    };
    // 쌍(pairwise) 마킹 반경(셀): 깔린 a 를 routing b 기준으로 막을 때 = ceil((r_a + r_b + gap)/cell).
    auto pair_radius = [&](int a, int b) -> int {
        double sep = rmm_of(a) + rmm_of(b) + eff_gap_mm;
        int r = static_cast<int>(std::ceil(sep / cell_for_r));
        if (r < 0) r = 0; if (r > PAIR_RADIUS_MAX) r = PAIR_RADIUS_MAX;
        return r;
    };
    // per-task 관경 clearance(B2) — per_task 가 ON 이고 w_clear>0 이면, 그 배관의 관경 반경만큼 벽(장애물)에서
    //   중심선을 띄우도록 clearance_radius 임계를 max(기존, 반경)으로 올린다(굵은 배관이 벽에 표면을 박지 않게).
    //   반경 ≤ 기존 clearance_radius(가는 배관)거나 OFF면 doc.params 그대로 → 기존 동작·골든 불변.
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
        // 입력 순서 유지.
    } else if (priority == "shortest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) < dist(b); });
    } else if (priority == "longest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) > dist(b); });
    } else if (priority == "diameter") {
        // 굵은 배관 먼저(동률은 거리 긴 것 먼저) — 굵은 배관이 최단(직선) 경로를 선점하고 가는 배관이
        // 그 곁을 피하게 한다. 관경 미상(0)이면 전 작업 동률 → longest 와 동일(기존 동작 불변).
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            if (dia(a) != dia(b)) return dia(a) > dia(b);
            return dist(a) > dist(b);
        });
    } else if (priority == "utility") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            const std::string la = doc.tasks[static_cast<size_t>(a)].utility_label();
            const std::string lb = doc.tasks[static_cast<size_t>(b)].utility_label();
            if (la != lb) return la < lb;
            if (dia(a) != dia(b)) return dia(a) > dia(b);   // 유틸 묶음 안에서 굵은 배관 먼저.
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
    const long long max_exp = (occ.size() > 5000000LL) ? large_grid_cap() : -1;
    // 탐색 진행율 보고 간격(확장 수). 너무 잦으면 콜백 폭주 → 5만마다(배관당 수십 회).
    const long long progress_every = on_pipe ? 50000LL : 0;
    // 계층 corridor 가속(거대격자 어려운 배관) — coarse 가이드로 fine 탐색을 튜브에 한정해 탐색량을 줄인다
    // (품질·성공수 보존). **escalation 게이트**: 먼저 저예산(HIER_PROBE) 직접 A* 를 돌려 대부분(쉬운 배관·
    // 개방 랙 직선)은 빠르게 성공시키고, 예산을 초과하는 '어려운 배관'만 계층 corridor 로 재시도한다.
    //   (거리 기반 게이트는 긴 직선까지 hier 로 보내 역행 — cell=50 스텁ON 134ms→24s. probe 기반이 옳다.)
    // 작은 격자(골든)는 미적용 → 골든/기존 동작 완전 불변.
    // 그룹/회랑 모드(use_corridor)에서도 hier 를 켠다 — 과거엔 껐으나, 그러면 어려운 배관(예 #146)이
    // 단일 bounded weighted A* 만으로 혼잡 종단을 못 뚫고 상한 도달 실패했다. probe(저예산 직접 A*)는
    // 회랑 바이어스를 그대로 쓰고(쉬운 배관 번들 유지), 예산 초과한 어려운 배관만 계층 corridor 로
    // escalate(연결 우선, 소프트 바이어스 없이 튜브 한정) → 번들 보존 + 어려운 배관 구제 양립.
    const bool large_grid = occ.size() > 5000000LL;
    const bool use_hier = large_grid;
    const int HIER_FACTOR = 8, HIER_RADIUS = 2;
    const long long HIER_PROBE = 300000LL;     // 직접 A* 저예산 — 초과(어려운 배관)면 hier 로 escalate.
    // probe(300k)·hier(튜브 한정) 둘 다 실패한 어려운 배관의 '무제한 폴백' 예산.
    //   기본 = max_exp(=12M, 무손실): 이 폴백은 실제로 경로를 구제한다 — 실측 cell=50 ALL 에서 폴백을
    //   2M 로 자르면 성공 146→133(성공 배관 확장수가 1.9M~11.9M 까지 연속 분포, 진짜 실패 12M 과 뒤섞임).
    //   즉 '평탄한 예산 절감'은 반드시 실제 경로를 잃는다(속도/커버리지 트레이드오프). 그래서 기본은 무손실.
    //   ※ env R3D_FALLBACK_EXP=N(>0) 로 의도적으로 낮춰 시간을 끊을 수 있다(혼잡 배관 커버리지 일부 포기).
    //     0/미설정 = max_exp(무손실 기본). 줄이는 대상은 'hier 실패 후 폴백'만 — 작은격자/corridor 주 탐색 불변.
    long long fallback_exp = max_exp;
    if (const char* fe = std::getenv("R3D_FALLBACK_EXP")) {
        char* end = nullptr; long long v = std::strtoll(fe, &end, 10);
        if (end != fe && v > 0) fallback_exp = (max_exp > 0) ? std::min(v, max_exp) : v;
    }
    std::optional<ImplicitOccupancy> coarse;   // 첫 어려운 배관에서 1회 지연 생성.
    // (독립 배관 병렬화 시도·기각: optimistic 병렬 A*+순차 충돌 복구는 순차와 바이트 동일했으나(정확),
    //  project6 c100/c25/c10 전부 wall-clock 이득 0~음수였다. 미세격자 A* 는 거대 해시맵을 스트리밍하는
    //  메모리대역 바운드라 스레드들이 대역을 경합하고, Phase A 가 '마크 없는' 더 큰 탐색을 중복 수행해
    //  병렬 이득을 상쇄. → 도입 보류, 순차 유지. 자세한 측정은 CLAUDE.md '다음 작업 후보'.)
    int done = 0;
    bool aborted = false;   // on_pipe 가 0아님(취소)을 반환하면 set → 현재 배관 탐색 중단 + 배치 루프 종료.
    std::map<int, std::vector<Cell>> placed;   // 성공 배관 oi→경로(rip-up 회복용, 키 오름차순 결정적).
    for (int oidx = 0; oidx < static_cast<int>(order.size()); ++oidx) {
        const int oi = order[static_cast<size_t>(oidx)];
        const RouteTask& t = doc.tasks[static_cast<size_t>(oi)];
        // 이격 갭(use_gap) 모드 — 깔린 배관을 'routing 배관(oi) 기준 쌍 반경'으로 다시 막아 센터선 거리를
        //   정확히 r_a + r_b + gap 으로 보장한다(per-pipe 재구성). gap=0 이면 위에서 만든 증분 work 를 그대로 쓴다.
        if (use_gap) {
            work = occ.copy();
            for (const auto& kv : placed) mark_pipe(work, kv.second, pair_radius(kv.first, oi));
        }
        // 종단 스냅 반경 — 기본 2. 배관 팽창(eff_pipe_radius>0)을 쓰면 앞 배관이 인접 종단 셀까지 막아
        // (공용 랙·근접 PoC) 종단이 묻혀 exp=0 즉시 실패가 난다 → 스냅 반경을 팽창분만큼 키워 종단이
        // 자유셀로 탈출하게 한다(가장 가까운 자유셀 선택이라 위치 왜곡 최소). radius=0 이면 기존(2) 동일.
        // use_gap 이면 깔린 배관이 쌍 반경(더 큼)으로 막혀 있어, 종단이 그 확장영역을 벗어나도록 스냅 반경도
        //   쌍 자기반경(ceil((2r+gap)/cell))만큼 키운다(근접 PoC 가 묻혀 실패하지 않게). gap=0 이면 기존(2+radius).
        const int snap_r = use_gap ? 2 + pair_radius(oi, oi) : 2 + radius_of(oi);
        Cell s = snap_to_free_cell(work, work.to_cell(t.start_mm), snap_r);
        Cell g = snap_to_free_cell(work, work.to_cell(t.end_mm), snap_r);
        const RouteParams tp = params_for(oi);   // per-task 관경 clearance(B2) 반영(OFF/가는관=doc.params 동일).

        // 탐색 중 진행율(처리상태 %) 콜백 — phase=0. 현재 배관의 order/task 인덱스로 행을 찾는다.
        // 콜백이 취소(0아님)를 반환하면 aborted 를 세우고 true 반환 → astar 가 탐색 루프를 즉시 종료.
        std::function<bool(long long, double)> intra;
        if (on_pipe) {
            intra = [&](long long expanded, double prog) -> bool {
                if (on_pipe(0, oidx, oi, false, 0.0, 0, expanded, 0.0, done, n, prog, nullptr) != 0)
                    aborted = true;
                return aborted;
            };
        }
        AStarResult res;
        bool routed = false;
        // 폴백(아래 !routed) 예산 — 기본은 max_exp(작은격자/corridor 주 탐색은 무제한 보존). hier 까지 실패한
        // 어려운 배관만 bounded(fallback_exp)로 끊어 시간을 절약한다(성공수 보존, 위 상수 주석 참조).
        long long fb_exp = max_exp;
        if (use_hier) {
            // 1) 저예산 직접 A* — 쉬운 배관(개방 랙 직선 등)은 여기서 빠르게 성공(hier 오버헤드 없음).
            //    회랑 모드면 probe 도 회랑 바이어스를 줘 쉬운 배관 번들을 유지한다(어려운 배관만 아래로 escalate).
            const long long probe = (max_exp > 0) ? std::min(HIER_PROBE, max_exp) : HIER_PROBE;
            res = astar_weighted(work, s, g, tp, probe, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 on_pipe ? &intra : nullptr, progress_every, AllowAll{}, t.goal_dir);
            if (res.success && !res.path.empty()) {
                routed = true;                       // 쉬운 배관 — 직접 최적 경로 채택.
            } else if (res.expanded_nodes >= probe) {
                // 2) 저예산 초과(어려운 배관) → 계층 corridor(coarse 가이드 → 튜브 한정)로 재시도.
                if (!coarse) coarse.emplace(coarse_implicit_from_doc(doc, HIER_FACTOR));
                if (route_hier(work, *coarse, HIER_FACTOR, HIER_RADIUS, s, g, tp,
                               max_exp, collect_visited, res))
                    routed = true;
                else
                    fb_exp = fallback_exp;   // probe+hier 모두 실패 = 사실상 막힘 → 폴백 예산 제한.
            } else {
                routed = true;   // probe 소진 전 탐색 고갈 = 경로 없음(접근불가) → 그 실패 결과 채택.
            }
        }
        if (!routed)
            res = astar_weighted(work, s, g, tp, fb_exp, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 on_pipe ? &intra : nullptr, progress_every, AllowAll{}, t.goal_dir);
        bool ok = res.success && !res.path.empty();
        // 목표 진입축 제약(goal_dir)으로 실패하면 무제약으로 1회 폴백 — 연결 우선(성공률 보존). 일직선
        //   진입은 못 해도 경로는 살린다(혼잡 종단). 진입축 제약이 없던(goal_dir<0) 배관은 그대로.
        if (!ok && t.goal_dir >= 0 && !aborted) {
            res = astar_weighted(work, s, g, tp, fb_exp, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 on_pipe ? &intra : nullptr, progress_every, AllowAll{}, -1);
            ok = res.success && !res.path.empty();
        }
        std::vector<Cell> path = res.path;
        // 킨크/역주행 제거(가중 탐색 전용, w_heur>1). 골든·표준 A*(w=1)는 미적용 → 결과 바이트 불변.
        // mark 전에 적용해 후속 배관이 단축경로를 회피(M1/M2 보존). 회랑/번들 바이어스가 만든 톱니도 정리.
        if (ok && doc.params.w_heur > 1.0 && path.size() >= 4) {
            std::vector<Cell> up = unkink_path(work, path);
            if (up.size() < path.size()) {  // 더 짧아졌을 때만 채택(길이·꺾임 감소).
                path = std::move(up);
                res.path = path;
                res.length_mm = (path.size() - 1) * doc.params.cell_mm;
                res.turns = count_turns(path);
            }
        }
        // (C2 코너 최소반경은 배치 중간이 아니라 '모든 배관 라우팅 후' 비교란 최종 패스로 적용 — 아래 참조.
        //  배치 중간에 직선화하면 바뀐 셀이 다음 배관의 점유를 교란해 오히려 총 꺾임이 늘었다[실측 135→141].)
        doc.results[static_cast<size_t>(oi)] = to_scene_result(res);
        if (ok) {
            // 깔린 경로(+반경)를 점유로 추가(다음 배관 회피). per-task 반경(B1, OFF면 글로벌)으로 관경만큼 띄움.
            mark_pipe(work, path, radius_of(oi));
            if (use_corridor) add_corridor_cells(work, corridor, path, corridor_radius);
            placed[oi] = path;   // rip-up 회복(아래)용 — oi→경로(결정적 std::map 순회).
        }
        ++done;
        if (on_pipe) {  // phase=1 완료 — 지표 + (성공 시) 경로 셀. 반환이 취소면 다음 배관부터 중단.
            if (on_pipe(1, oidx, oi, ok, res.length_mm, res.turns, res.expanded_nodes, res.elapsed_ms,
                        done, n, 1.0, ok ? &path : nullptr) != 0)
                aborted = true;
        }
        // 취소 요청 — 완료된 배관 결과(doc.results)는 보존하고 남은 배관은 처리하지 않고 종료.
        if (aborted) break;
    }

    // ---- rip-up 회복(옵션2) ----
    // main 패스 후 남은 실패 배관을, 그 '장애물-only 이상 경로'를 가로막는 placed 배관(blocker)을 뜯어
    // 재배치해 해소한다. **무손실**(채택 시 성공 단조 +1)·**결정적**(blocker=placed 키 오름차순). pipe_radius
    // 는 동일 적용하되 회랑 바이어스는 미사용(route_ripup 와 동일 — 연결 우선). build_work 가 매 시도 occ(장애물
    // only) 사본에서 부분집합을 다시 깔아 M1(셀 공유 0)을 보존. 거대격자 + 미취소 + 실패>0 일 때만(작은 골든
    // 격자는 main 으로 충분 → 골든/기존 동작 불변). env R3D_RIPUP=off 로 끌 수 있다.
    bool ripup_on = true;
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
            const int snap_r = 2 + radius_of(ti);   // per-task 반경(B1).
            const RouteParams tpr = params_for(ti); // per-task 관경 clearance(B2).
            Cell ss = snap_to_free_cell(w, w.to_cell(tt.start_mm), snap_r);
            Cell gg = snap_to_free_cell(w, w.to_cell(tt.end_mm), snap_r);
            AStarResult r = astar_weighted(w, ss, gg, tpr, max_exp, false,
                                           nullptr, nullptr, 0, AllowAll{}, tt.goal_dir);
            if ((!r.success || r.path.empty()) && tt.goal_dir >= 0)   // 진입축 막힘 → 무제약 폴백.
                r = astar_weighted(w, ss, gg, tpr, max_exp, false);
            return r;
        };
        auto build_work = [&](const std::map<int, std::vector<Cell>>& paths) -> Occ {
            Occ w = occ.copy();   // occ = 장애물 only(불변 기준).
            for (const auto& kv : paths) mark_pipe(w, kv.second, radius_of(kv.first));   // per-task 반경.
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
                AStarResult ideal = route_on(occ, f);   // 장애물만으로의 이상 경로.
                if (!(ideal.success && !ideal.path.empty())) continue;  // 장애물만으로도 불가(접근불가).
                std::unordered_set<uint64_t> cs;
                cs.reserve(ideal.path.size() * 2);
                for (const Cell& c : ideal.path) cs.insert(pack(c));
                std::vector<int> blockers;   // placed 키 오름차순(std::map).
                for (const auto& kv : placed) {
                    for (const Cell& c : kv.second)
                        if (cs.count(pack(c))) { blockers.push_back(kv.first); break; }
                }
                if (blockers.empty() || static_cast<int>(blockers.size()) > MAX_RIPUP) continue;
                std::map<int, std::vector<Cell>> trial = placed;
                for (int b : blockers) trial.erase(b);
                Occ wt = build_work(trial);
                AStarResult rf = route_on(wt, f);   // 뜯어낸 공간에서 실패 배관 재배치.
                if (!(rf.success && !rf.path.empty())) continue;
                mark_pipe(wt, rf.path, radius_of(f));   // per-task 반경(B1).
                trial[f] = rf.path;
                std::vector<AStarResult> rbs(blockers.size());
                bool all_ok = true;
                for (size_t bi = 0; bi < blockers.size(); ++bi) {
                    AStarResult rb = route_on(wt, blockers[bi]);   // 뜯은 blocker 재라우팅.
                    if (rb.success && !rb.path.empty()) {
                        mark_pipe(wt, rb.path, radius_of(blockers[bi]));   // per-task 반경(B1).
                        trial[blockers[bi]] = rb.path;
                    } else {
                        all_ok = false;
                    }
                    rbs[bi] = std::move(rb);
                }
                if (!all_ok) continue;   // 무손실 위배(blocker 재배치 실패) → 이 시도 폐기.
                placed = std::move(trial);
                doc.results[static_cast<size_t>(f)] = to_scene_result(rf);
                for (size_t bi = 0; bi < blockers.size(); ++bi)
                    doc.results[static_cast<size_t>(blockers[bi])] = to_scene_result(rbs[bi]);
                changed = true;
                // 회복된 실패 배관을 콜백으로 알린다(phase=1, oidx=-1=rip-up 표식) → 라이브 3D/행 갱신.
                if (on_pipe)
                    on_pipe(1, -1, f, true, rf.length_mm, rf.turns, rf.expanded_nodes, rf.elapsed_ms,
                            done, n, 1.0, &placed[f]);
            }
            if (!changed) break;   // 더 회복 불가 → 종료.
        }
    }

    // ---- C1 negotiated-congestion (CBS-lite, Phase C) ----
    // 평면 rip-up(직접 blocker만 재배치)으로도 남은 실패 배관을, blocker 가 재배치 못 하면 그 blocker 의
    // blocker 까지 bounded depth 로 재귀적으로 양보시켜 해소한다(conflict-based search 경량판). 핵심 불변식:
    //   resolve(target, state) 가 true 면 결과 out 은 **state 의 모든 배관 + target 을 전부 포함**(재귀
    //   resolve 도 동일 보장 by construction) → 성공 수 단조 +1(무손실). 결정적(정렬 키·고정 순서). 깊이가
    //   매 재귀 1 감소하므로 종료(분기 ≤ (MAXBLK+1)^(depth+1)). 기본 eff_cbs_depth=0 → 미실행(골든 불변).
    if (eff_cbs_depth > 0 && !aborted && large_grid && has_fail()) {
        const int CBS_MAXBLK = 4;   // 한 레벨 blocker 상한(분기 폭발 차단).
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
        // 경로 셀 → AStarResult(doc.results 저장용 — 길이·꺾임 재계산, 확장수는 진단 보조라 0).
        auto result_from_path = [&](const std::vector<Cell>& p) -> AStarResult {
            AStarResult r;
            r.success = true; r.path = p;
            r.length_mm = (p.size() - 1) * doc.params.cell_mm;
            r.turns = count_turns(p);
            return r;
        };
        // 재귀 협상: state 위에 target 을 끼워넣되, 막는 blocker 를 depth 만큼 재귀 양보시킨다.
        //   성공 시 out = state 전 배관 + target (전부 라우팅됨). 실패면 state 불변(부작용 없음).
        std::function<bool(int, const std::map<int, std::vector<Cell>>&, int,
                           std::map<int, std::vector<Cell>>&)> resolve;
        resolve = [&](int target, const std::map<int, std::vector<Cell>>& state, int depth,
                      std::map<int, std::vector<Cell>>& out) -> bool {
            AStarResult ideal = route_on(occ, target);          // 장애물만으로의 이상 경로.
            if (!(ideal.success && !ideal.path.empty())) return false;
            std::unordered_set<uint64_t> cs;
            cs.reserve(ideal.path.size() * 2);
            for (const Cell& c : ideal.path) cs.insert(pack(c));
            std::vector<int> blockers;                          // state 키 오름차순(std::map) → 결정적.
            for (const auto& kv : state) {
                if (kv.first == target) continue;
                for (const Cell& c : kv.second)
                    if (cs.count(pack(c))) { blockers.push_back(kv.first); break; }
            }
            if (blockers.empty() || static_cast<int>(blockers.size()) > CBS_MAXBLK) return false;
            std::map<int, std::vector<Cell>> trial = state;
            for (int b : blockers) trial.erase(b);              // blocker 뜯기.
            AStarResult rf = route_on(build_work(trial), target);
            if (!(rf.success && !rf.path.empty())) return false;
            trial[target] = rf.path;                            // target 배치.
            for (int b : blockers) {                            // 뜯은 blocker 재배치(or 재귀 양보).
                AStarResult rb = route_on(build_work(trial), b);
                if (rb.success && !rb.path.empty()) { trial[b] = rb.path; continue; }
                if (depth <= 0) return false;                   // 더 못 양보 → 무손실 위배 → 폐기.
                std::map<int, std::vector<Cell>> sub;
                if (!resolve(b, trial, depth - 1, sub)) return false;
                trial = std::move(sub);                         // sub ⊇ trial ∪ {b} (불변식).
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
                if (!resolve(f, placed, eff_cbs_depth, out)) continue;
                // 채택(무손실) — out 의 배관 중 경로가 바뀐/새로 생긴 것만 결과 갱신.
                for (const auto& kv : out) {
                    auto it = placed.find(kv.first);
                    if (it == placed.end() || it->second != kv.second)
                        doc.results[static_cast<size_t>(kv.first)] = to_scene_result(result_from_path(kv.second));
                }
                placed = std::move(out);
                changed = true;
                if (on_pipe)   // 회복된 배관 라이브 갱신(oidx=-1=rip-up/CBS 표식).
                    on_pipe(1, -1, f, true, (placed[f].size() - 1) * doc.params.cell_mm,
                            count_turns(placed[f]), 0, 0.0, done, n, 1.0, &placed[f]);
            }
            if (!changed) break;
        }
    }

    // ---- C2 코너 최소반경 최종 패스(Phase C) ----
    // 모든 배관 라우팅·rip-up·CBS 가 끝난 뒤, 각 성공 배관의 짧은 단관(엘보 간 직선 < mult×관경)을 흡수한다.
    // **비교란**: 라우팅에 쓴 점유(work)를 건드리지 않고, 배관마다 '장애물 + 다른 배관(각 반경)'만으로 만든
    // 검사 점유(chk)에 대해 직교 흡수 → 다운스트림 배관 라우팅을 바꾸지 않는다(배치 상호작용으로 총 꺾임이
    // 늘던 문제 해소). 충돌검사 통과 + 꺾임 비증가 + 길이 비증가일 때만 채택(무손실·물리유효, 양 끝점 고정).
    // 결정적(placed 키 오름차순). 기본 eff_min_straight=0 → 미실행(골든 불변). 비용 O(성공수²)(opt-in 한정).
    if (eff_min_straight > 0.0 && cell_for_r > 0.0 && !aborted && !placed.empty()) {
        for (auto& kv : placed) {
            const int pi = kv.first;
            std::vector<Cell>& path = kv.second;
            const double d = doc.tasks[static_cast<size_t>(pi)].diameter_mm;
            const int min_run = (d > 0.0)
                ? static_cast<int>(std::ceil(eff_min_straight * d / cell_for_r)) : 0;
            if (min_run <= 1 || path.size() < 5) continue;
            // 검사 점유 = 장애물 + 다른 배관(각 per-task 반경). 자기 자신은 제외(자기 충돌 무의미).
            Occ chk = occ.copy();
            for (const auto& other : placed)
                if (other.first != pi) mark_pipe(chk, other.second, radius_of(other.first));
            std::vector<Cell> sp = enforce_min_straight(chk, path, min_run);
            if (sp.size() < path.size() ||
                (sp.size() == path.size() && count_turns(sp) < count_turns(path))) {
                // 더 짧거나(꺾임↓ 자동) 같은 길이라도 꺾임이 줄 때만 채택.
                if (count_turns(sp) <= count_turns(path)) {
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
}

// 격자 크기로 백엔드 선택(sparse 해법):
//   작은 격자(≤5M 셀, 골든 등) → DenseOccupancy: 기존 동작·바이트 결과 완전 불변.
//   거대 격자(>5M 셀, 25mm/10mm) → ImplicitOccupancy: 복셀화 없는 O(장애물) 저장 + 64비트 키 +
//   온디맨드 클리어런스 → 130MB/2GB 배열·520MB 거리변환·int 오버플로를 모두 회피.
void route_multi_into_doc(SceneDoc& doc, const std::string& priority, bool collect_visited,
                          const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr,
                          int pipe_radius = 0, bool per_task_radius = false,
                          int cbs_depth = 0, double min_straight_mult = 0.0,
                          double pipe_gap_mm = 0.0) {
    const long long cells =
        static_cast<long long>(doc.shape.i) * doc.shape.j * doc.shape.k;
    if (cells > 5000000LL) {
        route_multi_impl(doc, implicit_from_doc(doc), priority, collect_visited, on_pipe, seed,
                         pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm);
    } else {
        route_multi_impl(doc, occupancy_from_doc(doc), priority, collect_visited, on_pipe, seed,
                         pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm);
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

extern "C" R3dStatus r3d_set_task_diameter(R3dEngine* e, int32_t task, double diameter_mm) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    e->doc.tasks[static_cast<size_t>(task)].diameter_mm = diameter_mm > 0.0 ? diameter_mm : 0.0;
    return R3D_OK;
}

// 작업의 목표 진입축 제약(goal_dir) 설정 — A* 가 end_mm 에 'axis 방향(NEIGHBORS_6 인덱스 0..5 =
//   +x,-x,+y,-y,+z,-z)으로 진입할 때만' 도달 인정. 덕트 종단 스텁 리드인 축을 주면 일직선 진입(접속부
//   군더더기 꺾임 제거). 제약으로 막히면 엔진이 무제약 1회 폴백(연결 우선). axis 가 [0,5] 밖이면 -1(무제약).
extern "C" R3dStatus r3d_set_task_goal_dir(R3dEngine* e, int32_t task, int32_t axis) {
    if (!e) return R3D_ERR_ARG;
    if (task < 0 || task >= static_cast<int32_t>(e->doc.tasks.size())) return R3D_ERR_RANGE;
    e->doc.tasks[static_cast<size_t>(task)].goal_dir = (axis >= 0 && axis <= 5) ? axis : -1;
    return R3D_OK;
}

extern "C" R3dStatus r3d_route_multi(R3dEngine* e, const char* priority) {
    if (!e) return R3D_ERR_ARG;
    try {
        const std::vector<Cell>* seed = e->corridor_seed.empty() ? nullptr : &e->corridor_seed;
        route_multi_into_doc(e->doc, priority ? priority : "longest", e->collect_visited, {}, seed,
                             e->pipe_radius, e->per_task_radius, e->cbs_depth, e->min_straight_mult,
                             e->pipe_gap_mm);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// per-task 관경 반경(B1) 활성화 — ON 이면 route_multi 가 각 배관 diameter_mm 로 마킹 반경을 자동 산출
//   (호출자 글로벌 pipe_radius 책임 제거·가는 배관 과패킹 해소). OFF(기본)=글로벌 pipe_radius(기존 동작).
extern "C" R3dStatus r3d_set_per_task_radius(R3dEngine* e, int32_t enabled) {
    if (!e) return R3D_ERR_ARG;
    e->per_task_radius = enabled != 0;
    return R3D_OK;
}

// C1 negotiated-congestion(CBS-lite) 깊이 설정 — 0=OFF(평면 rip-up만·기존 동작·골든 불변). >0 이면 평면
//   rip-up 후 남은 실패 배관을 연쇄(재귀) rip-up 으로 해소(무손실·결정적). [0,3] 클램프. env R3D_CBS 도 가능.
extern "C" R3dStatus r3d_set_cbs_depth(R3dEngine* e, int32_t depth) {
    if (!e) return R3D_ERR_ARG;
    e->cbs_depth = depth < 0 ? 0 : (depth > 3 ? 3 : depth);
    return R3D_OK;
}

// C2 코너 최소반경 배수 설정 — 엘보 간 직선(런) ≥ (mult × 관경) 보장(제작성). 경로(셀) 단계에서 충돌검사
//   하에 짧은 단관을 흡수. 0=OFF(기존 동작·골든 불변). 권장 2.0. 음수면 0. env R3D_MIN_STRAIGHT 도 가능.
extern "C" R3dStatus r3d_set_min_straight(R3dEngine* e, double mult) {
    if (!e) return R3D_ERR_ARG;
    e->min_straight_mult = mult > 0.0 ? mult : 0.0;
    return R3D_OK;
}

// 배관-배관 이격(mm) 설정 — 두 배관 센터선 거리 ≥ r1 + r2 + gap 보장(표면 사이 최소 gap mm). 0=OFF(기존
//   동작·표면 맞닿음·골든 불변). 규격 60mm. route_multi 메인 루프가 깔린 배관을 쌍 반경으로 막는다. env R3D_PIPE_GAP.
extern "C" R3dStatus r3d_set_pipe_gap(R3dEngine* e, double gap_mm) {
    if (!e) return R3D_ERR_ARG;
    e->pipe_gap_mm = gap_mm > 0.0 ? gap_mm : 0.0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                              void* user) {
    if (!e) return R3D_ERR_ARG;
    try {
        ProgressCb on_pipe;  // cb 가 널이면 비활성(콜백 없는 route_multi 와 동일).
        if (cb) {
            on_pipe = [cb, user](int phase, int oi, int ti, bool ok, double len, int turns,
                                 long long exp, double ms, int done, int total, double prog,
                                 const std::vector<Cell>* path) -> int {
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
                return cb(user, phase, oi, ti, ok ? 1 : 0, len, turns, exp, ms, done, total, prog,
                          pptr, plen);
            };
        }
        const std::vector<Cell>* seed = e->corridor_seed.empty() ? nullptr : &e->corridor_seed;
        route_multi_into_doc(e->doc, priority ? priority : "longest", e->collect_visited, on_pipe, seed,
                             e->pipe_radius, e->per_task_radius, e->cbs_depth, e->min_straight_mult,
                             e->pipe_gap_mm);
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

// 배관 점유 팽창 반경(셀) 설정(옵션1, 배관-배관 충돌 회피). route_multi(_progress) 가 깔린 배관을
// mark_pipe(radius)로 막아 다음 배관 중심선을 띄운다. 음수면 0 으로 클램프. 0=기존 동작(경로 셀만).
extern "C" R3dStatus r3d_set_pipe_radius(R3dEngine* e, int32_t radius_cells) {
    if (!e) return R3D_ERR_ARG;
    e->pipe_radius = radius_cells > 0 ? radius_cells : 0;
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
        // 점유 백엔드 무관(템플릿) — 결과 추출을 제네릭 람다로. 대형 격자(>5M 셀)는 ImplicitOccupancy
        //   (복셀화 없음·O(장애물))로 전환해 Dense 전배열 폭발·int 오버플로를 방지한다(A3, route_multi 게이트 동일).
        auto run = [&](auto&& occ) {
            std::vector<int> order = order_indices(occ, doc.tasks, prio);
            auto mr = route_ripup(occ, doc.tasks, doc.params, prio, 0, 2, -1,
                                  max_rounds > 0 ? max_rounds : 10, max_ripup > 0 ? max_ripup : 4,
                                  e->collect_visited);
            doc.results.assign(doc.tasks.size(), std::nullopt);
            for (size_t pos = 0; pos < mr.pipes.size(); ++pos)
                doc.results[static_cast<size_t>(order[pos])] = to_scene_result(mr.pipes[pos].result);
        };
        const long long cells = (long long)doc.shape.i * doc.shape.j * doc.shape.k;
        if (cells > 5000000LL) run(implicit_from_doc(doc));   // 대형 = Implicit.
        else run(occupancy_from_doc(doc));                    // 소형 = Dense(골든 불변).
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
        const RouteTask& t = e->doc.tasks[static_cast<size_t>(task)];
        // 백엔드 선택(route_multi 와 동일 정책): 작은 격자(≤5M, 골든)는 DenseOccupancy 로 기존 동작·
        //   바이트 결과 완전 불변. 거대 격자(>5M, 25mm/10mm)는 복셀화 없는 ImplicitOccupancy(O(장애물)) +
        //   탐색 상한(12M, 메모리 폭증·런어웨이 방지)으로 전환 → C# 코너/복제 후처리의 단일 수리 A* 가
        //   1.3억 셀 격자에서도 매 호출 130M 복셀화 없이 빠르게 동작(그룹패턴/기존설계추종 후처리 활성화).
        const long long cells =
            static_cast<long long>(e->doc.shape.i) * e->doc.shape.j * e->doc.shape.k;
        SceneResult sr;
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
        const Cell& shape = e->doc.shape;
        bool size_only = (buf == nullptr || buf_cells <= 0);
        // 점유 백엔드 무관 스캔 — 대형 격자(>5M 셀)는 ImplicitOccupancy(전배열 미할당)로 is_blocked 질의해
        //   Dense 전배열(예 25mm ~1.3억 셀×4B) 메모리 폭발을 방지(A3). 소형은 Dense(기존 동작 동일).
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
            return written;  // size_only=true 면 전체 카운트, false 면 실제 복사한 셀 수.
        };
        const long long cells = (long long)shape.i * shape.j * shape.k;
        return cells > 5000000LL ? scan(implicit_from_doc(e->doc)) : scan(occupancy_from_doc(e->doc));
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
