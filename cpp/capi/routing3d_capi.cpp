// Routing3D ??�씠?곕툕 C ABI ?�ы쁽 (routing3d_capi) ??Phase 3
// =============================================================================
// [?????��????�뒗 ??
//   routing3d_capi.h ??C ABI ??C++ ?�붿�??붿쭊 ?꾩뿉 ??�쾶 ?�ы쁽??�떎. 紐⑤�?export ??�닔??
//   ??�쇅??寃쎄??諛뽰?�濡???�???? ??�룄�?try/catch �?媛먯???곹깭 ?�붾뱶濡?蹂닿???�떎.
//   ?붿쭊 ?곹깭(R3dEngine)??SceneDoc ??�굹�???�쁽??��? ??�슦?????�??留듭??利됱�??�ъ꽦??�떎.
//   ??��? docs/csharp_helix_interop_design.md, ??�뜑: capi/routing3d_capi.h.
//
// [??���?寃�?  (?꾨줈??�듃 ?�⑦??�?��)
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
#include "routing3d/octree_occupancy.hpp"
#include "routing3d/scene_io.hpp"
#ifdef ROUTING3D_USE_OPENVDB
#include "routing3d/vdb_occupancy.hpp"
#endif

using namespace routing3d;

// ?�덊?�紐??몃뱾????�젣 ?뺤쓽: ???�몄�???�굹(寃⑹?????��誘명�??μ븷臾??묒뾽/寃곌????蹂댁?�.
struct R3dEngine {
    SceneDoc doc;
    bool collect_visited = false;  // 湲곕??OFF(A2) ???????λ??硫붾?�由?蹂듭�???��??蹂댄?? 諛⑸Ц留돠룸떒???좊땲媛
                                   // ?꾩슂?????�� ?몄텧?�? r3d_set_collect_visited(1) �?opt-in.
    // ??�뒿?????�� ??(ijk) ??w_corridor>0 ????route_multi 媛 ??�뱶�??????諛곌???�??�곸?�濡??좊룄(L2b).
    // r3d_set_corridor_cells �???�젙/?�덇�?? ??���???�쑝�?湲곗????�옉(源붾??諛곌? ??踰덈뱾留곷쭔).
    std::vector<Cell> corridor_seed;
    // 諛곌?-諛곌? ?�⑸�???�뵾(???�?): 源붾??諛곌????�??�??�붽???????�갹 諛섍�???). 0=寃쎈�???�?湲곗??.
    // >0 ??�??mark_pipe 媛 寃쎈�?짹radius 6-??�썐??留됱�???�쓬 諛곌? 以묒??좎쓣 洹몃�???꾩슫??????�젣 ?�寃쎌?�濡?
    // ???��??��???�㈃??寃�??�吏? ??�뒗????�컖/?�쇰???�⑸�???�냼). r3d_set_pipe_radius �???�젙. ?��??? 湲곕�??곗텧??
    // ?몄텧???�곗�?BuildEngineForRows)媛 ??�뻾. env R3D_PIPE_RADIUS 濡쒕�??????媛????�뱶?�ъ뒪 A/B).
    int pipe_radius = 0;
    bool per_task_radius = false;  // B1 ??ON ??�??route_multi 媛 �?諛곌? diameter_mm �?諛섍�??�?�� ?곗텧.
    // 諛곌?-諛곌? ??�꺽(mm) ????諛곌? ??�꽣??嫄곕????r1 + r2 + pipe_gap_mm 蹂댁?? 0=湲곗????�옉(??�㈃ 留욌???�룰????�덈?).
    //   >0 ??�??硫붿???�⑦봽媛? 源붾??諛곌???routing 諛곌? 湲곗? ??諛섍�?ceil((r_a+r_b+gap)/cell))??�줈 留됰???per-pipe
    //   ?????. r3d_set_pipe_gap. 洹쒓�? ??�㈃ ????理쒖??60mm ?�?. env R3D_PIPE_GAP.
    double pipe_gap_mm = 0.0;
    // C1 negotiated-congestion(CBS-lite, Phase C) ???곗뇙(???) rip-up 理쒕? 源딆?? 0=OFF(??�㈃ rip-up�?
    //   湲곗????�옉쨌怨⑤�??�덈?). >0 ??�????�㈃ rip-up ????? ??�뙣 諛곌??? blocker 媛 ??같移?�???�㈃ �?blocker ??
    //   blocker 源뚯? ??源딆?�留?�겮 ????곸쑝�??묐낫??�폒 ??�냼(?�댁?�??�룰�?뺤쟻). r3d_set_cbs_depth / env R3D_CBS.
    int cbs_depth = 0;
    // C2 ?�붾�?理쒖?�諛?�꼍(Phase C) ????�낫 ????吏곸�?????(mult ???��? 誘몃�??�????�옉 ?�덇? ??寃쎈�???) ??��?�?��
    //   ?묒쁿 ?�붾꼫瑜??�⑸�??�뒗 吏곴???곌껐�???�닔????�븻???�⑸룎寃??????��쨌爰?�엫 ??��쬆媛??????��, ????�젏 ?�좎??. 0=OFF
    //   (湲곗????�옉쨌怨⑤�??�덈?). 沅뚯??2.0(??�낫 �?吏곸�???2?�愿?�?. r3d_set_min_straight / env R3D_MIN_STRAIGHT.
    double min_straight_mult = 0.0;
    // 코너 최소직선(절대 mm, 하드 제약). >0 이면 A* 가 '한 번 꺾인 뒤 이 길이만큼 직진하기 전엔 다시 꺾지
    //   못하도록' 강제한다(상태에 진행 셀 수 run 추가). min_straight_mult(관경 배수·후처리 흡수)와 달리
    //   탐색 단계의 하드 보장이며 관경 무관·전 배관 적용(목표 직전 마지막 구간은 면제). 셀로는
    //   ceil(min_straight_mm/cell)→params.min_straight_cells. 0=OFF(골든 불변). r3d_set_min_straight_mm.
    double min_straight_mm = 0.0;
    R3dRuntimeOptions runtime{};
};

namespace {

// ??�꼍蹂??�뿉???묒쓽 long long ??�룄????�뒗??誘몄�??0??�븯/???��??�뙣�?def). 嫄곕?寃⑹??25mm ?? ?�?�� ?곹븳??
// 32GB+ ??�쾭?�?�� ??�썙 ??�???諛곌? ?�ㅻ�?��?????믪씠????�룄 ??硫붾?�由????뺤옣 ?몃뱶 ??�떆�?g/came/closed)??
// ??��????�?RAM ???�媛 ??�쓣 ???�� ????? ?�? 寃⑹???�⑤�????좎큹???�댁???-1)??�???곹뼢 ??�쓬.
long long env_ll(const char* name, long long def) {
    if (const char* s = std::getenv(name)) {
        char* end = nullptr;
        long long v = std::strtoll(s, &end, 10);
        if (end != s && v > 0) return v;
    }
    return def;
}

// 嫄곕?寃⑹???�?�� ?곹븳(硫붾?�由??곗뼱??�씠 蹂댄??. 湲곕??48M(??12M, 32GB+ ??�쾭 湲곗? ?곹뼢 ??25mm ?�? 寃⑹???
// 留됲????�옟 諛곌?????源딆???�?��???깃났??�넁; ??踰덉�???諛곌?�??�?��???�???�겕 硫붾?�由?????�룄?�꾩????�떆�?.
// env R3D_MAX_EXP �??�붽? ????? (12M ?? 吏㏃? 嫄곕??#146 2,277mm�?1??]?몃뜲????�옟 ?�낅???????�???��?// ?꾨떖??�뜕 ??��??????25mm + pipe_radius ??�갹??�줈 留덉?�?諛곌? 吏꾩??��?? ?�곸븘吏?寃쎌??)
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

// std::string ??malloc 踰꾪???�쒕???좊떦). r3d_free_string ??�줈 ??�젣.
char* dup_string(const std::string& s) {
    char* p = static_cast<char*>(std::malloc(s.size() + 1));
    if (!p) return nullptr;
    std::memcpy(p, s.c_str(), s.size() + 1);
    return p;
}

// const char* ??optional<string>. ?�?���?None(=\N), ?꾨땲�??�몄?????��Ц?�?�� ??�슜).
std::optional<std::string> opt_str(const char* s) {
    if (!s) return std::nullopt;
    return std::string(s);
}

// AStarResult ??SceneResult(?붿쭊 寃곌????????�쐞). ?깃났 ??寃쎈�???�? visited 媛 ??���???
// ??�쑝�???�퍡 蹂듭�?媛??�솕 '諛⑸Ц�? / scene.txt [visited] ?뱀??.
SceneResult to_scene_result(const AStarResult& r) {
    SceneResult s;
    s.success = r.success;
    s.length_mm = r.length_mm;
    s.cost_mm = r.cost_mm;
    s.turns = r.turns;
    s.expanded_nodes = r.expanded_nodes;
    s.elapsed_ms = r.elapsed_ms;
    s.fail = static_cast<int>(r.fail);   // ??�뙣 ???�(A1) ?꾨떖.
    if (r.success) s.path = r.path;
    if (!r.visited.empty()) s.visited = r.visited;
    return s;
}

// optional<SceneResult> ??R3dResult(POD). ??�쑝�?0??�줈.
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
    o.fail_reason = static_cast<int32_t>(r->fail);   // ??�뙣 ???�(A1, RouteFail). ?깃났=0.
}

// 嫄곕? 寃⑹??�?�� 蹂듭?????�씠(O(?μ븷臾???) ?�??????�쁽??�뒗 ImplicitOccupancy ??doc 濡쒕????�ъ꽦.
// ?? ??�?? ?�닿???硫붾?�由???25mm/10mm ???�? 寃⑹?????????�???�쾭???��??洹쇰????�냼(S3).
ImplicitOccupancy implicit_from_doc(const SceneDoc& doc) {
    ImplicitOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;  // ?�?�� 0(??�솕) 諛뺤???嫄�?�??�??occupancy_from_doc ????�씪).
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
// 嫄곕?寃⑹???κ굅由?諛곌? 媛??�슜 coarse ?�??�?factor �???). ??�씪 origin쨌諛뺤뒪, ??�?factor �??�듦�?
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

// ?�꾩�?corridor ??�슦????coarse 媛??�뱶濡?fine ?�?��??tube(coarse 寃쎈�?짹radius ??�갹)�???�뱶 ??�븳??�떎.
// 嫄곕?寃⑹???κ굅由?諛곌????�?��??�쓣 ??�?以꾩??? **??��?�紐?�뜽?? fine ????�씪**(weighted A* + ??�???�???+
// ????�꼸????�??寃쎈�???�쭏 蹂댁?? 媛??��???�뙣/??�툕 ??寃쎈�???�쓬 ??false 諛섑???몄텧?�? ?�댁???fine ??�줈
// ??��????깃났 ????? 0). work=源붾?�諛�? ??�?fine ?�??(?�⑸�??�뵾 ?�?).
template <class Occ>
bool route_hier(const Occ& work, const ImplicitOccupancy& coarse, int factor, int radius,
                Cell s, Cell g, const RouteParams& params, long long max_exp,
                bool collect_visited, AStarResult& out) {
    auto to_coarse = [factor](const Cell& c) {
        return Cell{c.i / factor, c.j / factor, c.k / factor};  // i>=0 ??諛붾????�닓??
    };
    // fine ?�낅???coarse ???? ?λ???뺥듃 洹쇱�??coarse(?�듭?) ??�긽?꾩뿉??留됲? ??�쓣 ????�떎 ???�?? coarse
    // ??�???�깄??媛??�뱶媛? ??�옉/?꾩갑??�쾶 ??�떎. ??�깄??�줈 ??�릿 ?�낅??�?? ?꾨옒 ?곌껐 諛뺤?�濡???�툕????�?
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

    // 2) ??�툕(coarse ?? ??吏묓빀) = coarse 寃쎈�?짹radius ??�갹 + ??????�젣 fine ?�낅??붿뒪??coarse) ?곌껐 諛뺤??
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
    // ?�낅???곌껐 諛뺤??to_coarse(?�낅???붿뒪??coarse, 짹radius) ??fine ?�낅??????諛섎�????�툕????�룄�?
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

    // 3) fine A* ??fine ????coarse ??????�툕????�쓣 ???�� ?뺤옣(??�뱶 ??�븳). ??��?�紐?�뜽 ??�씪(??�쭏 蹂댁??.
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

// 吏꾪�??�쒕�???????�???. phase=0(?�?�� 吏꾪�?/1(諛곌? ?꾨즺). 諛섑??0 ?�꾩?? 0?꾨떂=?�⑥??abort).
//   ?몄옄: phase, order_index, task_index, success, length_mm, turns, expanded_nodes, elapsed_ms,
//         done, total, progress01, path(?꾨즺�?깃났 ??寃쎈�???, ?꾨땲�?nullptr).
using ProgressCb = std::function<int(int, int, int, bool, double, int, long long, double, int, int,
                                     double, const std::vector<Cell>*)>;

// ---------------------------------------------------------------- 寃쎈�??꾩쿂?? ???��/??�????�굅
// A?�???吏곴??吏곸�??�?�� ??�씪 ??�낫)�???�뒗 ????�쓣 axisOrder ??�꽌�???�꽦(??�젏 ??�?.
// ????�Ⅸ ?�뺤? ?�?��??嫄�?�??�????��??0).
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

// A ?? B ??吏곴??????�낫(?�뺤�???) �???�뒗 ?�⑸�??�뒗 ????�쓣 李얜????�뺤????꾨낫 ?꾩닔). 3??李⑥?????�뙣.
template <class Occ>
bool ortho_connect(const Occ& occ, Cell A, Cell B, std::vector<Cell>& out) {
    int axes = (A.i != B.i ? 1 : 0) + (A.j != B.j ? 1 : 0) + (A.k != B.k ? 1 : 0);
    if (axes > 2) return false;                       // 2??�낫(3??????�텞 ???곸뿉????�쇅(蹂댁???.
    static const int orders[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    for (const auto& ord : orders) {
        std::vector<Cell> v = walk_order(A, B, ord);
        bool clear = true;
        for (const Cell& c : v) if (occ.is_blocked(c)) { clear = false; break; }
        if (clear) { out = std::move(v); return true; }
    }
    return false;
}

// 寃쎈�?�?�� ??�?????�� ??�굅: ??�뼱�???寃쎈�?�?�� '??吏㏃? 吏곴???곌껐(?�⑸�??�쓬)'�???泥댄�??洹몃?????�텞.
// occ(?μ븷臾???�? 源붾??諛곌?) ?�⑸룎留?寃????寃곌?????�??�쇰????좏슚. mark_pipe ?꾩뿉 ?곸슜???�??꾩냽
// 諛곌?????�텞寃쎈줈瑜???�뵾(M1/M2 蹂댁??. 寃곗???媛????j ?곗꽑쨌怨좎???�뺤???. 湲몄?�媛? �????�� ??�??�댄븳猷??�� 李⑤??.
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
        for (int j = n - 1; j >= a + 2; --j) {   // 媛????j ?곗꽑(理쒕? ??�텞).
            std::vector<Cell> seg;
            if (!ortho_connect(occ, path[static_cast<size_t>(a)], path[static_cast<size_t>(j)], seg))
                continue;
            const int segSteps = static_cast<int>(seg.size()) - 1, origSteps = j - a;
            if (segSteps > origSteps) continue;   // 湲몄뼱吏?�?湲곌�?
            if (segSteps == origSteps) {
                // 媛숈? 湲몄?�硫??�얠??????곸쓣 ???�� ??�???�㈃ ?깅땲/吏洹몄?�洹??뺣━).
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

// ?�붾�?理쒖?�諛?�꼍(C2, Phase C): ??�낫 ????吏곸�?????min_run_cells 誘몃�??�????�옉 ?�덇?(吏㏃? ???) ??�?吏㏃?
// ?곗쓣 媛濡쒖�???묒쁿 ?�붾꼫瑜?'?�⑸�??�뒗 吏곴??????�낫) ?곌껐'(ortho_connect)�???�닔????�븻?? occ(?μ븷臾???�?
// 源붾??諛곌?) ?�⑸룎寃??????�� + ?�얠????��쬆媛? + 湲몄????��쬆媛??????�� ??�??�댁?�??�룸Ъ?�ъ�???. ????�젏(PoC/??��???
// ?�좎???�붾�??꾨낫?�?�� ??�쇅). 寃곗????�붾�???�쫫李⑥?�쨌??踰덉�???�굹, 蹂�????�붾�??????. min_run_cells<=1 ??�??// ?�?�� 洹몃?�?諛섑???�⑤�?湲곗????�옉 ?�덈?). PathRectifier(???�� ??�꺼, ??�룎???? ????寃쎈�??? ??��???�⑸�???�쟾.
template <class Occ>
std::vector<Cell> enforce_min_straight(const Occ& occ, const std::vector<Cell>& path, int min_run_cells) {
    if (min_run_cells <= 1 || path.size() < 5) return path;
    std::vector<Cell> cur = path;
    bool changed = true;
    int guard = 0;
    while (changed && guard++ < 64) {   // guard=?�댄븳猷??�� 李⑤??�?諛섎??1�???�닔 ??理쒕? ?�붾�???�쭔??.
        changed = false;
        // ?�붾�??몃뜳??諛⑺�??꾪솚?? ??�쭛 ??[0, ?꾪솚?�?��?? n-1]. ????0,n-1)?? ?�좎???
        std::vector<int> corners;
        corners.push_back(0);
        for (size_t m = 1; m + 1 < cur.size(); ++m) {
            Cell d0{cur[m].i - cur[m - 1].i, cur[m].j - cur[m - 1].j, cur[m].k - cur[m - 1].k};
            Cell d1{cur[m + 1].i - cur[m].i, cur[m + 1].j - cur[m].j, cur[m + 1].k - cur[m].k};
            if (!(d0 == d1)) corners.push_back(static_cast<int>(m));
        }
        corners.push_back(static_cast<int>(cur.size()) - 1);
        // ?몄젒 ?�붾�????湲몄?�瑜?蹂닿?? 吏㏃? ??�? ?곗쓣 ?묒쁿 ?�붾�?ci-1, ci+1) 吏곴??곌껐�???�닔.
        for (size_t ci = 1; ci + 1 < corners.size(); ++ci) {
            const int runNext = corners[ci + 1] - corners[ci];      // ci~ci+1 ???? ??.
            const int runPrev = corners[ci] - corners[ci - 1];      // ci-1~ci ???? ??.
            if (runNext >= min_run_cells && runPrev >= min_run_cells) continue;  // ?????�⑸??
            const int a = corners[ci - 1], b = corners[ci + 1];
            std::vector<Cell> seg;
            if (!ortho_connect(occ, cur[static_cast<size_t>(a)], cur[static_cast<size_t>(b)], seg))
                continue;                                            // ?�⑸�??�?�� 3?�뺤�?????�닔 ?�덇?.
            std::vector<Cell> slice(cur.begin() + a, cur.begin() + b + 1);
            if (count_turns(seg) > count_turns(slice)) continue;     // ?�얠??利앷? 湲덉?.
            if (static_cast<int>(seg.size()) - 1 > b - a) continue;  // 湲몄??利앷? 湲덉?.
            std::vector<Cell> next(cur.begin(), cur.begin() + a);    // [0..a) + seg + (b..end].
            for (const Cell& c : seg) next.push_back(c);
            for (size_t t = static_cast<size_t>(b) + 1; t < cur.size(); ++t) next.push_back(cur[t]);
            cur = std::move(next);
            changed = true;
            break;   // ?�붾�????????�닔�??몃뜳??? 諛붾??.
        }
    }
    return cur;
}

// ??�쨷 諛곌? ??�감 ??�슦??�쓽 諛깆�???�닿? 蹂몄�?Occ = Dense/Implicit). order/snap/astar/mark_pipe ??�씪.
// 寃곌?�瑜?'?�?�� ?묒뾽 ?몃뜳?? ?????ν�??몃뱾 API(get_result(task)) 留ㅽ�??蹂댁???�떎.
// on_pipe 媛 ?좏슚??�㈃ 諛곌?留덈???몄텧(吏꾪�???�씠??�줈洹몄?? ??寃곌????�꽌?�?�� ?곹뼢 ??�쓬.
template <class Occ>
void route_multi_impl(SceneDoc& doc, Occ occ, const std::string& priority, bool collect_visited,
                      const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr,
                      int pipe_radius = 0, bool per_task_radius = false,
                      int cbs_depth = 0, double min_straight_mult = 0.0,
                      double pipe_gap_mm = 0.0,
                      const R3dRuntimeOptions* runtime = nullptr) {
    const long long large_threshold = runtime ? opt_or_default(runtime->large_grid_threshold, 5000000LL) : 5000000LL;
    const long long configured_max_exp = runtime ? runtime->max_expansions : 0;
    const long long configured_fallback_exp = runtime ? runtime->fallback_expansions : 0;
    const int configured_hier_factor = runtime ? runtime->hier_factor : 0;
    const int configured_hier_radius = runtime ? runtime->hier_radius : 0;
    const long long configured_hier_probe = runtime ? runtime->hier_probe : 0;
    const int configured_ripup_enabled = runtime ? runtime->ripup_enabled : -1;

    Occ work = occ.copy();  // ?�?�� ?�?? ?�덈?(M2).
    // C1 CBS 源딆???곗뇙 rip-up) ???몄옄 ?곗꽑, env R3D_CBS(>=0) 媛 ??�쑝�?????? 0=OFF(??�㈃ rip-up留뙿룰낏???�덈?).
    int eff_cbs_depth = cbs_depth < 0 ? 0 : cbs_depth;
    if (const char* cs = std::getenv("R3D_CBS")) {
        char* end = nullptr; long v = std::strtol(cs, &end, 10);
        if (end != cs && v >= 0) eff_cbs_depth = static_cast<int>(v);
    }
    if (eff_cbs_depth > 3) eff_cbs_depth = 3;   // ?�꾧�???�?李⑤???�꾧�???(MAXBLK+1)^(depth+1)).
    // C2 ?�붾�?理쒖?�諛?�꼍 諛곗????�낫 �?吏곸�???mult?�愿?�? ???몄옄 ?곗꽑, env R3D_MIN_STRAIGHT(>=0) ????? 0=OFF.
    double eff_min_straight = min_straight_mult < 0.0 ? 0.0 : min_straight_mult;
    if (const char* ms = std::getenv("R3D_MIN_STRAIGHT")) {
        char* end = nullptr; double v = std::strtod(ms, &end);
        if (end != ms && v >= 0.0) eff_min_straight = v;
    }
    // 諛곌? ?�?? ??�갹 諛섍�????�?, 諛곌?-諛곌? ?�⑸�???�뵾). ?몄옄 ?곗꽑, env R3D_PIPE_RADIUS(>=0) 媛 ??�쑝�??????
    // (??�뱶?�ъ뒪 --dbroute A/B ??. 0=寃쎈�???�?湲곗????�옉쨌怨⑤�??�덈?).
    int eff_pipe_radius = pipe_radius < 0 ? 0 : pipe_radius;
    if (const char* pr = std::getenv("R3D_PIPE_RADIUS")) {
        char* end = nullptr;
        long v = std::strtol(pr, &end, 10);
        if (end != pr && v >= 0) eff_pipe_radius = static_cast<int>(v);
    }
    // per-task ?��?諛섍�?B1) ??ON ??�??�?諛곌???diameter_mm �?諛섍�???�?�� ?곗텧(?몄텧??�?��????�굅, 媛??諛곌?
    //   ?�쇳?????�냼). OFF(湲곕?? ?�?�� ?��?誘몄�??�??湲濡쒕�?eff_pipe_radius ??��???湲곗????�옉쨌怨⑤�??�덈?.
    //   env R3D_PER_TASK_RADIUS 濡쒕�???????�떎(??�뱶?�ъ뒪 A/B).
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
        return eff_pipe_radius;   // OFF/?��?誘몄�???湲濡쒕�?
    };
    // ???? 諛곌?-諛곌? ??�꺽(??�꽣??嫄곕????r1 + r2 + gap, 湲곕??60mm) ????
    // 湲곗??留덊�?radius_of ??ceil(d/cell)-1)?? ??�꽣?좎쓣 '???��?d)'留뚰�?��??꾩썙 ??諛곌? '??�㈃?????�숇???
    // (gap=0). 洹쒓�?? ??諛곌? 諛섍�??+ ???�(60mm). gap>0 ??�??**硫붿???�⑦�?�?�� 源붾??諛곌???routing 諛곌? 湲곗?
    // ??pairwise) 諛섍�?= ceil((r_a + r_b + gap)/cell) ??�줈 留됱�?* ??�쓬 諛곌? ??�꽣?좎쓣 ?뺥솗??r_a+r_b+gap 留뚰�?    // ?꾩슫??per-pipe ?????. gap=0(湲곕????�??湲곗??利앸??留덊�??�⑤�?湲곗????�옉 ?�덈?). ?몄옄 ?곗꽑쨌env R3D_PIPE_GAP.
    double eff_gap_mm = pipe_gap_mm < 0.0 ? 0.0 : pipe_gap_mm;
    if (const char* pg = std::getenv("R3D_PIPE_GAP")) {
        char* end = nullptr; double v = std::strtod(pg, &end);
        if (end != pg && v >= 0.0) eff_gap_mm = v;
    }
    const bool use_gap = eff_gap_mm > 0.0 && cell_for_r > 0.0;
    const int PAIR_RADIUS_MAX = 24;   // ??諛섍�??곹븳(嫄곕? ?��?gap ??�?李⑤??.
    // ?��?諛섍�?mm) ??per_task & ?��????�� d/2, ?꾨땲�?湲濡쒕�?諛섍�???)??mm �???�궛.
    auto rmm_of = [&](int ti) -> double {
        if (eff_per_task && ti >= 0 && ti < static_cast<int>(doc.tasks.size())) {
            double d = doc.tasks[static_cast<size_t>(ti)].diameter_mm;
            if (d > 0.0) return d * 0.5;
        }
        return eff_pipe_radius * cell_for_r;
    };
    // ??pairwise) 留덊�?諛섍�???): 源붾??a ??routing b 湲곗???�줈 留됱????= ceil((r_a + r_b + gap)/cell).
    auto pair_radius = [&](int a, int b) -> int {
        double sep = rmm_of(a) + rmm_of(b) + eff_gap_mm;
        int r = static_cast<int>(std::ceil(sep / cell_for_r));
        if (r < 0) r = 0; if (r > PAIR_RADIUS_MAX) r = PAIR_RADIUS_MAX;
        return r;
    };
    // per-task ?��?clearance(B2) ??per_task 媛 ON ??��?w_clear>0 ??�?? �?諛곌????��?諛섍꼍留?�겮 �??μ븷臾??�?��
    //   以묒??좎쓣 ?꾩슦?꾨줉 clearance_radius ?꾧퀎瑜?max(湲곗?? 諛섍�???�줈 ??????�듭? 諛곌???踰쎌�???�㈃??諛뺤? ??�쾶).
    //   諛섍�???湲곗??clearance_radius(媛??諛곌?)嫄곕�?OFF�?doc.params 洹몃?�???湲곗????�옉쨌怨⑤�??�덈?.
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
        // ??�젰 ??�꽌 ?�?.
    } else if (priority == "shortest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) < dist(b); });
    } else if (priority == "longest") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return dist(a) > dist(b); });
    } else if (priority == "diameter") {
        // ?�듭? 諛곌? ?�쇱?(??�쪧?? 嫄곕??�?�??�쇱?) ???�듭? 諛곌???理쒕??吏곸�? 寃쎈줈瑜??좎젏??��?媛??諛곌???
        // �??�곸????�븯�???�떎. ?��?誘몄�?0)??�?????묒뾽 ??�쪧 ??longest ?? ??�씪(湲곗????�옉 ?�덈?).
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            if (dia(a) != dia(b)) return dia(a) > dia(b);
            return dist(a) > dist(b);
        });
    } else if (priority == "utility") {
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            const std::string la = doc.tasks[static_cast<size_t>(a)].utility_label();
            const std::string lb = doc.tasks[static_cast<size_t>(b)].utility_label();
            if (la != lb) return la < lb;
            if (dia(a) != dia(b)) return dia(a) > dia(b);   // ?좏떥 ?�띠????�뿉???�듭? 諛곌? ?�쇱?.
            return dist(a) > dist(b);
        });
    } else {
        throw std::invalid_argument("unknown priority: " + priority);
    }

    // ???�� ?몃젰(params.w_corridor>0)??�??源붾??諛곌? ?�곸?????��??�줈 ??�썙 ??�쓬 諛곌??????��紐⑥???
    // ??湲곗????�퀎泥?�읆 ?�듭????�쑝�??�됱?��??�닿??湲몄?�媛? ??�뼱??�떎. 0??�??湲곗????�옉????�씪.
    doc.results.assign(static_cast<size_t>(n), std::nullopt);
    std::unordered_set<long long> corridor;
    const bool use_corridor = doc.params.w_corridor > 0.0;
    const int corridor_radius = doc.params.corridor_radius > 0 ? doc.params.corridor_radius : 1;
    // ??�뒿?????�� ??�뱶(L2b) ??w_corridor>0 ?????�? 二쇱????(seed)?????��??誘몃???ｌ뼱, 諛곌???
    // �??�곸??'?멸쾶'(w_corridor 硫댁?? 吏????꾨줉 ?좊룄(湲곗???��???��????뺤긽 ?곕씪媛�?. 0??�???�댁??
    if (use_corridor && seed) {
        for (const Cell& c : *seed)
            if (work.in_bounds(c)) corridor.insert(static_cast<long long>(work.lin(c)));
    }
    // ????寃⑹????25mm�?.3????)?�?��??寃쎈줈媛? ??�뒗/留됲??諛곌????꾨떖 媛?ν�??????�? ?뺤옣??
    // g/came 留듭????GB �???�???硫붾?�由??�좉�?0xC0000005). ?�?�� ?곹븳????洹몃??諛곌???議곌�??�낅�??�떎.
    // ?�? 寃⑹???�⑤�??????곹븳 ??�쓬(-1) ??�줈 湲곗????�옉쨌寃곗젙??蹂댁??
    const long long max_exp = (occ.size() > large_threshold)
        ? opt_or_default(configured_max_exp, large_grid_cap()) : -1;
    // ?�?�� 吏꾪�??蹂닿??媛꾧�??뺤옣 ??. ??��????�硫??�쒕�???�???5留뚮�??諛곌?????�떗 ??.
    const long long progress_every = on_pipe ? 50000LL : 0;
    // ?�꾩�?corridor 媛??嫄곕?寃⑹????�???諛곌?) ??coarse 媛??�뱶濡?fine ?�?��????�툕????�젙???�?��??�쓣 以꾩???
    // (??�쭏�?깃났??蹂댁??. **escalation 寃뚯???*: ?�쇱? ????�궛(HIER_PROBE) 吏곸??A* ?????�� ???�??????諛곌?�?    // 媛쒕�???吏곸�??? ??��?�寃??깃났??�궎?? ??�궛???�덇???�뒗 '??�???諛곌?'�??�꾩�?corridor �?????꾪븳??
    //   (嫄곕??湲곕�?寃뚯??몃뒗 �?吏곸꽑源?? hier �?蹂�?�???�???cell=50 ??�뀅ON 134ms??4s. probe 湲곕�????�떎.)
    // ?�? 寃⑹???�⑤�???誘몄??????�⑤�?湲곗????�옉 ?꾩쟾 ?�덈?.
    // 洹몃�????�� 紐⑤�?use_corridor)?�?��??hier ???�좊?????�쇨�???�먯??? 洹몃??��???�???諛곌?(??#146)??
    // ??�씪 bounded weighted A* 留뚯?�濡???�옟 ?�낅???�???��??곹븳 ?꾨떖 ??�뙣??�떎. probe(????�궛 吏곸??A*)??
    // ???�� 諛붿???�뒪??洹몃?�??곌�?????諛곌? 踰덈�??�?), ??�궛 ?�덇?????�???諛곌?�??�꾩�?corridor �?
    // escalate(?곌껐 ?곗꽑, ??�봽??諛붿???�뒪 ??�씠 ??�툕 ??�젙) ??踰덈�?蹂댁??+ ??�???諛곌? ?�ъ젣 ?묐┰.
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
    const long long HIER_PROBE = opt_or_default(configured_hier_probe, 300000LL);     // 吏곸??A* ????�궛 ???�덇????�???諛곌?)�?hier �?escalate.
    // probe(300k)쨌hier(??�툕 ??�젙) ??????�뙣????�???諛곌???'?�댁?????��? ??�궛.
    //   湲곕??= max_exp(=12M, ?�댁?�??: ????��?? ??�젣�?寃쎈줈瑜??�ъ젣??�떎 ????��? cell=50 ALL ?�?�� ??��??
    //   2M �??�?���??깃났 146??33(?깃났 諛곌? ?뺤옣??? 1.9M~11.9M 源뚯? ?곗냽 ?�꾪�? 吏꾩�???�뙣 12M ????�꽎??.
    //   �?'??�깂????�궛 ??�컧'?? 諛섎�????�젣 寃쎈줈瑜??껊뒗????�룄/?�ㅻ�?��?? ?몃젅??��??�봽). 洹몃???湲곕??? ?�댁?�??
    //   ??env R3D_FALLBACK_EXP=N(>0) �???�룄?곸쑝�???????�컙????�쓣 ????�떎(??�옟 諛곌? ?�ㅻ�?��?? ??? ??�?.
    //     0/誘몄�??= max_exp(?�댁?�??湲곕??. 以꾩??????�? 'hier ??�뙣 ????��?�????�?寃⑹??corridor �??�?�� ?�덈?.
    long long fallback_exp = configured_fallback_exp > 0 ? configured_fallback_exp : max_exp;
    if (const char* fe = std::getenv("R3D_FALLBACK_EXP")) {
        char* end = nullptr; long long v = std::strtoll(fe, &end, 10);
        if (end != fe && v > 0) fallback_exp = (max_exp > 0) ? std::min(v, max_exp) : v;
    }
    std::optional<ImplicitOccupancy> coarse;   // �???�???諛곌??�?�� 1??吏????�꽦.
    // (??�┰ 諛곌? 蹂묐?????�룄쨌湲곌컖: optimistic 蹂묐??A*+??�감 ?�⑸�?蹂듦?????�감?? 諛붿?????�씪??�쑝???뺥솗),
    //  project6 c100/c25/c10 ?�? wall-clock ??��?0~???��???? 誘몄�?��?�옄 A* ??嫄곕? ??�떆留듭????�듃?�щ컢??�뒗
    //  硫붾?�由????諛붿???�씪 ??�젅??�뱾????????寃�?빀??��? Phase A 媛 '留덊�???�뒗' ?????�?��??以묐????�뻾??
    //  蹂묐????��???곸뇙. ???꾩엯 蹂�?�? ??�감 ?�?. ?�?��??痢≪??? CLAUDE.md '??�쓬 ?묒뾽 ?꾨낫'.)
    int done = 0;
    bool aborted = false;   // on_pipe 媛 0?꾨떂(?�⑥????諛섑???�㈃ set ???꾩옱 諛곌? ?�?�� 以묐??+ 諛곗???�⑦�??�낅�?
    std::map<int, std::vector<Cell>> placed;   // ?깃났 諛곌? oi?믨꼍�?rip-up ???��?? ????�쫫李⑥??寃곗???.
    for (int oidx = 0; oidx < static_cast<int>(order.size()); ++oidx) {
        const int oi = order[static_cast<size_t>(oidx)];
        const RouteTask& t = doc.tasks[static_cast<size_t>(oi)];
        // ??�꺽 �?use_gap) 紐⑤�???源붾??諛곌???'routing 諛곌?(oi) 湲곗? ??諛섍�???�줈 ??�떆 留됱�???�꽣??嫄곕?�瑜?
        //   ?뺥솗??r_a + r_b + gap ??�줈 蹂댁???�떎(per-pipe ?????. gap=0 ??�???꾩뿉??留뚮�?利앸??work ??洹몃?�???�??
        if (use_gap) {
            work = occ.copy();
            for (const auto& kv : placed) mark_pipe(work, kv.second, pair_radius(kv.first, oi));
        }
        // ?�낅????�깄 諛섍�???湲곕??2. 諛곌? ??�갹(eff_pipe_radius>0)???곕㈃ ??諛곌????몄젒 ?�낅????源뚯? 留됱�?        // (?�듭????�룰???PoC) ?�낅????�삵? exp=0 利됱????�뙣媛 ??�떎 ????�깄 諛섍�????�갹?�꾨�????�썙 ?�낅???
        // ?�????�???�텧??�쾶 ??�떎(媛??媛源뚯???�???? ?좏깮??�???꾩튂 ??�끝 理쒖??. radius=0 ??�??湲곗??2) ??�씪.
        // use_gap ??�??源붾??諛곌?????諛섍�???????�줈 留됲? ??�뼱, ?�낅???�??뺤옣?곸뿭??踰쀬뼱??�룄�???�깄 諛섍�??
        //   ???�?��諛섍�?ceil((2r+gap)/cell))留뚰�???�슫??洹쇱??PoC 媛 ?�삵? ??�뙣??? ??�쾶). gap=0 ??�??湲곗??2+radius).
        const int snap_r = use_gap ? 2 + pair_radius(oi, oi) : 2 + radius_of(oi);
        Cell s = snap_to_free_cell(work, work.to_cell(t.start_mm), snap_r);
        Cell g = snap_to_free_cell(work, work.to_cell(t.end_mm), snap_r);
        const RouteParams tp = params_for(oi);   // per-task ?��?clearance(B2) 諛섏??OFF/媛?�?=doc.params ??�씪).

        // ?�?�� �?吏꾪�??泥섎??곹깭 %) ?�쒕�???phase=0. ?꾩옱 諛곌???order/task ?몃뜳??�줈 ??�쓣 李얜???
        // ?�쒕�???�⑥??0?꾨떂)??諛섑???�㈃ aborted ???몄슦??true 諛섑????astar 媛 ?�?�� ?�⑦봽瑜?利됱???�낅�?
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
        // ??��??꾨옒 !routed) ??�궛 ??湲곕??? max_exp(?�?寃⑹??corridor �??�?��?? ?�댁???蹂댁??. hier 源뚯? ??�뙣??
        // ??�???諛곌?�?bounded(fallback_exp)�???�뼱 ??�컙????�빟??�떎(?깃났??蹂댁?? ???곸닔 二쇱�?李몄??.
        long long fb_exp = max_exp;
        if (use_hier) {
            // 1) ????�궛 吏곸??A* ??????諛곌?(媛쒕�???吏곸�????? ??�????��?�寃??깃났(hier ??�쾭??�뱶 ??�쓬).
            //    ???�� 紐⑤뱶硫?probe ?????�� 諛붿???�뒪??�?????諛곌? 踰덈�???�???�떎(??�???諛곌?�??꾨옒�?escalate).
            const long long probe = (max_exp > 0) ? std::min(HIER_PROBE, max_exp) : HIER_PROBE;
            res = astar_weighted(work, s, g, tp, probe, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 on_pipe ? &intra : nullptr, progress_every, AllowAll{}, t.goal_dir);
            if (res.success && !res.path.empty()) {
                routed = true;                       // ????諛곌? ??吏곸??理쒖??寃쎈�?�?���?
            } else if (res.expanded_nodes >= probe) {
                // 2) ????�궛 ?�덇????�???諛곌?) ???�꾩�?corridor(coarse 媛??��?????�툕 ??�젙)�??????
                if (!coarse) coarse.emplace(coarse_implicit_from_doc(doc, HIER_FACTOR));
                if (route_hier(work, *coarse, HIER_FACTOR, HIER_RADIUS, s, g, tp,
                               max_exp, collect_visited, res))
                    routed = true;
                else
                    fb_exp = fallback_exp;   // probe+hier 紐⑤�???�뙣 = ?????留됲??????��???�궛 ??�븳.
            } else {
                routed = true;   // probe ???�� ???�?�� ?�좉�?= 寃쎈�???�쓬(?묎렐?�덇?) ??�???�뙣 寃곌??�?���?
            }
        }
        if (!routed)
            res = astar_weighted(work, s, g, tp, fb_exp, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 on_pipe ? &intra : nullptr, progress_every, AllowAll{}, t.goal_dir);
        bool ok = res.success && !res.path.empty();
        // 紐⑺�?吏꾩??��???�빟(goal_dir)??�줈 ??�뙣??�㈃ ?�댁???�쑝�?1????��????곌껐 ?곗꽑(?깃났??蹂댁??. ??�쭅??
        //   吏꾩??? �???��?寃쎈�????�?????�옟 ?�낅??. 吏꾩??��???�빟????�뜕(goal_dir<0) 諛곌??? 洹몃?�?
        if (!ok && t.goal_dir >= 0 && !aborted) {
            res = astar_weighted(work, s, g, tp, fb_exp, collect_visited,
                                 use_corridor ? &corridor : nullptr,
                                 on_pipe ? &intra : nullptr, progress_every, AllowAll{}, -1);
            ok = res.success && !res.path.empty();
        }
        std::vector<Cell> path = res.path;
        // ???��/??�????�굅(媛�??�?�� ?꾩슜, w_heur>1). ?�⑤뱺쨌??? A*(w=1)??誘몄?????寃곌??諛붿????�덈?.
        // mark ?꾩뿉 ?곸슜???꾩냽 諛곌?????�텞寃쎈줈瑜???�뵾(M1/M2 蹂댁??. ???��/踰덈�?諛붿???�뒪媛 留뚮�??깅땲???뺣━.
        if (ok && doc.params.w_heur > 1.0 && path.size() >= 4) {
            std::vector<Cell> up = unkink_path(work, path);
            if (up.size() < path.size()) {  // ??吏㏃븘�???�� ???�� �?���?湲몄?�쨌?�얠??媛먯??.
                path = std::move(up);
                res.path = path;
                res.length_mm = (path.size() - 1) * doc.params.cell_mm;
                res.turns = count_turns(path);
            }
        }
        // (C2 ?�붾�?理쒖?�諛?�꼍?? 諛곗??以묎�???꾨땲??'紐⑤�?諛곌? ??�슦???? ??��??? 理쒖�???�뒪�??곸슜 ???꾨옒 李몄??
        //  諛곗??以묎�??吏곸�?뷀�?��?諛붾????????�쓬 諛곌????�?????�먮?????�엳?????�얠?????��?????��? 135??41].)
        doc.results[static_cast<size_t>(oi)] = to_scene_result(res);
        if (ok) {
            // 源붾??寃쎈�?+諛섍�????�??�??�붽?(??�쓬 諛곌? ??�뵾). per-task 諛섍�?B1, OFF�?湲濡쒕�???�줈 ?�寃쎈�???�?.
            mark_pipe(work, path, radius_of(oi));
            if (use_corridor) add_corridor_cells(work, corridor, path, corridor_radius);
            placed[oi] = path;   // rip-up ???��(?꾨옒)????oi?믨꼍�?寃곗???std::map ??�쉶).
        }
        ++done;
        if (on_pipe) {  // phase=1 ?꾨즺 ??吏??+ (?깃났 ?? 寃쎈�???. 諛섑????�⑥?�硫???�쓬 諛곌??�??以묐??
            if (on_pipe(1, oidx, oi, ok, res.length_mm, res.turns, res.expanded_nodes, res.elapsed_ms,
                        done, n, 1.0, ok ? &path : nullptr) != 0)
                aborted = true;
        }
        // ?�⑥???붿껌 ???꾨즺??諛곌? 寃곌??doc.results)??蹂댁???��???? 諛곌??? 泥섎???? ??��??�낅�?
        if (aborted) break;
    }

    // ---- rip-up ???��(???�?) ----
    // main ??�뒪 ????? ??�뙣 諛곌??? �?'?μ븷臾?only ??�긽 寃쎈�???媛濡쒕�??placed 諛곌?(blocker)????�?    // ??같移?�빐 ??�냼??�떎. **?�댁?�??*(�?���????깃났 ??��?+1)�?*寃곗???*(blocker=placed ????�쫫李⑥??. pipe_radius
    // ????�씪 ?곸슜??�릺 ???�� 諛붿???�뒪??誘몄�??route_ripup ?? ??�씪 ???곌껐 ?곗꽑). build_work 媛 �???�룄 occ(?μ븷臾?
    // only) ????�?�� ?�?�꾩�??�쓣 ??�떆 源붿�?M1(?? ?�듭?� 0)??蹂댁?? 嫄곕?寃⑹??+ 誘몄???+ ??�뙣>0 ?????��(?�? ?�⑤�?    // 寃⑹???main ??�줈 ?�⑸?????�⑤�?湲곗????�옉 ?�덈?). env R3D_RIPUP=off �???????�떎.
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
            const int snap_r = 2 + radius_of(ti);   // per-task 諛섍�?B1).
            const RouteParams tpr = params_for(ti); // per-task ?��?clearance(B2).
            Cell ss = snap_to_free_cell(w, w.to_cell(tt.start_mm), snap_r);
            Cell gg = snap_to_free_cell(w, w.to_cell(tt.end_mm), snap_r);
            AStarResult r = astar_weighted(w, ss, gg, tpr, max_exp, false,
                                           nullptr, nullptr, 0, AllowAll{}, tt.goal_dir);
            if ((!r.success || r.path.empty()) && tt.goal_dir >= 0)   // 吏꾩??��?留됲?????�댁?????��?
                r = astar_weighted(w, ss, gg, tpr, max_exp, false);
            return r;
        };
        auto build_work = [&](const std::map<int, std::vector<Cell>>& paths) -> Occ {
            Occ w = occ.copy();   // occ = ?μ븷臾?only(?�덈? 湲곗?).
            for (const auto& kv : paths) mark_pipe(w, kv.second, radius_of(kv.first));   // per-task 諛섍�?
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
                AStarResult ideal = route_on(occ, f);   // ?μ븷臾?�쭔??�줈????�긽 寃쎈�?
                if (!(ideal.success && !ideal.path.empty())) continue;  // ?μ븷臾?�쭔??�줈???�덇?(?묎렐?�덇?).
                std::unordered_set<uint64_t> cs;
                cs.reserve(ideal.path.size() * 2);
                for (const Cell& c : ideal.path) cs.insert(pack(c));
                std::vector<int> blockers;   // placed ????�쫫李⑥??std::map).
                for (const auto& kv : placed) {
                    for (const Cell& c : kv.second)
                        if (cs.count(pack(c))) { blockers.push_back(kv.first); break; }
                }
                if (blockers.empty() || static_cast<int>(blockers.size()) > MAX_RIPUP) continue;
                std::map<int, std::vector<Cell>> trial = placed;
                for (int b : blockers) trial.erase(b);
                Occ wt = build_work(trial);
                AStarResult rf = route_on(wt, f);   // ??�???�듦�?�?�� ??�뙣 諛곌? ??같移?
                if (!(rf.success && !rf.path.empty())) continue;
                mark_pipe(wt, rf.path, radius_of(f));   // per-task 諛섍�?B1).
                trial[f] = rf.path;
                std::vector<AStarResult> rbs(blockers.size());
                bool all_ok = true;
                for (size_t bi = 0; bi < blockers.size(); ++bi) {
                    AStarResult rb = route_on(wt, blockers[bi]);   // ??? blocker ????고똿.
                    if (rb.success && !rb.path.empty()) {
                        mark_pipe(wt, rb.path, radius_of(blockers[bi]));   // per-task 諛섍�?B1).
                        trial[blockers[bi]] = rb.path;
                    } else {
                        all_ok = false;
                    }
                    rbs[bi] = std::move(rb);
                }
                if (!all_ok) continue;   // ?�댁?�???꾨같(blocker ??같移???�뙣) ??????�룄 ?�?��.
                placed = std::move(trial);
                doc.results[static_cast<size_t>(f)] = to_scene_result(rf);
                for (size_t bi = 0; bi < blockers.size(); ++bi)
                    doc.results[static_cast<size_t>(blockers[bi])] = to_scene_result(rbs[bi]);
                changed = true;
                // ???��????�뙣 諛곌????�쒕�??�줈 ???��??phase=1, oidx=-1=rip-up ??�떇) ????�씠??3D/??媛깆??
                if (on_pipe)
                    on_pipe(1, -1, f, true, rf.length_mm, rf.turns, rf.expanded_nodes, rf.elapsed_ms,
                            done, n, 1.0, &placed[f]);
            }
            if (!changed) break;   // ?????�� ?�덇? ???�낅�?
        }
    }

    // ---- C1 negotiated-congestion (CBS-lite, Phase C) ----
    // ??�㈃ rip-up(吏곸??blocker�???같移???�줈????? ??�뙣 諛곌??? blocker 媛 ??같移?�???�㈃ �?blocker ??
    // blocker 源뚯? bounded depth �?????곸쑝�??묐낫??�폒 ??�냼??�떎(conflict-based search 寃쎈???. ???�� ?�덈???
    //   resolve(target, state) 媛 true �?寃곌??out ?? **state ??紐⑤�?諛곌? + target ???�? ??�?*(???
    //   resolve ????�씪 蹂댁??by construction) ???깃났 ????��?+1(?�댁?�??. 寃곗????뺣젹 ??�룰?????�꽌). 源딆?�媛?
    //   �???? 1 媛먯????�??�낅�??�꾧�???(MAXBLK+1)^(depth+1)). 湲곕??eff_cbs_depth=0 ??誘몄????�⑤�??�덈?).
    if (eff_cbs_depth > 0 && !aborted && has_fail()) {
        const int CBS_MAXBLK = 4;   // ????�꺼 blocker ?곹븳(?�꾧�???�?李⑤??.
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
        // 寃쎈�??? ??AStarResult(doc.results ???μ????湲몄?�쨌?�얠??????? ?뺤옣??�뒗 吏꾨??蹂댁???0).
        auto result_from_path = [&](const std::vector<Cell>& p) -> AStarResult {
            AStarResult r;
            r.success = true; r.path = p;
            r.length_mm = (p.size() - 1) * doc.params.cell_mm;
            r.turns = count_turns(p);
            return r;
        };
        // ??? ?묒긽: state ?꾩뿉 target ????�썙?ｋ릺, 留됰??blocker ??depth 留뚰�???? ?묐낫??�궓??
        //   ?깃났 ??out = state ??諛곌? + target (?�? ??�슦??�맖). ??�뙣�?state ?�덈?(?�?묒슜 ??�쓬).
        std::function<bool(int, const std::map<int, std::vector<Cell>>&, int,
                           std::map<int, std::vector<Cell>>&)> resolve;
        resolve = [&](int target, const std::map<int, std::vector<Cell>>& state, int depth,
                      std::map<int, std::vector<Cell>>& out) -> bool {
            AStarResult ideal = route_on(occ, target);          // ?μ븷臾?�쭔??�줈????�긽 寃쎈�?
            if (!(ideal.success && !ideal.path.empty())) return false;
            std::unordered_set<uint64_t> cs;
            cs.reserve(ideal.path.size() * 2);
            for (const Cell& c : ideal.path) cs.insert(pack(c));
            std::vector<int> blockers;                          // state ????�쫫李⑥??std::map) ??寃곗???
            for (const auto& kv : state) {
                if (kv.first == target) continue;
                for (const Cell& c : kv.second)
                    if (cs.count(pack(c))) { blockers.push_back(kv.first); break; }
            }
            if (blockers.empty() || static_cast<int>(blockers.size()) > CBS_MAXBLK) return false;
            std::map<int, std::vector<Cell>> trial = state;
            for (int b : blockers) trial.erase(b);              // blocker ??�?
            AStarResult rf = route_on(build_work(trial), target);
            if (!(rf.success && !rf.path.empty())) return false;
            trial[target] = rf.path;                            // target 諛곗??
            for (int b : blockers) {                            // ??? blocker ??같移?or ??? ?묐낫).
                AStarResult rb = route_on(build_work(trial), b);
                if (rb.success && !rb.path.empty()) { trial[b] = rb.path; continue; }
                if (depth <= 0) return false;                   // ??�??묐낫 ???�댁?�???꾨같 ???�?��.
                std::map<int, std::vector<Cell>> sub;
                if (!resolve(b, trial, depth - 1, sub)) return false;
                trial = std::move(sub);                         // sub ??trial ??{b} (?�덈???.
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
                // �?���??�댁?�?? ??out ??諛곌? �?寃쎈줈媛? 諛붾????�줈 ??�릿 寃껊�?寃곌??媛깆??
                for (const auto& kv : out) {
                    auto it = placed.find(kv.first);
                    if (it == placed.end() || it->second != kv.second)
                        doc.results[static_cast<size_t>(kv.first)] = to_scene_result(result_from_path(kv.second));
                }
                placed = std::move(out);
                changed = true;
                if (on_pipe)   // ???��??諛곌? ??�씠??媛깆??oidx=-1=rip-up/CBS ??�떇).
                    on_pipe(1, -1, f, true, (placed[f].size() - 1) * doc.params.cell_mm,
                            count_turns(placed[f]), 0, 0.0, done, n, 1.0, &placed[f]);
            }
            if (!changed) break;
        }
    }

    // ---- C2 ?�붾�?理쒖?�諛?�꼍 理쒖�???�뒪(Phase C) ----
    // 紐⑤�?諛곌? ??�슦??�톜ip-up쨌CBS 媛 ??�궃 ?? �??깃났 諛곌???吏㏃? ???(??�낫 �?吏곸�?< mult?�愿?�?????�닔??�떎.
    // **??��???**: ??�슦??�뿉 ???�??(work)??嫄�?뱶由?? ??��? 諛곌?留덈??'?μ븷臾?+ ??�Ⅸ 諛곌?(�?諛섍�?'留뚯?�濡?留뚮�?    // 寃???�??(chk)??????吏곴????�닔 ????�슫??�듃??諛곌? ??�슦??�쓣 諛붽?�吏 ??�뒗??諛곗???곹샇?묒슜??�줈 ???�얠???
    // ??�뜕 ?�몄????�냼). ?�⑸룎寃??????�� + ?�얠????��쬆媛? + 湲몄????��쬆媛??????�� �?���??�댁?�??�룸Ъ?�ъ�??? ????�젏 ?�좎??.
    // 寃곗???placed ????�쫫李⑥??. 湲곕??eff_min_straight=0 ??誘몄????�⑤�??�덈?). ??��??O(?깃났???(opt-in ??�젙).
    if (eff_min_straight > 0.0 && cell_for_r > 0.0 && !aborted && !placed.empty()) {
        for (auto& kv : placed) {
            const int pi = kv.first;
            std::vector<Cell>& path = kv.second;
            const double d = doc.tasks[static_cast<size_t>(pi)].diameter_mm;
            const int min_run = (d > 0.0)
                ? static_cast<int>(std::ceil(eff_min_straight * d / cell_for_r)) : 0;
            if (min_run <= 1 || path.size() < 5) continue;
            // 寃???�?? = ?μ븷臾?+ ??�Ⅸ 諛곌?(�?per-task 諛섍�?. ?�?�� ?�?��?? ??�쇅(?�?�� ?�⑸�??�댁?�誘?.
            Occ chk = occ.copy();
            for (const auto& other : placed)
                if (other.first != pi) mark_pipe(chk, other.second, radius_of(other.first));
            std::vector<Cell> sp = enforce_min_straight(chk, path, min_run);
            if (sp.size() < path.size() ||
                (sp.size() == path.size() && count_turns(sp) < count_turns(path))) {
                // ??吏㏐�???�얠????�?��) 媛숈? 湲몄???�룄 ?�얠???�????�� �?���?
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

// 寃⑹????린濡?諛깆�???좏깮(sparse ??��?:
//   ?�? 寃⑹????M ??, ?�⑤�??? ??DenseOccupancy: 湲곗????�옉쨌諛붿씠??寃곌???꾩쟾 ?�덈?.
//   嫄곕? 寃⑹??>5M ??, 25mm/10mm) ??ImplicitOccupancy: 蹂듭?????�뒗 O(?μ븷臾? ????+ 64??��????+
//   ??�뵒留⑤�???�???�?????130MB/2GB 諛곗뿴쨌520MB 嫄곕?�蹂???�톓nt ??�쾭???��??紐⑤�???�뵾.
void route_multi_into_doc(SceneDoc& doc, const std::string& priority, bool collect_visited,
                          const ProgressCb& on_pipe = {}, const std::vector<Cell>* seed = nullptr,
                          int pipe_radius = 0, bool per_task_radius = false,
                          int cbs_depth = 0, double min_straight_mult = 0.0,
                          double pipe_gap_mm = 0.0,
                          const R3dRuntimeOptions* runtime = nullptr) {
#ifdef ROUTING3D_USE_OPENVDB
    route_multi_impl(doc, vdb_from_doc(doc), priority, collect_visited, on_pipe, seed,
                     pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm, runtime);
#else
    const long long cells =
        static_cast<long long>(doc.shape.i) * doc.shape.j * doc.shape.k;
    const long long large_threshold = runtime ? opt_or_default(runtime->large_grid_threshold, 5000000LL) : 5000000LL;
    if (cells > large_threshold) {
        route_multi_impl(doc, implicit_from_doc(doc), priority, collect_visited, on_pipe, seed,
                         pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm, runtime);
    } else {
        route_multi_impl(doc, occupancy_from_doc(doc), priority, collect_visited, on_pipe, seed,
                         pipe_radius, per_task_radius, cbs_depth, min_straight_mult, pipe_gap_mm, runtime);
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
        // Level 1(?�몄??? API ???몃뱾 ??�씠 ?몄텧???�?visited ??�쭛 湲곕??on.
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
    if (e->runtime.ripup_enabled < -1) e->runtime.ripup_enabled = -1;
    if (e->runtime.ripup_enabled > 1) e->runtime.ripup_enabled = 1;
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

// ???��(pass-through) 媛앹�??�붽? ???�??�?媛??�솕?? 寃쎈�?�?�� ?�⑸�??????꾨떂(doc.passthrough).
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

// ?묒뾽??紐⑺�?吏꾩??��???�빟(goal_dir) ??�젙 ??A* 媛 end_mm ??'axis 諛⑺�?NEIGHBORS_6 ?몃뜳??0..5 =
//   +x,-x,+y,-y,+z,-z)??�줈 吏꾩??????��' ?꾨떖 ?몄젙. ?뺥듃 ?�낅????��??�щ뱶???�뺤??二쇰????�쭅??吏꾩???묒냽?�
//   ?�곕??붽린 ?�얠????�굅). ??�빟??�줈 留됲?�硫??붿쭊???�댁???1????��??곌껐 ?곗꽑). axis 媛 [0,5] 諛뽰?�硫?-1(?�댁???.
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
                             e->pipe_gap_mm, &e->runtime);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// per-task ?��?諛섍�?B1) ??�꽦????ON ??�??route_multi 媛 �?諛곌? diameter_mm �?留덊�?諛섍�???�?�� ?곗텧
//   (?몄텧??湲濡쒕�?pipe_radius �?��????�굅쨌媛???諛곌? ?�쇳?????�냼). OFF(湲곕??=湲濡쒕�?pipe_radius(湲곗????�옉).
extern "C" R3dStatus r3d_set_per_task_radius(R3dEngine* e, int32_t enabled) {
    if (!e) return R3D_ERR_ARG;
    e->per_task_radius = enabled != 0;
    return R3D_OK;
}

// C1 negotiated-congestion(CBS-lite) 源딆????�젙 ??0=OFF(??�㈃ rip-up留뙿룰린�???�옉쨌怨⑤�??�덈?). >0 ??�????�㈃
//   rip-up ????? ??�뙣 諛곌????곗뇙(???) rip-up ??�줈 ??�냼(?�댁?�??�룰�?뺤쟻). [0,3] ??�??? env R3D_CBS ??媛??
extern "C" R3dStatus r3d_set_cbs_depth(R3dEngine* e, int32_t depth) {
    if (!e) return R3D_ERR_ARG;
    e->cbs_depth = depth < 0 ? 0 : (depth > 3 ? 3 : depth);
    return R3D_OK;
}

// C2 ?�붾�?理쒖?�諛?�꼍 諛곗????�젙 ????�낫 �?吏곸�??? ??(mult ???��? 蹂댁????�옉??. 寃쎈�???) ??��?�?�� ?�⑸룎寃???
//   ??�뿉 吏㏃? ???????�닔. 0=OFF(湲곗????�옉쨌怨⑤�??�덈?). 沅뚯??2.0. ???���?0. env R3D_MIN_STRAIGHT ??媛??
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

// 諛곌?-諛곌? ??�꺽(mm) ??�젙 ????諛곌? ??�꽣??嫄곕????r1 + r2 + gap 蹂댁????�㈃ ????理쒖??gap mm). 0=OFF(湲곗??//   ??�옉�??�㈃ 留욌???�룰????�덈?). 洹쒓�?60mm. route_multi 硫붿???�⑦봽媛? 源붾??諛곌?????諛섍�??�줈 留됰??? env R3D_PIPE_GAP.
extern "C" R3dStatus r3d_set_pipe_gap(R3dEngine* e, double gap_mm) {
    if (!e) return R3D_ERR_ARG;
    e->pipe_gap_mm = gap_mm > 0.0 ? gap_mm : 0.0;
    return R3D_OK;
}

extern "C" R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                              void* user) {
    if (!e) return R3D_ERR_ARG;
    try {
        ProgressCb on_pipe;  // cb 媛 ?�?���???��????�쒕�???�뒗 route_multi ?? ??�씪).
        if (cb) {
            on_pipe = [cb, user](int phase, int oi, int ti, bool ok, double len, int turns,
                                 long long exp, double ms, int done, int total, double prog,
                                 const std::vector<Cell>* path) -> int {
                // 寃쎈�???(i,j,k) ???꾩떆 int 諛곗뿴濡???�꽌 ?�쒕�???꾨떖(????곕뒗 ?몄텧 ??�븞�??좏슚).
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
                             e->pipe_gap_mm, &e->runtime);
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// ??�뒿?????�� ??(ijk ??�쨷??諛곗�? 湲몄??n)???붿쭊????�젙??�떎(L2b). w_corridor>0 ????route_multi 媛
// ??????�쓣 ???�� ??�뱶�???�븘 諛곌???�??�곸?�濡??좊룄??�떎. n<=0 ?�?�� ijk==null ??�?????��????��???
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

// 諛곌? ?�?? ??�갹 諛섍�???) ??�젙(???�?, 諛곌?-諛곌? ?�⑸�???�뵾). route_multi(_progress) 媛 源붾??諛곌???
// mark_pipe(radius)�?留됱�???�쓬 諛곌? 以묒??좎쓣 ?꾩슫?? ???���?0 ??�줈 ??�??? 0=湲곗????�옉(寃쎈�???�?.
extern "C" R3dStatus r3d_set_pipe_radius(R3dEngine* e, int32_t radius_cells) {
    if (!e) return R3D_ERR_ARG;
    e->pipe_radius = radius_cells > 0 ? radius_cells : 0;
    return R3D_OK;
}

// rip-up & reroute(Step 3.8): ??�뜑 route_ripup ???몄텧??�릺, 寃곌?�瑜?'?�?�� ?묒뾽 ?몃뜳??�?
// ??�룎??doc.results ??????get_result 留ㅽ�?蹂댁?? doc.tasks ?�덈?). ?곗꽑??�쐞 ??�뿴??
// order_indices �?????route_ripup ??�? order_tasks ?? ??�씪 ??�젙 ?뺣젹 ???꾩튂 ??�튂).
extern "C" R3dStatus r3d_route_ripup(R3dEngine* e, const char* priority, int32_t max_rounds,
                                     int32_t max_ripup) {
    if (!e) return R3D_ERR_ARG;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        SceneDoc& doc = e->doc;
        const std::string prio = priority ? priority : "longest";
        // ?�?? 諛깆�???�닿?(??�뵆?? ??寃곌???�붿?????�꽕?????���? ????寃⑹??>5M ??)??ImplicitOccupancy
        //   (蹂듭?????�쓬쨌O(?μ븷臾?)�??꾪솚??Dense ?꾨같????컻쨌int ??�쾭???��??諛⑹???�떎(A3, route_multi 寃뚯?????�씪).
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

// ?????λ???corridor ??�슦?? ?μ븷臾?�쓣 fine/coarse Sparse ?�??�?留뚮뱾�??묒뾽�?route_corridor.
// Sparse + astar_hashed ??occ.size() 諛곗�????? ??�쑝誘�??�덈???寃⑹?????�옉(硫붾?�由??�?? ??).
extern "C" R3dStatus r3d_route_corridor(R3dEngine* e, int32_t factor, int32_t radius) {
    if (!e) return R3D_ERR_ARG;
    if (factor < 1 || radius < 0) return R3D_ERR_ARG;
    try {
        apply_min_straight_cells(e);   // 코너 최소직선(절대 mm)→셀 제약 반영.
        SceneDoc& doc = e->doc;

        // fine/coarse ?????�??�??μ븷臾?�쭔). coarse ?? = fine ?? ??factor.
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
                // ??�솕 諛뺤???�댁??
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

// ??�감 ?�꾩�?corridor: r3d_route_corridor ?? 媛숈? Sparse + astar_hashed ??��? priority ??�꽌�?
// ??諛곌?????�슦??�븯???깃났 寃쎈줈瑜?fine ?�????mark_pipe �??�붽?????�쓬 諛곌?????�븯�???�떎(?�⑸�?0).
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
                // ??�솕 諛뺤???�댁??
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
                // ??�쓬 諛곌?????�븯?꾨줉 fine ?�????寃쎈�?+諛섍�????�붽?. coarse ??媛??��??誘명�??
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
        // 諛깆�???좏깮(route_multi ?? ??�씪 ?뺤콉): ?�? 寃⑹????M, ?�⑤�???DenseOccupancy �?湲곗????�옉�?        //   諛붿???寃곌???꾩쟾 ?�덈?. 嫄곕? 寃⑹??>5M, 25mm/10mm)??蹂듭?????�뒗 ImplicitOccupancy(O(?μ븷臾?) +
        //   ?�?�� ?곹븳(12M, 硫붾?�由???쬆쨌?곗뼱??�씠 諛⑹?)??�줈 ?꾪솚 ??C# ?�붾�?蹂듭???꾩쿂?�ъ쓽 ??�씪 ??�━ A* 媛
        //   1.3???? 寃⑹??�?��??�??몄텧 130M 蹂듭?????�씠 ??��?�寃???�옉(洹몃�???��/湲곗???�퀎異붿쥌 ?꾩쿂????�꽦??.
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
        return R3D_ERR_RUNTIME;  // ?꾩쭅 ??�슦??????
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

// 諛⑸Ц(?뺤옣) ?? 蹂듭�???媛??�솕 '諛⑸Ц�? ?? copy_path ?? ??�씪 ?뺤떇.
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

// ?�??�??�붾�????) ?몃뜳??蹂듭�???媛??�솕 '?�??�? ?? ?꾩옱 doc ??obstacles �?利됱�?voxelize.
// buf=NULL, buf_cells=0 ??�?????? ??�쭔 諛섑?????�利?議고??. ?�??蹂듭�???泥섏??buf_cells �?
extern "C" int32_t r3d_copy_blocked(const R3dEngine* e, int32_t* buf, int32_t buf_cells) {
    if (!e) return 0;
    try {
        const Cell& shape = e->doc.shape;
        bool size_only = (buf == nullptr || buf_cells <= 0);
        // ?�?? 諛깆�???�닿? ??�틪 ??????寃⑹??>5M ??)??ImplicitOccupancy(?꾨같??誘명�??�?is_blocked 吏덉???
        //   Dense ?꾨같????25mm ~1.3??????B) 硫붾?�由???�??諛⑹?(A3). ??�삎?? Dense(湲곗????�옉 ??�씪).
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
            return written;  // size_only=true �??꾩껜 移댁??? false �???�젣 蹂듭�???? ??
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
extern "C" int32_t r3d_copy_blocked_sampled(const R3dEngine* e, int32_t max_cells, int32_t* buf) {
    if (!e || max_cells <= 0 || !buf) return 0;
    try {
        std::vector<Cell> all_cells;
        const Cell& shape = e->doc.shape;
        const long long total_cells = (long long)shape.i * shape.j * shape.k;
#ifdef ROUTING3D_USE_OPENVDB
        all_cells = vdb_from_doc(e->doc).blocked_cells();
#else
        if (total_cells > 5000000LL) {
            all_cells = implicit_from_doc(e->doc).blocked_cells();
        } else {
            auto occ = occupancy_from_doc(e->doc);
            for (int i = 0; i < shape.i; ++i)
                for (int j = 0; j < shape.j; ++j)
                    for (int k = 0; k < shape.k; ++k) {
                        Cell c{i, j, k};
                        if (occ.is_blocked(c)) all_cells.push_back(c);
                    }
        }
#endif
        auto n = static_cast<int32_t>(all_cells.size());
        if (n <= max_cells) {
            for (int32_t idx = 0; idx < n; ++idx) {
                buf[3 * idx + 0] = all_cells[static_cast<size_t>(idx)].i;
                buf[3 * idx + 1] = all_cells[static_cast<size_t>(idx)].j;
                buf[3 * idx + 2] = all_cells[static_cast<size_t>(idx)].k;
            }
            return n;
        }
        for (int32_t s = 0; s < max_cells; ++s) {
            size_t idx = static_cast<size_t>((long long)s * n / max_cells);
            buf[3 * s + 0] = all_cells[idx].i;
            buf[3 * s + 1] = all_cells[idx].j;
            buf[3 * s + 2] = all_cells[idx].k;
        }
        return max_cells;
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
        // ???�利?議고??紐⑤�? buf 誘몄???
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
        return written;  // size_only=true �??꾩껜 移댁??? false �???�젣 蹂듭�???? ??
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
        out->fail_reason   = (int32_t)res.fail;
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

// ============================================================================ 옥트리 리프 열거 (3D 가시화)
extern "C" R3dStatus r3d_enum_octree_leaves(R3dEngine* e,
                                             R3dOctreeLeaf* buf, int32_t maxCount,
                                             int32_t* out_count) {
    if (!e || !buf || maxCount <= 0 || !out_count) return R3D_ERR_ARG;
    if (e->doc.shape.i <= 0) return R3D_ERR_ARG;   // scene 미로드
    *out_count = 0;
    try {
        OctreeOccupancy occ;
        occ.build(e->doc);

        int count = 0;
        const double cell = e->doc.cell_mm;
        const Vec3&  ori  = e->doc.origin;

        for (const auto& node : occ.nodes) {
            if (!node.is_leaf()) continue;
            if (count >= maxCount) break;
            buf[count].x0_mm  = (float)(ori.x + node.x0 * cell);
            buf[count].y0_mm  = (float)(ori.y + node.y0 * cell);
            buf[count].z0_mm  = (float)(ori.z + node.z0 * cell);
            buf[count].size_mm = (float)(node.side * cell);
            buf[count].state  = (int32_t)node.state;
            count++;
        }
        *out_count = count;
        return R3D_OK;
    } catch (...) {
        return R3D_ERR_RUNTIME;
    }
}

