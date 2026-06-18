// 가변셀 옥트리 점유맵 + 점프 기반 A* — Routing3D C++ 엔진
// =============================================================================
// [이 파일이 하는 일]
//   장애물 AABB를 옥트리로 색인해 자유공간에서 대형 셀 단위로 점프하는
//   적응형 경로탐색을 제공한다.
//
//   핵심 아이디어:
//     - 장애물 근처: 최소 셀(fine-cell) 해상도로 정밀하게
//     - 자유공간  : 옥트리 리프 한 변(최대 수백 셀)을 한 번에 점프
//     - 탐색 상태 수 = 기존 A* 의 1/side² ~ 1/side³ 감소
//
//   OctreeOccupancy  — SceneDoc 장애물로 옥트리 빌드, blocked/jump 질의
//   astar_octree     — OctreeOccupancy 위의 점프 A*(결과는 fine-cell 경로)
//
// [인터페이스]
//   OctreeOccupancy occ;  occ.build(doc);
//   auto res = astar_octree(occ, start, goal, params, max_exp);
//
// [골든 불변식]
//   occ.build 는 ImplicitOccupancy 와 동일 격자·셀 크기를 사용.
//   astar_octree 경로는 모든 fine-cell 이 occ.is_blocked==false.
// =============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_set>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/scene_io.hpp"

namespace routing3d {

// ─────────────────────────────────────────────────────────────────
// 1. 옥트리 노드
// ─────────────────────────────────────────────────────────────────
// children 인덱싱: ci = dz*4 + dy*2 + dx  (dx,dy,dz ∈ {0,1})
//   dx=0 → x ∈ [x0, x0+half)   dx=1 → x ∈ [x0+half, x0+side)
struct OctNode {
    int x0, y0, z0;   // 원점(fine-cell 단위)
    int side;          // 한 변 길이(fine-cell, 항상 2의 거듭제곱)
    int8_t  state;     // -1=MIXED, 0=FREE, 1=BLOCKED
    int children[8];   // 자식 id. 리프이면 모두 -1.
    int parent;        // 루트이면 -1.

    bool is_leaf()    const noexcept { return children[0] < 0; }
    bool is_free()    const noexcept { return state == 0; }
    bool is_blocked() const noexcept { return state == 1; }
};

// ─────────────────────────────────────────────────────────────────
// 2. 옥트리 점유맵
// ─────────────────────────────────────────────────────────────────
class OctreeOccupancy {
public:
    // ---- 격자 메타 ----
    int    nx = 0, ny = 0, nz = 0;
    double cell_mm_ = 25.0;
    Vec3   origin_;

    // ---- 옥트리 ----
    std::vector<OctNode> nodes;
    int root_id   = -1;
    int root_side = 0;

    // ---- 배관 마킹(깔린 배관 fine-cell 집합) ----
    // ImplicitOccupancy.marked_ 와 동일 역할(옥트리 구조 불변, 별도 집합으로 관리).
    std::unordered_set<uint64_t> marked_;

    // SceneDoc 으로 옥트리 빌드
    void build(const SceneDoc& doc);

    // ---- 점유 질의 ----
    bool in_bounds(const Cell& c) const noexcept {
        return c.i>=0 && c.j>=0 && c.k>=0 && c.i<nx && c.j<ny && c.k<nz;
    }
    bool is_blocked(const Cell& c) const noexcept {
        if (!in_bounds(c)) return true;
        if (marked_.count(pack(c))) return true;
        return find_leaf_impl(root_id, c.i, c.j, c.k) < 0;
    }

    // fine-cell 을 포함하는 자유 리프 id 반환 (-1 = blocked/out)
    int find_leaf(const Cell& c) const noexcept {
        if (!in_bounds(c) || marked_.count(pack(c))) return -1;
        return find_leaf_impl(root_id, c.i, c.j, c.k);
    }

    // ---- 점프 질의 ----
    // 방향 axis(NEIGHBORS_6 순서 0..5) 로 c 에서 최대 몇 셀 전진 가능한지.
    // 멈추는 조건: ① 옥트리 리프 경계 ② marked_ 셀 ③ 격자 밖/blocked ④ max_dist 도달
    // 반환값 ≥ 0 (0 = 한 셀도 전진 불가)
    int max_jump(const Cell& c, int axis, int max_dist = 65536) const noexcept;

    // ---- 배관 마킹 ----
    void block_cell(const Cell& c) {
        if (in_bounds(c)) marked_.insert(pack(c));
    }
    int add_box(const AABB& box) {
        // 복셀화해서 marked_ 에 추가
        CellRange r = grid_box_range(box, origin_, cell_mm_, Cell{nx,ny,nz});
        for (int k=r.lo.k;k<r.hi.k;++k)
            for (int j=r.lo.j;j<r.hi.j;++j)
                for (int i=r.lo.i;i<r.hi.i;++i)
                    marked_.insert(pack({i,j,k}));
        return 0;
    }

    // ImplicitOccupancy 호환 인터페이스 (CostModel, lin/unlin)
    Vec3 to_world(const Cell& c) const { return grid_cell_to_world(c, origin_, cell_mm_); }
    Cell to_cell(const Vec3& w)  const { return grid_world_to_cell(w, origin_, cell_mm_); }
    long long size() const { return (long long)nx*ny*nz; }
    double cell_mm() const { return cell_mm_; }
    Cell shape()     const { return {nx,ny,nz}; }

    long long lin(const Cell& c) const noexcept {
        return (long long)c.i + (long long)nx*((long long)c.j + (long long)ny*c.k);
    }
    Cell unlin(long long idx) const noexcept {
        int i = (int)(idx % nx);
        int j = (int)((idx/nx) % ny);
        int k = (int)(idx/(nx*ny));
        return {i,j,k};
    }

    // 온디맨드 클리어런스 (HasClearanceQuery 개념 충족)
    int clearance_cells(const Cell& c, int max_r) const noexcept {
        if (max_r <= 0) return max_r;
        // 6방향 BFS 간이 버전: 반경 내 blocked 거리 반환
        for (int r = 0; r < max_r; ++r) {
            for (const auto& d : NEIGHBORS_6) {
                Cell nb{c.i+d.i*(r+1), c.j+d.j*(r+1), c.k+d.k*(r+1)};
                if (!in_bounds(nb) || is_blocked(nb)) return r;
            }
        }
        return max_r;
    }

    OctreeOccupancy copy() const { return *this; }

    // ─── 자유 리프의 인접 리프 목록 (그래프 탐색용) ───
    // axis 방향으로 leaf_id 에 면 인접한 자유 리프 id 목록을 out 에 추가
    void face_neighbors(int leaf_id, int axis, std::vector<int>& out) const;

private:
    // 장애물 AABB (fine-cell 반열린 구간 [lo, hi))
    struct BoxI { int x0,y0,z0,x1,y1,z1; };  // [x0,x1) × [y0,y1) × [z0,z1)
    std::vector<BoxI> obs_;

    int alloc_node();
    void build_node(int id, const std::vector<int>& obs_ids);
    int  find_leaf_impl(int id, int x, int y, int z) const noexcept;

    // 재귀 이웃 탐색 (표준 옥트리 알고리즘)
    // axis 방향으로 node id 의 '같은 크기 이상' 이웃 반환 (-1=없음)
    int equal_or_larger_neighbor(int id, int axis) const noexcept;
    // node id 의 face_axis 쪽 면에 있는 모든 자유 리프를 out 에 추가
    void collect_face_leaves(int id, int face_axis, std::vector<int>& out) const;

    static uint64_t pack(const Cell& c) noexcept {
        return ((uint64_t)(uint32_t)c.i << 42) |
               ((uint64_t)(uint32_t)c.j << 21) |
               (uint64_t)(uint32_t)c.k;
    }

    bool obs_intersects(const BoxI& b, const OctNode& n) const noexcept {
        return b.x0<n.x0+n.side && b.x1>n.x0 &&
               b.y0<n.y0+n.side && b.y1>n.y0 &&
               b.z0<n.z0+n.side && b.z1>n.z0;
    }
    bool obs_contains(const BoxI& b, const OctNode& n) const noexcept {
        return b.x0<=n.x0 && b.x1>=n.x0+n.side &&
               b.y0<=n.y0 && b.y1>=n.y0+n.side &&
               b.z0<=n.z0 && b.z1>=n.z0+n.side;
    }
};

// ─────────────────────────────────────────────────────────────────
// 3. 점프 A* 결과 / 함수 선언
// ─────────────────────────────────────────────────────────────────
// AStarResult 를 재사용 (astar.hpp 참조).
// astar_octree 는 내부적으로 점프를 사용하지만 반환하는 path 는
// 모든 fine-cell 을 포함한다 (segment 단위가 아닌 셀 단위 완전 열거).
AStarResult astar_octree(
    OctreeOccupancy& occ,
    Cell start, Cell goal,
    RouteParams params,
    long long max_expansions = 0,   // 0=무제한
    int goal_dir = -1,              // -1=무제약, 0..5=진입축 고정
    bool collect_visited = false
);

// ─────────────────────────────────────────────────────────────────
// 4. 구현
// ─────────────────────────────────────────────────────────────────

// ── alloc_node ───────────────────────────────────────────────────
inline int OctreeOccupancy::alloc_node() {
    int id = (int)nodes.size();
    nodes.push_back({});
    auto& n = nodes.back();
    n.state = -1; n.parent = -1;
    for (auto& c : n.children) c = -1;
    return id;
}

// ── build ─────────────────────────────────────────────────────────
inline void OctreeOccupancy::build(const SceneDoc& doc) {
    cell_mm_ = doc.cell_mm;
    origin_  = doc.origin;
    nx = doc.shape.i; ny = doc.shape.j; nz = doc.shape.k;

    // 루트 한 변 = 2^ceil(log2(max(nx,ny,nz)))
    int maxn = std::max({nx, ny, nz});
    root_side = 1;
    while (root_side < maxn) root_side <<= 1;

    // 장애물 → fine-cell BoxI
    obs_.clear();
    obs_.reserve(doc.obstacles.size());
    for (const auto& ob : doc.obstacles) {
        auto fl = [](double v, double o, double c) {
            return (int)std::floor((v-o)/c);
        };
        auto cl = [](double v, double o, double c) {
            return (int)std::ceil((v-o)/c);
        };
        int x0=std::max(fl(ob.min_xyz.x,origin_.x,cell_mm_),0);
        int y0=std::max(fl(ob.min_xyz.y,origin_.y,cell_mm_),0);
        int z0=std::max(fl(ob.min_xyz.z,origin_.z,cell_mm_),0);
        int x1=std::min(cl(ob.max_xyz.x,origin_.x,cell_mm_),nx);
        int y1=std::min(cl(ob.max_xyz.y,origin_.y,cell_mm_),ny);
        int z1=std::min(cl(ob.max_xyz.z,origin_.z,cell_mm_),nz);
        if (x0<x1 && y0<y1 && z0<z1)
            obs_.push_back({x0,y0,z0,x1,y1,z1});
    }

    // 옥트리 빌드
    nodes.clear();
    nodes.reserve(obs_.size() * 128 + 64);
    root_id = alloc_node();
    auto& root = nodes[root_id];
    root.x0=0; root.y0=0; root.z0=0;
    root.side=root_side; root.parent=-1;

    std::vector<int> all_obs((int)obs_.size());
    std::iota(all_obs.begin(), all_obs.end(), 0);
    build_node(root_id, all_obs);
}

// ── build_node ────────────────────────────────────────────────────
inline void OctreeOccupancy::build_node(int id, const std::vector<int>& obs_ids) {
    // 부모 참조 후 grow() 로 무효화될 수 있으므로 로컬로 복사
    int x0=nodes[id].x0, y0=nodes[id].y0, z0=nodes[id].z0, side=nodes[id].side;

    // 격자 완전 밖 → BLOCKED
    if (x0>=nx || y0>=ny || z0>=nz) { nodes[id].state=1; return; }

    // 교차 장애물 필터
    std::vector<int> hit;
    for (int oi : obs_ids)
        if (obs_intersects(obs_[oi], nodes[id])) hit.push_back(oi);

    if (hit.empty()) { nodes[id].state=0; return; }  // FREE

    // 하나라도 완전 포함 → BLOCKED
    for (int oi : hit)
        if (obs_contains(obs_[oi], nodes[id])) { nodes[id].state=1; return; }

    // 최소 셀 → BLOCKED (더 이상 분할 불가)
    if (side == 1) { nodes[id].state=1; return; }

    // MIXED → 8자식으로 분할
    nodes[id].state = -1;
    int half = side >> 1;
    for (int dz=0;dz<2;++dz) for (int dy=0;dy<2;++dy) for (int dx=0;dx<2;++dx) {
        int ci = dz*4+dy*2+dx;
        int cid = alloc_node();           // grow 가능 → 이후 nodes[id] 재참조 필요
        nodes[id].children[ci] = cid;    // nodes 가 grow 됐어도 id 는 유효(index)
        nodes[cid].x0     = nodes[id].x0 + dx*half;
        nodes[cid].y0     = nodes[id].y0 + dy*half;
        nodes[cid].z0     = nodes[id].z0 + dz*half;
        nodes[cid].side   = half;
        nodes[cid].parent = id;
        build_node(cid, hit);
    }
}

// ── find_leaf_impl ────────────────────────────────────────────────
inline int OctreeOccupancy::find_leaf_impl(int id, int x, int y, int z) const noexcept {
    const auto& n = nodes[id];
    if (n.is_leaf()) return n.is_free() ? id : -1;
    int half = n.side >> 1;
    int dx = (x >= n.x0+half) ? 1 : 0;
    int dy = (y >= n.y0+half) ? 1 : 0;
    int dz = (z >= n.z0+half) ? 1 : 0;
    int cid = n.children[dz*4+dy*2+dx];
    if (cid < 0) return -1;
    return find_leaf_impl(cid, x, y, z);
}

// ── max_jump ──────────────────────────────────────────────────────
// 방향 테이블: axis 0=+i,1=-i,2=+j,3=-j,4=+k,5=-k
static constexpr int JUMP_DI[] = {1,-1,0,0,0,0};
static constexpr int JUMP_DJ[] = {0,0,1,-1,0,0};
static constexpr int JUMP_DK[] = {0,0,0,0,1,-1};

inline int OctreeOccupancy::max_jump(const Cell& c, int axis, int max_dist) const noexcept {
    int leaf = find_leaf(c);
    if (leaf < 0) return 0;
    const OctNode& n = nodes[leaf];

    // 리프 내 최대 전진 가능 거리
    int extent;
    switch (axis) {
        case 0: extent = (n.x0+n.side-1) - c.i; break;  // +i
        case 1: extent =  c.i - n.x0;             break;  // -i
        case 2: extent = (n.y0+n.side-1) - c.j; break;  // +j
        case 3: extent =  c.j - n.y0;             break;  // -j
        case 4: extent = (n.z0+n.side-1) - c.k; break;  // +k
        case 5: extent =  c.k - n.z0;             break;  // -k
        default: return 0;
    }
    // extent=0: 리프 경계에 정확히 있음 → 인접 리프 1셀 전진 시도 가능
    int limit = (extent > 0) ? std::min(extent, max_dist) : std::min(1, max_dist);

    int di = JUMP_DI[axis], dj = JUMP_DJ[axis], dk = JUMP_DK[axis];
    for (int s = 1; s <= limit; ++s) {
        Cell nb{c.i+di*s, c.j+dj*s, c.k+dk*s};
        if (!in_bounds(nb) || is_blocked(nb) || marked_.count(pack(nb))) return s-1;
    }
    return limit;
}

// ── equal_or_larger_neighbor ──────────────────────────────────────
// 표준 옥트리 이웃 알고리즘: axis 방향의 '같은 크기 이상' 이웃 반환.
// 결과가 MIXED 이면 smaller neighbors 는 collect_face_leaves 로 수집.
inline int OctreeOccupancy::equal_or_larger_neighbor(int id, int axis) const noexcept {
    if (id == root_id) return -1;
    const OctNode& n  = nodes[id];
    int pid           = n.parent;
    const OctNode& p  = nodes[pid];

    // 부모 내 자식 인덱스 계산 (위치로 역산)
    int half = p.side >> 1;
    int dx = (n.x0 >= p.x0+half) ? 1 : 0;
    int dy = (n.y0 >= p.y0+half) ? 1 : 0;
    int dz = (n.z0 >= p.z0+half) ? 1 : 0;

    // axis 의 차원(dim)과 '먼 쪽 bit'
    // axis: 0=+i,1=-i,2=+j,3=-j,4=+k,5=-k
    static constexpr int DIM[]      = {0,0,1,1,2,2};
    static constexpr int FAR_BIT[]  = {1,0,1,0,1,0}; // +방향이면 bit=1 이 먼쪽
    int dim = DIM[axis];
    int bits[3] = {dx,dy,dz};
    bool on_far = (bits[dim] == FAR_BIT[axis]);

    if (on_far) {
        // 부모의 이웃으로 올라가 내려옴
        int pnb = equal_or_larger_neighbor(pid, axis);
        if (pnb < 0) return -1;
        const OctNode& pn = nodes[pnb];
        if (pn.is_leaf()) return pnb;  // 더 큰 이웃
        // 내려올 때 같은 dim 은 반전, 나머지는 동일
        int nbits[3] = {dx,dy,dz};
        nbits[dim] = 1 - FAR_BIT[axis];  // 근접면 쪽 자식
        return pn.children[nbits[2]*4 + nbits[1]*2 + nbits[0]];
    } else {
        // 부모 내 형제 (동일 위치, dim 만 반전)
        int nbits[3] = {dx,dy,dz};
        nbits[dim] ^= 1;
        return p.children[nbits[2]*4 + nbits[1]*2 + nbits[0]];
    }
}

// ── collect_face_leaves ───────────────────────────────────────────
// node id 의 face_axis 쪽 면에 있는 자유 리프를 수집.
// face_axis 는 '이 노드로 진입하는 방향' (진입 = 반대편 face 를 통해).
// face_axis=0(+i)이면 이 노드의 x0 면(= -i 면)에 있는 자식들을 재귀 수집.
inline void OctreeOccupancy::collect_face_leaves(int id, int face_axis,
                                                  std::vector<int>& out) const {
    const OctNode& n = nodes[id];
    if (n.is_blocked()) return;
    if (n.is_free()) { out.push_back(id); return; }

    // MIXED: face_axis 방향으로 '근접면'에 있는 자식만 재귀
    // face_axis: 0=+i면→dx=0 자식, 1=-i면→dx=1 자식
    //            2=+j면→dy=0,  3=-j면→dy=1
    //            4=+k면→dz=0,  5=-k면→dz=1
    static constexpr int NEAR_DIM[]  = {0,0,1,1,2,2};
    static constexpr int NEAR_BIT[]  = {0,1,0,1,0,1};  // 근접면 쪽 bit
    int dim  = NEAR_DIM[face_axis];
    int nbit = NEAR_BIT[face_axis];
    for (int dz=0;dz<2;++dz) for (int dy=0;dy<2;++dy) for (int dx=0;dx<2;++dx) {
        int bits[3]={dx,dy,dz};
        if (bits[dim] != nbit) continue;
        int cid = n.children[dz*4+dy*2+dx];
        if (cid >= 0) collect_face_leaves(cid, face_axis, out);
    }
}

// ── face_neighbors ────────────────────────────────────────────────
inline void OctreeOccupancy::face_neighbors(int leaf_id, int axis,
                                             std::vector<int>& out) const {
    int nb = equal_or_larger_neighbor(leaf_id, axis);
    if (nb < 0) return;
    const OctNode& nbn = nodes[nb];
    if (nbn.is_free())    { out.push_back(nb); return; }
    if (nbn.is_blocked()) return;
    // MIXED: axis 의 반대 face 에 있는 자유 리프 수집
    // axis 0(+i)이면 이웃의 -i 면(face_axis=0)에 있는 자식
    collect_face_leaves(nb, axis, out);
}

// ─────────────────────────────────────────────────────────────────
// 5. astar_octree — 점프 기반 A*
// ─────────────────────────────────────────────────────────────────
// 상태: (lin, dir, run_capped) — astar_weighted 와 동일 인코딩.
// 점프: 각 방향에서 max_jump 셀을 한 번에 전진 (비용=점프거리×cell_mm).
// 결과 path: 모든 fine-cell 을 포함한 완전한 경로(셀별 열거).
inline AStarResult astar_octree(OctreeOccupancy& occ,
                                  Cell start, Cell goal,
                                  RouteParams params,
                                  long long max_expansions,
                                  int goal_dir,
                                  bool collect_visited) {
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();
    auto ms  = [&]{ return std::chrono::duration<double,std::milli>(Clock::now()-t0).count(); };

    AStarResult R;

    // 시작/목표 유효성 검사
    if (occ.is_blocked(start)) { R.fail=RouteFail::StartBlocked; R.elapsed_ms=ms(); return R; }
    if (occ.is_blocked(goal))  { R.fail=RouteFail::GoalBlocked;  R.elapsed_ms=ms(); return R; }
    if (start == goal) {
        R.success=true; R.path={start}; R.expanded_nodes=1; R.elapsed_ms=ms(); return R;
    }

    const double cmm   = params.cell_mm;
    const int min_run  = (params.min_straight_cells > 1) ? params.min_straight_cells : 0;
    const int RS       = (min_run > 1) ? (min_run+1) : 1;

    // 상태 인코딩 (astar_weighted 와 동일)
    auto state_of = [&](long long lin, int dir, int run) -> long long {
        int r = (min_run>1) ? std::min(run, min_run) : 0;
        return (lin*7 + (dir+1)) * RS + r;
    };

    // 휴리스틱
    auto h_raw = [&](const Cell& c) {
        return (double)manhattan(c,goal) * cmm;
    };
    auto h_w = [&](const Cell& c, double h_start) -> double {
        double hw = h_raw(c);
        if (params.w_heur > 1.0 && max_expansions > 0) {
            double w = (params.w_heur_near > 0.0 && h_start > 0.0)
                     ? params.w_heur_near + (params.w_heur-params.w_heur_near)*hw/h_start
                     : params.w_heur;
            hw *= w;
        }
        return hw;
    };

    double h_start = h_raw(start);

    detail::StateMap nodes_map;
    // 추적: state → {g, parent_state, parent_cell}
    // parent 셀 필요 (경로 역추적 시 점프 구간 복원)
    struct TraceSlot { double g; long long parent; Cell parent_cell; bool visited=false; };
    std::unordered_map<long long, TraceSlot> trace;

    std::priority_queue<detail::PQItem, std::vector<detail::PQItem>, detail::PQCmp> pq;
    long long counter = 0;
    long long expanded = 0;

    long long s0 = state_of(occ.lin(start), -1, 0);
    trace[s0] = {0.0, -1, start};
    pq.push({h_w(start,h_start), counter++, start, -1, 0});

    while (!pq.empty()) {
        auto cur = pq.top(); pq.pop();

        long long cur_lin = occ.lin(cur.cell);
        long long cur_st  = state_of(cur_lin, cur.dir, cur.run);

        // 이미 처리됐으면 건너뜀
        auto it = trace.find(cur_st);
        if (it == trace.end()) continue;
        if (it->second.g < cur.f - h_raw(cur.cell) - 1e-9) continue;  // stale

        // closed 체크 (StateMap 재활용)
        bool ins;
        auto& slot = nodes_map.emplace(cur_st, ins);
        if (!ins && slot.closed) continue;
        slot.closed = true;

        ++expanded;
        if (collect_visited) R.visited.push_back(cur.cell);

        // 목표 도달 여부
        bool at_goal = (cur.cell == goal);
        if (at_goal && (goal_dir < 0 || cur.dir == goal_dir)) {
            // 경로 역추적
            std::vector<Cell> rev;
            long long st = cur_st;
            Cell      cc = cur.cell;
            while (true) {
                auto jt = trace.find(st);
                if (jt == trace.end()) break;
                rev.push_back(cc);
                long long pst = jt->second.parent;
                Cell pc       = jt->second.parent_cell;
                if (pst < 0) break;
                // 점프 구간 복원: pc → cc 를 fine-cell 로 풀어 삽입
                // cc 와 pc 의 차이로 방향 계산
                int di = cc.i-pc.i, dj = cc.j-pc.j, dk = cc.k-pc.k;
                int steps = std::abs(di)+std::abs(dj)+std::abs(dk);
                int si = (steps>0)?(di/steps):0, sj=(steps>0)?(dj/steps):0, sk=(steps>0)?(dk/steps):0;
                for (int s=steps-1; s>0; --s)
                    rev.push_back({cc.i-si*s, cc.j-sj*s, cc.k-sk*s});
                st = pst; cc = pc;
            }
            std::reverse(rev.begin(), rev.end());
            // rev[0] 이 start 이어야 함; start 가 없으면 앞에 붙임
            if (rev.empty() || !(rev.front() == start)) rev.insert(rev.begin(), start);

            R.success      = true;
            R.path         = std::move(rev);
            R.length_mm    = (R.path.size()>1) ? (double)(R.path.size()-1)*cmm : 0.0;
            R.turns        = count_turns(R.path);
            R.expanded_nodes = expanded;
            R.elapsed_ms   = ms();
            return R;
        }

        if (max_expansions > 0 && expanded >= max_expansions) {
            R.fail = RouteFail::ExpansionLimit;
            break;
        }

        double g_cur = it->second.g;

        // ── 6방향 점프 ──────────────────────────────────────────
        for (int ax = 0; ax < 6; ++ax) {
            // min_straight 제약: 이미 진행 중(dir>=0)이고 꺾이려면 run >= min_run
            bool is_turn = (cur.dir >= 0 && ax != cur.dir);
            if (is_turn && min_run > 1 && cur.run < min_run) continue;

            int di = JUMP_DI[ax], dj = JUMP_DJ[ax], dk = JUMP_DK[ax];
            Cell nc1{cur.cell.i+di, cur.cell.j+dj, cur.cell.k+dk};
            if (!occ.in_bounds(nc1) || occ.is_blocked(nc1)) continue;

            // 이 방향으로 점프 가능 거리
            int jmp = occ.max_jump(cur.cell, ax);
            if (jmp <= 0) continue;

            // 목표가 이 방향 직선상에 있으면 거기서 멈춤
            int dist_to_goal = -1;
            {
                int dgi=goal.i-cur.cell.i, dgj=goal.j-cur.cell.j, dgk=goal.k-cur.cell.k;
                if (di&&!dj&&!dk && !dgj&&!dgk && dgi*di>0) dist_to_goal=std::abs(dgi);
                else if (!di&&dj&&!dk && !dgi&&!dgk && dgj*dj>0) dist_to_goal=std::abs(dgj);
                else if (!di&&!dj&&dk && !dgi&&!dgj && dgk*dk>0) dist_to_goal=std::abs(dgk);
            }

            // 생성할 이웃 셀 거리 목록 (리프 경계 + 목표까지)
            // 두 후보: max_jump(리프 경계) 와 dist_to_goal(목표)
            static std::array<int,2> cands;
            int ncands = 1;
            cands[0] = jmp;
            if (dist_to_goal > 0 && dist_to_goal < jmp) {
                cands[1] = dist_to_goal; ncands = 2;
            }
            // 중복 제거는 불필요(항상 2개 이하)

            for (int ci = 0; ci < ncands; ++ci) {
                int s = cands[ci];
                Cell nb{cur.cell.i+di*s, cur.cell.j+dj*s, cur.cell.k+dk*s};
                if (!occ.in_bounds(nb) || occ.is_blocked(nb)) continue;

                // 이동 비용 (점프 거리 × cell_mm + 꺾임 페널티)
                double move = (double)s * cmm;
                if (is_turn) move += params.w_turn;
                // 도착 셀 페널티 (클리어런스 등) — 간략화: 점프 중간 셀은 무시
                // (자유 리프 내부이므로 장애물 클리어런스 페널티 없음)

                double g_nb = g_cur + move;
                int new_run = is_turn ? s : (cur.run + s);
                long long nb_lin = occ.lin(nb);
                long long nb_st  = state_of(nb_lin, ax, new_run);

                auto& tr = trace[nb_st];
                if (!tr.visited || g_nb < tr.g - 1e-12) {
                    tr.g           = g_nb;
                    tr.parent      = cur_st;
                    tr.parent_cell = cur.cell;
                    tr.visited     = true;

                    double f = g_nb + h_w(nb, h_start);
                    pq.push({f, counter++, nb, ax, new_run});

                    bool dummy;
                    nodes_map.emplace(nb_st, dummy).closed = false;
                }
            }
        }
    }

    if (R.fail == RouteFail::None) R.fail = RouteFail::NoPath;
    R.expanded_nodes = expanded;
    R.elapsed_ms     = ms();
    return R;
}

}  // namespace routing3d
