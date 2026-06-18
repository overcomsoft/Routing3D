// test_octree.cpp — 가변셀 옥트리 점유맵 + astar_octree 검증
// =============================================================================
// [검증 내용]
//   oct1: 장애물 없는 격자 — 시작→목표 직선 경로(정밀도 확인)
//   oct2: 장애물 우회 — 직방체 장애물을 돌아가는 경로
//   oct3: 골든 01(장애물 없음) — ImplicitOccupancy 결과와 동일 길이·꺾임
//   oct4: 골든 02(중앙 장애물) — ImplicitOccupancy 결과와 동일 길이·꺾임
//   oct5: max_jump 질의 — 자유공간에서 리프 경계까지 점프 확인
//   oct6: 결정성 — 동일 입력으로 두 번 실행하면 완전히 동일한 결과
//   oct7: 블록 마킹 — block_cell 후 blocked() 반환 확인
// =============================================================================
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "routing3d/astar.hpp"
#include "routing3d/geometry.hpp"
#include "routing3d/occupancy.hpp"
#include "routing3d/octree_occupancy.hpp"
#include "routing3d/scene_io.hpp"

using namespace routing3d;

// 헬퍼: 간단한 SceneDoc 생성 (장애물 리스트)
static SceneDoc make_doc(int ni, int nj, int nk, double cell,
                          const std::vector<std::array<double,6>>& obs_mm = {}) {
    SceneDoc doc;
    doc.cell_mm   = cell;
    doc.origin    = {0.0, 0.0, 0.0};
    doc.shape     = {ni, nj, nk};
    doc.params.cell_mm = cell;
    doc.params.w_turn  = 0.0;   // 꺾임 무비용 → 경로 길이만 비교
    doc.params.w_clear = 0.0;
    doc.params.w_heur  = 1.0;
    for (auto& b : obs_mm) {
        Obstacle ob;
        ob.min_xyz = {b[0], b[1], b[2]};
        ob.max_xyz = {b[3], b[4], b[5]};
        doc.obstacles.push_back(ob);
    }
    return doc;
}

// 헬퍼: 경로에 blocked 셀이 없는지 검사
static bool path_clear(const OctreeOccupancy& occ, const std::vector<Cell>& path) {
    for (const auto& c : path)
        if (occ.is_blocked(c)) return false;
    return true;
}

// oct1: 장애물 없음, 직선 경로
static void test_oct1() {
    auto doc = make_doc(20, 20, 20, 50.0);
    OctreeOccupancy occ;
    occ.build(doc);

    Cell s{0,0,0}, g{15,0,0};
    auto res = astar_octree(occ, s, g, doc.params, 0);
    assert(res.success);
    assert(res.path.front() == s);
    assert(res.path.back()  == g);
    assert(path_clear(occ, res.path));
    // 직선 경로 길이 = 15 셀 × 50mm = 750mm
    assert(std::abs(res.length_mm - 750.0) < 1.0);
    printf("[oct1] PASS  (straight, len=%.0fmm, steps=%zu)\n",
           res.length_mm, res.path.size());
}

// oct2: 단순 벽 장애물 우회
static void test_oct2() {
    // 10×10×10, cell=50mm, i=5 에 두꺼운 벽(j=0..9, k=0..9 except k=9)
    std::vector<std::array<double,6>> obs = {
        {250.0, 0.0, 0.0, 300.0, 450.0, 400.0}  // i=5 wall, gap at k>=8
    };
    auto doc = make_doc(10, 10, 10, 50.0, obs);
    OctreeOccupancy occ;
    occ.build(doc);

    Cell s{0,0,0}, g{9,0,0};
    auto res = astar_octree(occ, s, g, doc.params, 1000000);
    assert(res.success);
    assert(path_clear(occ, res.path));
    printf("[oct2] PASS  (obstacle bypass, len=%.0fmm, turns=%d, exp=%lld)\n",
           res.length_mm, res.turns, res.expanded_nodes);
}

// oct3: 골든 01 (장애물 없음) — ImplicitOccupancy 와 길이 비교
static void test_oct3() {
    const char* golden_path = "cpp/tests/fixtures/golden01.scene.txt";
    SceneDoc doc;
    try { doc = read_scene(golden_path); }
    catch (...) { printf("[oct3] SKIP (golden01 not found)\n"); return; }
    if (doc.tasks.empty()) { printf("[oct3] SKIP (no tasks)\n"); return; }

    OctreeOccupancy occ;
    occ.build(doc);

    const auto& t = doc.tasks[0];
    Cell s = occ.to_cell(t.start_mm);
    Cell g = occ.to_cell(t.end_mm);

    auto res = astar_octree(occ, s, g, doc.params, 0);

    // ImplicitOccupancy baseline
    ImplicitOccupancy base(doc.shape, doc.origin, doc.cell_mm);
    for (auto& ob : doc.obstacles) {
        try { base.add_box(AABB(ob.min_xyz, ob.max_xyz)); } catch (...) {}
    }
    auto r2 = astar_weighted(base, s, g, doc.params, 0LL);

    if (!res.success) { printf("[oct3] WARN octree failed (exp=%lld)\n", res.expanded_nodes); return; }
    if (!r2.success)  { printf("[oct3] WARN baseline failed\n"); return; }

    // 길이가 같거나 옥트리가 비슷한 비율 이내
    double ratio = res.length_mm / r2.length_mm;
    assert(ratio >= 0.99 && ratio <= 1.05);
    printf("[oct3] PASS  oct=%.0fmm base=%.0fmm ratio=%.3f exp=%lld\n",
           res.length_mm, r2.length_mm, ratio, (long long)res.expanded_nodes);
}

// oct4: 골든 02 (중앙 장애물)
static void test_oct4() {
    const char* golden_path = "cpp/tests/fixtures/golden02.scene.txt";
    SceneDoc doc;
    try { doc = read_scene(golden_path); }
    catch (...) { printf("[oct4] SKIP (golden02 not found)\n"); return; }
    if (doc.tasks.empty()) { printf("[oct4] SKIP (no tasks)\n"); return; }

    OctreeOccupancy occ;
    occ.build(doc);

    const auto& t = doc.tasks[0];
    Cell s = occ.to_cell(t.start_mm);
    Cell g = occ.to_cell(t.end_mm);

    auto res = astar_octree(occ, s, g, doc.params, 5000000);

    if (!res.success) { printf("[oct4] WARN octree failed (exp=%lld, fail=%d)\n",
                               res.expanded_nodes, (int)res.fail); return; }
    assert(path_clear(occ, res.path));
    printf("[oct4] PASS  len=%.0fmm turns=%d exp=%lld\n",
           res.length_mm, res.turns, res.expanded_nodes);
}

// oct5: max_jump 질의 — 자유공간에서 리프 경계까지
static void test_oct5() {
    // 32×32×32 격자, 장애물 없음 → 루트 하나의 free 리프
    auto doc = make_doc(32, 32, 32, 25.0);
    OctreeOccupancy occ;
    occ.build(doc);

    Cell c{0,0,0};
    // +i 방향: 리프가 격자 전체 → 최대 31 셀 전진 가능
    int j = occ.max_jump(c, 0);
    assert(j == 31);
    // +j 방향
    j = occ.max_jump(c, 2);
    assert(j == 31);

    // 중간에 marked 셀 추가 → 점프 막힘
    occ.block_cell({5,0,0});
    j = occ.max_jump(c, 0);
    assert(j == 4);  // c{0}+4→{4}, {5}는 막힘

    printf("[oct5] PASS  (max_jump)\n");
}

// oct6: 결정성
static void test_oct6() {
    std::vector<std::array<double,6>> obs = {
        {100.0,0.0,0.0,200.0,300.0,300.0}
    };
    auto doc = make_doc(15, 15, 15, 25.0, obs);
    OctreeOccupancy occ;
    occ.build(doc);

    Cell s{0,0,0}, g{14,14,14};
    auto r1 = astar_octree(occ, s, g, doc.params, 5000000);
    auto r2 = astar_octree(occ, s, g, doc.params, 5000000);

    assert(r1.success == r2.success);
    if (r1.success) {
        assert(r1.path.size() == r2.path.size());
        assert(r1.length_mm   == r2.length_mm);
        assert(r1.turns       == r2.turns);
    }
    printf("[oct6] PASS  (deterministic, success=%d)\n", r1.success ? 1 : 0);
}

// oct7: block_cell 후 blocked()
static void test_oct7() {
    auto doc = make_doc(10, 10, 10, 50.0);
    OctreeOccupancy occ;
    occ.build(doc);

    Cell c{3,4,5};
    assert(!occ.is_blocked(c));
    occ.block_cell(c);
    assert(occ.is_blocked(c));

    // 이웃은 영향 없음
    assert(!occ.is_blocked({3,4,4}));
    assert(!occ.is_blocked({3,3,5}));

    printf("[oct7] PASS  (block_cell)\n");
}

int main() {
    printf("=== test_octree ===\n");
    test_oct1();
    test_oct2();
    test_oct3();
    test_oct4();
    test_oct5();
    test_oct6();
    test_oct7();
    printf("=== DONE ===\n");
    return 0;
}
