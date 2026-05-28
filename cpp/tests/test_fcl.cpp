// FCL 정밀 충돌 테스트 — Routing3D C++ 엔진 (Phase 3, Step 3.7)
// =============================================================================
// [이 파일이 하는 일]
//   1) FCL 점 충돌이 해석적 AABB 내부판정 및 점유맵(is_blocked)과 일치하는지(셀 중심 샘플).
//   2) 구/캡슐 질의: 거리 > 반경이면 통과, < 반경이면 충돌 → 복셀보다 정밀한 이격 검사.
//   3) 두 박스 사이 '틈'을 가는 파이프: 가는 반경은 통과, 굵은 반경은 충돌(sub-voxel 정밀).
//   4) path_clear: 폴리라인 전체 통과 판정.
//
// [빌드/실행]  (FCL = vcpkg; USE_FCL=ON)
//   cmake -S cpp -B cpp/build ... -DUSE_FCL=ON `
//     -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R fcl --output-on-failure
// =============================================================================
#include <cstdio>
#include <string>
#include <vector>

#include "routing3d/fcl_scene.hpp"
#include "routing3d/occupancy.hpp"

using namespace routing3d;

static int g_failures = 0;

static void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

// 해석적 점-내부-박스 판정(경계 포함).
static bool point_in_box(const Vec3& p, const AABB& b) {
    return p.x >= b.lo.x && p.x <= b.hi.x && p.y >= b.lo.y && p.y <= b.hi.y && p.z >= b.lo.z &&
           p.z <= b.hi.z;
}

// (1) FCL 점 충돌 == 해석 AABB == 점유맵.
static void test_point_agreement() {
    std::printf("=== FCL 점 충돌 == AABB == 점유맵 ===\n");
    const AABB box(Vec3{1000, 1000, 0}, Vec3{2000, 2000, 1000});
    FclScene scene;
    scene.add_box(box);
    scene.build();

    DenseOccupancy occ(Cell{80, 80, 40}, Vec3{0, 0, 0}, 50.0);
    occ.add_box(box);

    // 셀 중심 샘플(경계 회피)로 세 판정이 일치하는지.
    int mism_box = 0, mism_occ = 0, n = 0;
    for (int i = 5; i < 60; i += 7)
        for (int j = 5; j < 60; j += 7)
            for (int k = 2; k < 18; k += 5) {
                Vec3 p{i * 50.0 + 25.0, j * 50.0 + 25.0, k * 50.0 + 25.0};
                bool fcl = scene.collides_point(p);
                if (fcl != point_in_box(p, box)) ++mism_box;
                if (fcl != occ.is_blocked(occ.to_cell(p))) ++mism_occ;
                ++n;
            }
    std::printf("  샘플 %d개\n", n);
    check(mism_box == 0, "FCL 점 == 해석 AABB 내부판정");
    check(mism_occ == 0, "FCL 점 == 점유맵 is_blocked (셀 중심)");
}

// (2) 구 질의: 거리 대 반경.
static void test_sphere_clearance() {
    std::printf("=== 구 이격 질의 ===\n");
    FclScene scene;
    scene.add_box(AABB(Vec3{1000, 1000, 0}, Vec3{2000, 2000, 1000}));
    scene.build();
    // 박스 x면(1000)에서 100mm 떨어진 중심.
    check(!scene.collides_sphere(Vec3{900, 1500, 500}, 50.0), "거리 100 > 반경 50 → 통과");
    check(scene.collides_sphere(Vec3{900, 1500, 500}, 150.0), "거리 100 < 반경 150 → 충돌");
}

// (3) 캡슐(파이프) 선분 질의 + 틈 통과(sub-voxel).
static void test_capsule_segment() {
    std::printf("=== 캡슐 선분 / 틈 통과 ===\n");
    FclScene scene;
    scene.add_box(AABB(Vec3{1000, 1000, 0}, Vec3{2000, 2000, 1000}));
    scene.build();
    // 박스 관통 선분.
    check(!scene.segment_clear(Vec3{500, 1500, 500}, Vec3{2500, 1500, 500}, 10.0),
          "박스 관통 선분 → 충돌(통과 불가)");
    // 박스 아래(y=500, 박스 y는 1000~)를 지나는 선분.
    check(scene.segment_clear(Vec3{500, 500, 500}, Vec3{2500, 500, 500}, 10.0),
          "박스 밖 선분 → 통과");

    // 두 박스 사이 틈(y 2000~2200, 폭 200) 통과: 가는 파이프 OK, 굵은 파이프 충돌.
    FclScene gap;
    gap.add_box(AABB(Vec3{1000, 1000, 0}, Vec3{3000, 2000, 1000}));
    gap.add_box(AABB(Vec3{1000, 2200, 0}, Vec3{3000, 3200, 1000}));
    gap.build();
    // 틈 중앙 y=2100 을 x축 따라 통과. 각 박스면까지 거리 100mm.
    check(gap.segment_clear(Vec3{1500, 2100, 500}, Vec3{2500, 2100, 500}, 50.0),
          "틈(폭200) 가는 파이프(r50) → 통과");
    check(!gap.segment_clear(Vec3{1500, 2100, 500}, Vec3{2500, 2100, 500}, 150.0),
          "틈(폭200) 굵은 파이프(r150) → 충돌");
}

// (4) path_clear 폴리라인.
static void test_path_clear() {
    std::printf("=== path_clear 폴리라인 ===\n");
    FclScene scene;
    scene.add_box(AABB(Vec3{1000, 1000, 0}, Vec3{2000, 2000, 1000}));
    scene.build();
    // 박스를 위(y)로 우회하는 폴리라인.
    std::vector<Vec3> ok{{500, 500, 500}, {500, 2500, 500}, {2500, 2500, 500}, {2500, 500, 500}};
    check(scene.path_clear(ok, 20.0), "우회 폴리라인 → 통과");
    // 박스를 관통하는 폴리라인.
    std::vector<Vec3> bad{{500, 1500, 500}, {2500, 1500, 500}};
    check(!scene.path_clear(bad, 20.0), "관통 폴리라인 → 충돌");
}

int main() {
    test_point_agreement();
    test_sphere_clearance();
    test_capsule_segment();
    test_path_clear();
    std::printf("\n%s (failures=%d)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
