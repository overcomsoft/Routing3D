// FCL 정밀 충돌 장면 (Precise Collision) — Routing3D C++ 엔진 (Phase 3, Step 3.7)
// =============================================================================
// [이 파일이 하는 일]
//   장애물 AABB 들을 FCL 충돌 객체(broadphase)로 담고, **배관(반경 r 캡슐)** 과의 정밀
//   기하 충돌을 질의한다. 점유맵(50mm 복셀)은 보수적·이산적이라, 실제 파이프 굵기/이격을
//   복셀보다 정확히 검사해야 하는 영역에서 FCL 로 보강한다(명세 phase3_plan.md §4.7).
//
//   사용 예: A* 가 찾은 셀 경로를 월드 폴리라인으로 바꾼 뒤, path_clear(점들, 반경) 로
//   파이프가 장애물과 실제로 간섭하지 않는지 sub-voxel 정밀 검증.
//
//   FCL/Eigen 타입은 헤더에 노출하지 않는다(pimpl) → 이 헤더는 FCL 없이도 include 가능.
//
// [빌드/실행]  (FCL = vcpkg x64-windows; 기본 OFF)
//   cmake -S cpp -B cpp/build ... -DUSE_FCL=ON `
//     -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R fcl --output-on-failure
// =============================================================================
#pragma once

#include <memory>
#include <vector>

#include "routing3d/geometry.hpp"

namespace routing3d {

class FclScene {
public:
    FclScene();
    ~FclScene();
    FclScene(FclScene&&) noexcept;
    FclScene& operator=(FclScene&&) noexcept;
    FclScene(const FclScene&) = delete;             // broadphase 가 객체 포인터를 보유 → 복사 금지.
    FclScene& operator=(const FclScene&) = delete;

    // 장애물 박스 등록(단위 mm). build() 전에 모두 추가한다.
    void add_box(const AABB& box);
    // broadphase 자료구조 확정(질의 전 1회 호출). 추가 후 다시 호출 가능.
    void build();
    size_t size() const;  // 등록된 박스 수.

    // ---- 정밀 질의 (mm) ----
    // 중심 center, 반경 radius 구가 장애물과 충돌하면 true.
    bool collides_sphere(const Vec3& center, double radius) const;
    // 점이 장애물 내부면 true(아주 작은 구로 근사).
    bool collides_point(const Vec3& p) const;
    // 선분 a→b 를 축으로 반경 radius 인 캡슐(파이프)이 장애물과 충돌하지 않으면 true(=통과 가능).
    bool segment_clear(const Vec3& a, const Vec3& b, double radius) const;
    // 폴리라인(점 목록)의 모든 구간이 통과 가능하면 true.
    bool path_clear(const std::vector<Vec3>& pts, double radius) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace routing3d
