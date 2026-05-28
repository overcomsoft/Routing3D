// OpenVDB 점유맵 (VdbOccupancy) — Routing3D C++ 엔진 (Phase 3, Step 3.6)
// =============================================================================
// [이 파일이 하는 일]
//   OpenVDB(BoolGrid) 를 점유 비트맵으로 사용하는 희소 백엔드. DenseOccupancy 와 동일
//   질의 인터페이스(불변식 O1). 좌표/복셀화는 geometry.hpp 공유 함수로 Dense/Sparse 와 일치.
//
//   핵심 이점: 'add_box' 가 OpenVDB 의 fill(타일 최적화)을 써서 **꽉 찬 영역을 타일로 압축**한다.
//   8,000m 바닥/천장 같은 큰 시트도 복셀당이 아니라 타일로 저장 → 메모리 소량(<32GB 목표 달성).
//   (해시셋 SparseOccupancy 는 시트에서 복셀이 폭증하지만, OpenVDB 는 타일로 해결.)
//
//   OpenVDB 타입은 헤더에 노출하지 않는다(pimpl) → 이 헤더는 OpenVDB 없이도 include 가능.
//
// [빌드/실행]  (OpenVDB = vcpkg x64-windows; 기본 OFF)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64 `
//     -DUSE_OPENVDB=ON `
//     -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake `
//     -DVCPKG_TARGET_TRIPLET=x64-windows
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release -R vdb --output-on-failure
// =============================================================================
#pragma once

#include <memory>

#include "routing3d/geometry.hpp"

namespace routing3d {

class VdbOccupancy {
public:
    VdbOccupancy(Cell shape, Vec3 origin, double cell_mm);
    ~VdbOccupancy();
    VdbOccupancy(const VdbOccupancy& other);             // 깊은 복사(그리드 deepCopy).
    VdbOccupancy& operator=(const VdbOccupancy& other);
    VdbOccupancy(VdbOccupancy&&) noexcept;
    VdbOccupancy& operator=(VdbOccupancy&&) noexcept;

    // ---- 질의 (DenseOccupancy 와 동일 계약) ----
    bool in_bounds(const Cell& c) const { return grid_in_bounds(c, shape_); }
    bool is_blocked(const Cell& c) const;  // 격자 밖 = 점유(G1), 그 외 = 활성 복셀 여부.
    Vec3 to_world(const Cell& c) const { return grid_cell_to_world(c, origin_, cell_); }
    Cell to_cell(const Vec3& w) const { return grid_world_to_cell(w, origin_, cell_); }

    // ---- 변경 ----
    void block_cell(const Cell& c);
    long long add_box(const AABB& box);  // fill(타일) 사용. 신규 점유 복셀 수(시트는 클 수 있음).

    // ---- 메타/통계 ----
    long long count_blocked() const;  // 활성 복셀 수(타일 포함 논리 개수).
    long long memory_bytes() const;   // 그리드 메모리 사용량(타일 압축 반영).
    Cell shape() const { return shape_; }
    Vec3 origin() const { return origin_; }
    double cell_mm() const { return cell_; }
    long long size() const { return static_cast<long long>(shape_.i) * shape_.j * shape_.k; }

    // A* g/closed 키(Dense 와 동일 공식). 초대형 격자에서는 호출 금지.
    int lin(const Cell& c) const { return c.i + shape_.i * (c.j + shape_.j * c.k); }
    Cell unlin(int idx) const;

    VdbOccupancy copy() const { return *this; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    Cell shape_;
    Vec3 origin_;
    double cell_;
};

}  // namespace routing3d
