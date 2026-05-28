// 점유맵 (Occupancy Map) — Routing3D C++ 엔진 (Phase 3, Step 3.2)
// =============================================================================
// [이 파일이 하는 일]
//   플랜트 공간을 cell_mm 정육면체 셀 격자로 표현하고 각 셀의 점유 여부를 관리한다.
//   현재 백엔드는 Dense(연속 메모리 1B/셀). 질의 인터페이스(is_blocked/in_bounds/
//   to_world/to_cell)는 명세 algorithm_spec.md §1,§2 의 계약과 1:1 대응한다.
//   OpenVDB 희소 백엔드는 Step 3.6 에서 동일 인터페이스로 추가한다.
//
// [빌드/실행]  cmake --build cpp/build --config Release  (geometry.hpp 헤더 참고)
// =============================================================================
#pragma once

#include <cstdint>
#include <vector>

#include "routing3d/geometry.hpp"

namespace routing3d {

class DenseOccupancy {
public:
    DenseOccupancy(Cell shape, Vec3 origin, double cell_mm);

    // ---- 질의 (백엔드 무관 계약) ----
    bool in_bounds(const Cell& c) const;
    bool is_blocked(const Cell& c) const;  // 격자 밖이면 true
    Vec3 to_world(const Cell& c) const;     // 셀 중심 월드좌표(mm)
    Cell to_cell(const Vec3& w) const;       // 포함 셀 인덱스

    // ---- 변경 ----
    void block_cell(const Cell& c);          // in_bounds 일 때만 점유
    int add_box(const AABB& box);            // 복셀화, 신규 점유 셀 수 반환

    // ---- 메타/통계 ----
    long long count_blocked() const;
    Cell shape() const { return shape_; }
    Vec3 origin() const { return origin_; }
    double cell_mm() const { return cell_; }
    long long size() const { return static_cast<long long>(shape_.i) * shape_.j * shape_.k; }

    // 선형 인덱스 (i + nx*(j + ny*k)). A* 의 g/closed 키로 사용.
    int lin(const Cell& c) const { return c.i + shape_.i * (c.j + shape_.j * c.k); }
    Cell unlin(int idx) const;

    const std::vector<uint8_t>& raw() const { return grid_; }

private:
    Cell shape_;
    Vec3 origin_;
    double cell_;
    std::vector<uint8_t> grid_;  // 1B/셀, 1=점유
};

}  // namespace routing3d
