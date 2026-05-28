// 점유맵 (Occupancy Map) — Routing3D C++ 엔진 (Phase 3, Step 3.2)
// =============================================================================
// [이 파일이 하는 일]
//   플랜트 공간을 cell_mm 정육면체 셀 격자로 표현하고 각 셀의 점유 여부를 관리한다.
//   백엔드 2종(질의 인터페이스 동일 — 불변식 O1):
//     - DenseOccupancy  : 연속 메모리 1B/셀. 작은 ROI 에 빠름.
//     - SparseOccupancy : 점유 셀만 해시셋 보관. 메모리 = O(점유 셀). 8,000m 등 초대형 격자용.
//   좌표 변환·복셀화는 geometry.hpp 의 공유 함수(grid_*)를 써서 두 백엔드가 정확히 일치한다.
//   질의 인터페이스(is_blocked/in_bounds/to_world/to_cell)는 명세 algorithm_spec.md §1,§2 와 대응.
//   OpenVDB 희소 백엔드는 Step 3.6 에서 동일 인터페이스로 추가한다.
//
// [빌드/실행]  cmake --build cpp/build --config Release  (geometry.hpp 헤더 참고)
// =============================================================================
#pragma once

#include <cstdint>
#include <unordered_set>
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

    // 깊은 사본(장애물 + 점유 그대로 복사). 다중 배관 라우팅의 '작업용 사본'에 사용.
    // (명세 §5: work = occ.copy() — 원본 보존). 기본 복사 생성자와 동일하나 의도를 드러낸다.
    DenseOccupancy copy() const { return *this; }

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

// 희소 점유맵 — 점유 셀의 패킹 키만 해시셋에 보관. 메모리 = O(점유 셀 수).
// 초대형 격자(예: 8,000m / 50mm = 160,000셀/축)도 점유가 적으면 메모리가 작다.
// 좌표/복셀화 규칙은 Dense 와 동일(geometry.hpp 공유 함수) → 동일 입력에 동일 질의 결과(O1).
// 주의: lin()/size() 는 Dense 와 같은 의미지만 초대형 격자에서는 int 범위를 넘으므로(A* 미사용)
//       대형 인스턴스에서는 호출하지 않는다. 점유 저장 키는 별도 64비트 패킹을 쓴다.
class SparseOccupancy {
public:
    SparseOccupancy(Cell shape, Vec3 origin, double cell_mm);

    // ---- 질의 (DenseOccupancy 와 동일 계약) ----
    bool in_bounds(const Cell& c) const { return grid_in_bounds(c, shape_); }
    bool is_blocked(const Cell& c) const;
    Vec3 to_world(const Cell& c) const { return grid_cell_to_world(c, origin_, cell_); }
    Cell to_cell(const Vec3& w) const { return grid_world_to_cell(w, origin_, cell_); }

    // ---- 변경 ----
    void block_cell(const Cell& c);
    int add_box(const AABB& box);

    // ---- 메타/통계 ----
    long long count_blocked() const { return static_cast<long long>(blocked_.size()); }
    Cell shape() const { return shape_; }
    Vec3 origin() const { return origin_; }
    double cell_mm() const { return cell_; }
    long long size() const { return static_cast<long long>(shape_.i) * shape_.j * shape_.k; }

    // A* 의 g/closed 키(Dense 와 동일 공식, int). 초대형 격자에서는 호출 금지.
    int lin(const Cell& c) const { return c.i + shape_.i * (c.j + shape_.j * c.k); }
    Cell unlin(int idx) const;

    SparseOccupancy copy() const { return *this; }

private:
    // 점유 셀 저장용 64비트 패킹 키(축당 21비트, 최대 2,097,151 → 8,000m/50mm=160k 여유).
    static uint64_t pack(const Cell& c) {
        return (static_cast<uint64_t>(c.i) << 42) | (static_cast<uint64_t>(c.j) << 21) |
               static_cast<uint64_t>(c.k);
    }

    Cell shape_;
    Vec3 origin_;
    double cell_;
    std::unordered_set<uint64_t> blocked_;
};

}  // namespace routing3d
