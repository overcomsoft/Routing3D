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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <unordered_set>
#include <vector>

#include "routing3d/box_index.hpp"
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
    // [인덱스 계약(A4)] Occupancy 백엔드의 lin() 반환은 **항상 long long 으로 다뤄야 한다**(unlin 인자도 long long).
    //   Dense/Sparse 는 구현상 int 를 반환하나 capi 가 5M 셀 초과 시 ImplicitOccupancy(long long lin)로 자동
    //   전환하므로, int 반환 백엔드는 **소격자(<5M, int 범위)에서만** 쓰인다 → 실제 오버플로 없음. A* state_of·
    //   clearance_map 등 인덱스 연산은 long long 으로 수행해 거대격자에서도 안전(불변식 S2/S3).
    int lin(const Cell& c) const { return c.i + shape_.i * (c.j + shape_.j * c.k); }
    Cell unlin(long long idx) const;  // 64비트 인자(A* state_of/7 가 long long) — Dense 는 항상 int 범위.

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
    Cell unlin(long long idx) const;  // 64비트 인자(astar_weighted 호환). Sparse 는 int 범위 내 사용.

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

// 암시적 점유맵 — 장애물을 '복셀화하지 않고' AABB 그대로 SpatialBoxIndex 에 보관하고,
// 동적으로 깔린 배관 셀만 해시셋(marked_)에 담는다. 메모리 = O(장애물 수) + O(깔린 셀 수)로
// **셀 크기와 무관**(근본 sparse 해법). 질의 인터페이스는 Dense/Sparse 와 동일(불변식 O1).
//   - is_blocked(cell)  : 셀 AABB 와 겹치는 장애물이 있으면(또는 marked_) 점유.
//                         겹침 판정은 DenseOccupancy.add_box 의 복셀화와 사실상 동일(과소차단 없음).
//   - lin/unlin         : **64비트**(10mm·20억셀에서도 오버플로 없음). astar_weighted 가 사용.
//   - clearance_cells   : 온디맨드 클리어런스(전역 거리변환 대신 박스 최근접 질의). CostModel 이
//                         HasClearanceQuery 컨셉으로 감지해 distance transform 배열을 만들지 않는다.
//   - copy()            : 장애물 인덱스는 불변이라 shared_ptr 공유, marked_ 만 깊은 복사(M2: 원본 불변).
// 주의: w_corridor>0(배관 번들링)은 corridor 셀 키를 unordered_set<int> 로 받는 기존 API 와 64비트
//       lin 이 맞지 않으므로 이 백엔드에서는 사용하지 않는다(대형 격자 라우팅은 w_corridor=0).
class ImplicitOccupancy {
public:
    ImplicitOccupancy(Cell shape, Vec3 origin, double cell_mm)
        : shape_(shape), origin_(origin), cell_(cell_mm),
          index_(std::make_shared<SpatialBoxIndex>(std::max(cell_mm * 16.0, 500.0))) {}

    // ---- 질의 (Dense/Sparse 와 동일 계약) ----
    bool in_bounds(const Cell& c) const { return grid_in_bounds(c, shape_); }
    bool is_blocked(const Cell& c) const {
        if (!in_bounds(c)) return true;  // 격자 밖 = 점유(G1).
        if (marked_.find(pack(c)) != marked_.end()) return true;
        Vec3 lo{origin_.x + c.i * cell_, origin_.y + c.j * cell_, origin_.z + c.k * cell_};
        Vec3 hi{lo.x + cell_, lo.y + cell_, lo.z + cell_};
        return index_->overlaps(lo, hi);
    }
    Vec3 to_world(const Cell& c) const { return grid_cell_to_world(c, origin_, cell_); }
    Cell to_cell(const Vec3& w) const { return grid_world_to_cell(w, origin_, cell_); }

    // ---- 변경 ----
    void block_cell(const Cell& c) {
        if (in_bounds(c)) marked_.insert(pack(c));
    }
    // 복셀화하지 않고 장애물 AABB 를 색인에 추가(반환 0 — '신규 점유 셀 수' 개념 없음).
    int add_box(const AABB& box) {
        index_->add(box.lo, box.hi);
        return 0;
    }

    // ---- 메타/통계 ----
    long long count_blocked() const { return static_cast<long long>(marked_.size()); }
    Cell shape() const { return shape_; }
    Vec3 origin() const { return origin_; }
    double cell_mm() const { return cell_; }
    long long size() const {
        return static_cast<long long>(shape_.i) * shape_.j * shape_.k;
    }

    // 64비트 선형 인덱스(astar_weighted state_of 의 키). 10mm 거대격자에서도 오버플로 없음.
    long long lin(const Cell& c) const {
        return static_cast<long long>(c.i) +
               static_cast<long long>(shape_.i) *
                   (static_cast<long long>(c.j) + static_cast<long long>(shape_.j) * c.k);
    }
    Cell unlin(long long idx) const {
        long long nx = shape_.i, ny = shape_.j;
        int i = static_cast<int>(idx % nx);
        int j = static_cast<int>((idx / nx) % ny);
        int k = static_cast<int>(idx / (nx * ny));
        return Cell{i, j, k};
    }

    // 장애물 인덱스 공유(불변), marked_ 만 깊은 복사 → 원본 점유 불변(계약 M2).
    ImplicitOccupancy copy() const { return *this; }

    // ---- S4: 온디맨드 클리어런스 ----
    // 셀 c 중심에서 가장 가까운 장애물 표면까지 거리를 '셀 단위'로 반환(상한 max_radius).
    // 전역 distance transform(vector<int>(N))을 대체 → 메모리 = O(탐색 셀), 셀 크기 무관.
    int clearance_cells(const Cell& c, int max_radius) const {
        if (max_radius <= 0) return max_radius;
        Vec3 ctr = grid_cell_to_world(c, origin_, cell_);
        double cap = static_cast<double>(max_radius) * cell_;  // 검사 반경(mm).
        double d = index_->nearest_dist(ctr, cap + cell_);
        int dc = static_cast<int>(std::floor(d / cell_));
        return dc < max_radius ? dc : max_radius;
    }

private:
    // 깔린 배관 셀 저장용 64비트 패킹 키(축당 21비트). 8,000m/4mm 까지 커버.
    static uint64_t pack(const Cell& c) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(c.i)) << 42) |
               (static_cast<uint64_t>(static_cast<uint32_t>(c.j)) << 21) |
               static_cast<uint64_t>(static_cast<uint32_t>(c.k));
    }

    Cell shape_;
    Vec3 origin_;
    double cell_;
    std::shared_ptr<SpatialBoxIndex> index_;  // 장애물(불변, 사본끼리 공유).
    std::unordered_set<uint64_t> marked_;      // 동적 점유(깔린 배관). 사본마다 독립.
};

}  // namespace routing3d
