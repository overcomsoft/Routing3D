// 점유맵 (Occupancy Map) — Routing3D C++ 엔진 (Phase 3, Step 3.2)
// =============================================================================
// [이 파일이 하는 일]
//   3D 플랜트 공간을 cell_mm 정육면체 셀 격자로 표현하고 각 셀의 점유 상태를
//   관리한다. 백엔드 3종의 질의 인터페이스가 동일(불변식 O1):
//     - DenseOccupancy  : 연속 메모리 1B/셀. 소형 ROI 에서 빠름.
//     - SparseOccupancy : 점유 셀만 해시셋에 보관. 메모리 = O(점유 셀). 8,000m 등 초대형 격자.
//     - ImplicitOccupancy: 장애물 AABB 를 복셀화하지 않고 SpatialBoxIndex 에 보관,
//                          필요 시 온디맨드로 겹침 질의. 메모리 O(장애물+배관셀). 5M+ 셀 자동 전환.
//   좌표 변환·복셀화는 geometry.hpp 의 공유 함수(grid_*)를 써서 모든 백엔드가 정확히 일치한다.
//   질의 인터페이스(is_blocked/in_bounds/to_world/to_cell)는 명세 algorithm_spec.md §1,§2 와 대응.
//   OpenVDB 추가 백엔드는 Step 3.6 에서 동일 인터페이스로 추가했다.
//
// [빌드/실행]  cmake --build cpp/build --config Release  (geometry.hpp 헤더 참고)
//
// =============================================================================
// [전체 모듈 개요]
//
// 점유맵(Occupancy Map)은 A* 라우팅의 **가·불가 판정 계층**이다.
// A* 가 이웃 셀을 확장할 때 is_blocked() 를 호출하여 통과 불가 셀을 제거하고,
// 배관 경로가 확정되면 block_cell() 로 해당 셀을 점유 표시하여
// 다음 배관이 그 셀을 통과하지 못하게 한다(다중 배관 충돌 회피 M1·M2).
//
// [백엔드 선택 기준]
//   - DenseOccupancy   : 격자 총 셀 < 5M (예: cell=100mm, 200×150×150 ≈ 4.5M)
//   - SparseOccupancy  : 초대형 격자이나 점유 셀이 희소한 corridor 탐색용
//   - ImplicitOccupancy: 격자 총 셀 ≥ 5M (예: cell=25mm → 수억 셀)
//     capi 가 크기를 보고 자동 전환. 10mm 도메인(17.8억 셀)도 메모리 안전.
//
// [불변식 O1 — 질의 결과 동일]
//   동일 장애물·배관 셀로 구성된 세 백엔드는 동일한 월드 좌표에 대해
//   동일한 is_blocked() 결과를 반환해야 한다.
//   이를 위해 좌표 변환·복셀화 함수를 geometry.hpp 에서 공유한다.
//
// [불변식 M2 — 원본 점유맵 불변]
//   route_multi 는 원본 점유맵의 copy() 를 만들어 작업별 사본을 사용하므로
//   원본 장애물 점유맵은 절대 변경되지 않는다.
//   copy() 는 각 백엔드마다 구현되며 의미론적으로 deep copy 이다.
//
// [인덱스 계약 A4 — lin/unlin 의 비트 폭]
//   Dense/Sparse 의 lin() 은 int(32비트) 를 반환하나,
//   ImplicitOccupancy 의 lin() 은 long long(64비트)를 반환한다.
//   A* state_of 계산은 항상 long long 으로 처리하여 격자 크기에 무관하게 안전.
//
// [의존 관계]
//   geometry.hpp (Cell/Vec3/AABB, grid_* 유틸)
//   box_index.hpp (SpatialBoxIndex — ImplicitOccupancy 의 AABB 인덱스)
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

// =============================================================================
// DenseOccupancy — 밀집 배열 점유맵 (1B/셀, 연속 메모리)
// =============================================================================
// 격자의 모든 셀을 uint8_t 배열(1B/셀)에 저장한다.
// 배열 인덱스 = lin(i,j,k) = i + nx*(j + ny*k) (행 우선, z가 가장 느린 축).
//
// 장점:
//   - is_blocked(): 배열 랜덤 접근 O(1), 캐시 친화적 순차 접근.
//   - add_box(): 격자 범위를 순회하며 1 을 기록. O(복셀 수).
//   - 구현 단순, 골든 테스트(ctest golden)의 기준 백엔드.
//
// 단점:
//   - 메모리 = shape.i × shape.j × shape.k 바이트. 5M 셀 = 5MB, 18억 셀 = 1.7GB(불가).
//   - 격자 전체를 미리 할당하므로 대형 도메인에 부적합.
//
// 사용 시나리오:
//   소형 씬(cell≥100mm, 셀 수 < 5M), 골든 테스트, Python pybind11 바인딩.
//   5M 이상이면 capi 가 자동으로 ImplicitOccupancy 로 전환한다.
class DenseOccupancy {
public:
    // -------------------------------------------------------------------
    // 생성자 — 격자 메타 초기화 + 배열 할당
    // -------------------------------------------------------------------
    // 인자:
    //   shape   — 격자 형상 (셀 수, 각 축)
    //   origin  — 격자 원점 (셀 (0,0,0) 의 lo 코너, mm)
    //   cell_mm — 셀 크기 (mm)
    // 동작: shape.i × shape.j × shape.k 바이트 배열을 0으로 초기화한다.
    // 시간복잡도: O(N) — N = 총 셀 수 (제로 초기화)
    DenseOccupancy(Cell shape, Vec3 origin, double cell_mm);

    // ---- 질의 (백엔드 무관 계약) ----

    // -------------------------------------------------------------------
    // in_bounds — 셀이 격자 [0, shape) 범위 안에 있는지 검사
    // -------------------------------------------------------------------
    // 반환: true = 격자 내부, false = 격자 밖
    // 시간복잡도: O(1)
    bool in_bounds(const Cell& c) const;

    // -------------------------------------------------------------------
    // is_blocked — 셀이 점유(통과 불가)인지 검사
    // -------------------------------------------------------------------
    // 격자 밖(grid_in_bounds = false)이면 항상 true 반환(G1 불변식).
    // 격자 안이면 배열 값(0/1)으로 판단.
    // A* 가 이웃 확장 시 매 스텝 호출 → 성능 임계 경로.
    // 시간복잡도: O(1)
    bool is_blocked(const Cell& c) const;

    // -------------------------------------------------------------------
    // to_world — 셀 중심의 월드 좌표(mm) 반환
    // -------------------------------------------------------------------
    // grid_cell_to_world() 의 래퍼. 셀 (i,j,k) 중심의 Vec3 를 반환한다.
    // 시간복잡도: O(1)
    Vec3 to_world(const Cell& c) const;

    // -------------------------------------------------------------------
    // to_cell — 월드 좌표를 포함하는 셀 인덱스 반환
    // -------------------------------------------------------------------
    // grid_world_to_cell() 의 래퍼. floor 기반이므로 격자 밖 좌표는
    // 범위 밖 인덱스를 반환할 수 있다(호출자가 in_bounds 확인 필요).
    // 시간복잡도: O(1)
    Cell to_cell(const Vec3& w) const;

    // ---- 변경 ----

    // -------------------------------------------------------------------
    // block_cell — 셀 하나를 점유로 표시
    // -------------------------------------------------------------------
    // in_bounds 인 경우에만 grid_[lin(c)] = 1 로 설정한다.
    // 격자 밖 셀은 무시(안전 — 경계 밖 마킹 시도를 자동 제거).
    // 시간복잡도: O(1)
    void block_cell(const Cell& c);

    // -------------------------------------------------------------------
    // add_box — AABB 를 격자에 복셀화하여 새로 점유된 셀 수 반환
    // -------------------------------------------------------------------
    // grid_box_range() 로 박스의 셀 범위를 계산하고 모든 셀을 block 한다.
    // 두께 0 박스(범위가 비어 있으면)는 건너뛴다.
    // 반환값: 이번 호출로 새로 점유된 셀 수 (이미 점유였으면 카운트 안 함).
    // 시간복잡도: O(복셀 수) — 박스 크기에 비례
    int add_box(const AABB& box);

    // -------------------------------------------------------------------
    // copy — 깊은 복사본 생성
    // -------------------------------------------------------------------
    // 장애물 + 배관 그리드를 완전히 복사한다. route_multi 의 '작업별 사본'에 사용.
    // (명세 §5: work = occ.copy() 로 원본 보존, M2 불변식).
    // 기본 복사 생성자와 동일하나 명시적 의도 표현을 위해 래퍼로 선언.
    // 시간복잡도: O(N) — N = 총 셀 수
    DenseOccupancy copy() const { return *this; }

    // ---- 메타/통계 ----

    // 점유된 셀(값=1)의 총 수를 반환. 디버그·검증용.
    // 시간복잡도: O(N)
    long long count_blocked() const;
    // 격자 형상 (셀 수, 각 축).
    Cell shape() const { return shape_; }
    // 격자 원점 (셀 (0,0,0) 의 lo 코너, mm).
    Vec3 origin() const { return origin_; }
    // 셀 크기 (mm).
    double cell_mm() const { return cell_; }
    // 격자 총 셀 수 = shape.i × shape.j × shape.k.
    long long size() const { return static_cast<long long>(shape_.i) * shape_.j * shape_.k; }

    // -------------------------------------------------------------------
    // lin — 셀 → 선형 배열 인덱스 (행 우선, z 가 가장 느린 축)
    // -------------------------------------------------------------------
    // 공식: i + nx*(j + ny*k)  (nx=shape.i, ny=shape.j)
    // A* 의 g/closed 맵에서 셀을 키로 사용할 때 이 인덱스를 사용한다.
    //
    // [인덱스 계약 A4]
    //   DenseOccupancy.lin() 은 int(32비트) 를 반환한다.
    //   capi 가 5M 셀 초과 시 ImplicitOccupancy(long long lin) 으로 자동 전환하므로
    //   int 반환 백엔드는 엄격히 <5M(int 범위) 에서만 사용됨이 보장된다.
    //   A* state_of·clearance_map 의 인덱스 산술은 long long 으로 진행하여
    //   거대 격자에서도 안전(불변식 S2/S3).
    // 시간복잡도: O(1)
    int lin(const Cell& c) const { return c.i + shape_.i * (c.j + shape_.j * c.k); }

    // -------------------------------------------------------------------
    // unlin — 선형 인덱스 → 셀 역변환 (64비트 인자)
    // -------------------------------------------------------------------
    // A* state_of/7 가 long long 으로 인덱스를 전달하나,
    // Dense 에서는 항상 int 범위이므로 계산은 int 로 수행한다.
    // 시간복잡도: O(1)
    Cell unlin(long long idx) const;

    // 내부 배열에 대한 읽기 전용 참조. 시각화·직렬화에 사용.
    const std::vector<uint8_t>& raw() const { return grid_; }

private:
    // 격자 형상 (셀 수, 각 축).
    Cell shape_;
    // 격자 원점 (mm).
    Vec3 origin_;
    // 셀 크기 (mm).
    double cell_;
    // 점유 배열. 인덱스 = lin(i,j,k). 값 0=비어 있음, 1=점유.
    std::vector<uint8_t> grid_;
};

// =============================================================================
// SparseOccupancy — 희소 해시셋 점유맵
// =============================================================================
// 점유된 셀의 64비트 키만 해시셋(unordered_set)에 보관한다.
// 메모리 = O(점유 셀 수) — Dense 처럼 전체 격자를 미리 할당하지 않는다.
//
// 사용 시나리오:
//   초대형 격자(예: 8,000m / 50mm = 160,000 셀/축)에서 점유가 희소한 경우.
//   대표적으로 corridor 라우팅: coarse 격자 골격만 점유 → 나머지는 비어 있음.
//
// 좌표/복셀화 규칙은 Dense 와 동일(geometry.hpp 공유 함수) →
//   동일 입력에 대해 동일 질의 결과(O1 불변식).
//
// 주의:
//   lin()/size() 는 Dense 와 같은 공식이지만, 초대형 격자에서는 int 범위를
//   벗어나므로 A* 에서 이 백엔드를 직접 사용하지 않는다.
//   SparseOccupancy 인스턴스에서는 lin() 을 외부 호출하지 않는 것이 원칙.
//   점유 셀 관리에는 별도 64비트 팩킹(pack)이 사용된다.
class SparseOccupancy {
public:
    // -------------------------------------------------------------------
    // 생성자 — 격자 메타 초기화 (배열 미할당)
    // -------------------------------------------------------------------
    // Dense 와 달리 배열을 할당하지 않는다 → O(1) 초기화.
    SparseOccupancy(Cell shape, Vec3 origin, double cell_mm);

    // ---- 질의 (DenseOccupancy 와 동일 계약) ----

    // 격자 범위 검사 O(1). geometry.hpp grid_in_bounds() 의 인라인 래퍼.
    bool in_bounds(const Cell& c) const { return grid_in_bounds(c, shape_); }

    // -------------------------------------------------------------------
    // is_blocked — 셀 점유 검사 (해시셋 조회)
    // -------------------------------------------------------------------
    // 격자 밖이면 true(G1). 격자 안이면 blocked_ 해시셋에서 pack(c) 키 조회.
    // 평균 O(1), 최악 O(n) — 해시 충돌. 실제로는 희소하여 거의 O(1).
    bool is_blocked(const Cell& c) const;

    // 셀 중심 월드 좌표. geometry.hpp grid_cell_to_world() 의 인라인 래퍼.
    Vec3 to_world(const Cell& c) const { return grid_cell_to_world(c, origin_, cell_); }
    // 월드 좌표 → 셀 인덱스. geometry.hpp grid_world_to_cell() 의 인라인 래퍼.
    Cell to_cell(const Vec3& w) const { return grid_world_to_cell(w, origin_, cell_); }

    // ---- 변경 ----

    // -------------------------------------------------------------------
    // block_cell — 셀을 점유로 표시 (해시셋 삽입)
    // -------------------------------------------------------------------
    // in_bounds 인 경우에만 blocked_ 에 pack(c) 키를 삽입한다.
    // 평균 O(1).
    void block_cell(const Cell& c);

    // -------------------------------------------------------------------
    // add_box — AABB 복셀화 (해시셋에 범위 삽입)
    // -------------------------------------------------------------------
    // grid_box_range() 로 범위 계산 후 모든 셀을 block_cell() 로 삽입.
    // 반환: 이번 호출로 새로 삽입된 셀 수.
    // 시간복잡도: O(복셀 수)
    int add_box(const AABB& box);

    // ---- 메타/통계 ----

    // 점유 셀 수 = blocked_ 해시셋 크기. O(1).
    long long count_blocked() const { return static_cast<long long>(blocked_.size()); }
    // 격자 형상.
    Cell shape() const { return shape_; }
    // 격자 원점 (mm).
    Vec3 origin() const { return origin_; }
    // 셀 크기 (mm).
    double cell_mm() const { return cell_; }
    // 격자 총 셀 수 (= Dense 와 동일 공식, 초대형 격자에서는 int 범위 초과 주의).
    long long size() const { return static_cast<long long>(shape_.i) * shape_.j * shape_.k; }

    // -------------------------------------------------------------------
    // lin — 셀 → 선형 인덱스 (Dense 와 동일 공식, int 반환)
    // -------------------------------------------------------------------
    // A* 의 g/closed 에서 사용할 수 있으나, 초대형 격자에서는 int 범위를
    // 벗어나므로 이 백엔드 인스턴스에서는 외부에서 lin() 을 호출하지 않는다.
    // 시간복잡도: O(1)
    int lin(const Cell& c) const { return c.i + shape_.i * (c.j + shape_.j * c.k); }

    // -------------------------------------------------------------------
    // unlin — 선형 인덱스 → 셀 역변환 (64비트 인자, Sparse 에서는 int 범위 제한)
    // -------------------------------------------------------------------
    // astar_weighted 가 환원할 때 호출. Sparse 에서는 int 범위 내에서만 안전.
    // 시간복잡도: O(1)
    Cell unlin(long long idx) const;

    // 깊은 복사본(M2 불변식). blocked_ 해시셋을 복사한다.
    // 시간복잡도: O(점유 셀 수)
    SparseOccupancy copy() const { return *this; }

private:
    // -------------------------------------------------------------------
    // pack — 셀 → 64비트 해시키 변환 (축당 21비트 할당)
    // -------------------------------------------------------------------
    // (i << 42) | (j << 21) | k 로 세 축 인덱스를 하나의 uint64_t 로 합친다.
    // 21비트 = 최대 2,097,151 → 8,000m/50mm = 160,000 이하이므로 충분.
    // 주의: ImplicitOccupancy.pack 과 비트 레이아웃이 동일하지 않음
    //       (uint32_t 캐스팅 여부 차이). 두 클래스의 pack 은 독립적.
    static uint64_t pack(const Cell& c) {
        return (static_cast<uint64_t>(c.i) << 42) | (static_cast<uint64_t>(c.j) << 21) |
               static_cast<uint64_t>(c.k);
    }

    // 격자 형상.
    Cell shape_;
    // 격자 원점 (mm).
    Vec3 origin_;
    // 셀 크기 (mm).
    double cell_;
    // 점유 셀의 64비트 키 집합. 조회·삽입 모두 평균 O(1).
    std::unordered_set<uint64_t> blocked_;
};

// =============================================================================
// ImplicitOccupancy — 암시적 점유맵 (복셀화 없이 AABB 인덱스 질의)
// =============================================================================
// 장애물을 '복셀화하지 않고' AABB 그대로 SpatialBoxIndex 에 보관하고,
// 적적으로 깔린 배관 셀만 해시셋(marked_)에 둔다.
// 메모리 = O(장애물 수) + O(깔린 셀 수) — **셀 크기에 무관**.
//
// **이 백엔드가 5M+ 셀에서 핵심인 이유**:
//   - Dense: 5M 셀 = 5MB, 18억 셀 = 1.7GB → 불가.
//   - ImplicitOccupancy: 장애물 1,000개 × 평균 ~24B/AABB ≈ 24KB,
//     배관 셀 수만 tens_of_thousands → 수 MB 수준.
//     10mm 격자(1,351×1,458×903 ≈ 17.8억 셀)도 메모리 안전(실측 성공).
//
// 질의 인터페이스는 Dense/Sparse 와 동일(불변식 O1):
//   is_blocked(cell):
//     1. 격자 밖이면 true (G1)
//     2. marked_ 에 있으면 true (배관 마킹)
//     3. SpatialBoxIndex.overlaps(셀 AABB) 로 장애물과 겹치면 true
//        → 겹침 판정은 DenseOccupancy.add_box 복셀화와 의미론적으로 동일
//
// lin/unlin:
//   **64비트** long long. 10mm 격자 17.8억 셀에서도 오버플로 없음.
//   Dense/Sparse 의 int lin 과 다르므로 A* 가 백엔드를 구분하여 처리.
//
// clearance_cells:
//   온디맨드 클리어런스(S4). 전역 거리변환 배열 대신
//   SpatialBoxIndex.nearest_dist() 로 박스 최근접 거리 질의.
//   메모리 O(탐색 수), 격자 크기 무관.
//
// copy():
//   장애물 인덱스(index_)는 불변이라 shared_ptr 공유.
//   marked_(배관 셀)는 깊은 복사(M2 불변식: 사본 독립).
//
// 주의:
//   w_corridor > 0(배관 번들링) 시 corridor 가 unordered_set<int> 를 받는
//   기존 API 와 64비트 lin 이 맞지 않으므로, 대형 격자에서는 w_corridor=0.
class ImplicitOccupancy {
public:
    // -------------------------------------------------------------------
    // 생성자 — 격자 메타 + SpatialBoxIndex 초기화
    // -------------------------------------------------------------------
    // 인자:
    //   shape   — 격자 형상 (셀 수)
    //   origin  — 격자 원점 (mm)
    //   cell_mm — 셀 크기 (mm)
    // SpatialBoxIndex 의 격자 셀 크기 = max(cell_mm × 16, 500mm).
    //   작은 장애물이 많을 때 인덱스 셀이 너무 작아지는 것을 방지.
    // 시간복잡도: O(1)
    ImplicitOccupancy(Cell shape, Vec3 origin, double cell_mm)
        : shape_(shape), origin_(origin), cell_(cell_mm),
          index_(std::make_shared<SpatialBoxIndex>(std::max(cell_mm * 16.0, 500.0))) {}

    // ---- 질의 (Dense/Sparse 와 동일 계약) ----

    // 격자 범위 검사 O(1). grid_in_bounds() 인라인 래퍼.
    bool in_bounds(const Cell& c) const { return grid_in_bounds(c, shape_); }

    // -------------------------------------------------------------------
    // is_blocked — 셀 점유 검사 (3단계 순서)
    // -------------------------------------------------------------------
    // 1. 격자 밖이면 true (G1 불변식: 격자 외부 = 통과 불가).
    // 2. marked_ 해시셋에 있으면 true (배관이 깔린 셀).
    // 3. SpatialBoxIndex.overlaps(셀 AABB) — 장애물 AABB 와 겹치면 true.
    //
    // 3번 단계가 이 백엔드의 핵심: 장애물을 미리 복셀화하지 않고
    // 필요한 셀에 대해서만 즉석 AABB 겹침 검사를 수행한다.
    // 겹침 판정은 add_box 의 floor/ceil 복셀화와 의미론적으로 동일.
    //
    // 시간복잡도: 평균 O(해당 셀 인접 장애물 수) — SpatialBoxIndex 가 O(1)에 가까운 박스 후보를 반환.
    bool is_blocked(const Cell& c) const {
        if (!in_bounds(c)) return true;  // 격자 밖 = 점유(G1).
        if (marked_.find(pack(c)) != marked_.end()) return true;
        Vec3 lo{origin_.x + c.i * cell_, origin_.y + c.j * cell_, origin_.z + c.k * cell_};
        Vec3 hi{lo.x + cell_, lo.y + cell_, lo.z + cell_};
        return index_->overlaps(lo, hi);
    }

    // 셀 중심 월드 좌표. grid_cell_to_world() 인라인 래퍼. O(1).
    Vec3 to_world(const Cell& c) const { return grid_cell_to_world(c, origin_, cell_); }
    // 월드 좌표 → 셀 인덱스. grid_world_to_cell() 인라인 래퍼. O(1).
    Cell to_cell(const Vec3& w) const { return grid_world_to_cell(w, origin_, cell_); }

    // ---- 변경 ----

    // -------------------------------------------------------------------
    // block_cell — 배관 셀을 marked_ 해시셋에 삽입
    // -------------------------------------------------------------------
    // 장애물 인덱스(index_)와 달리, 배관이 깔린 셀은 동적으로 추가된다.
    // in_bounds 인 경우에만 삽입(경계 밖 마킹 무시).
    // 시간복잡도: 평균 O(1)
    void block_cell(const Cell& c) {
        if (in_bounds(c)) marked_.insert(pack(c));
    }

    // -------------------------------------------------------------------
    // add_box — 장애물 AABB 를 인덱스에 추가 (복셀화 미수행)
    // -------------------------------------------------------------------
    // Dense.add_box 와 달리 격자를 순회하지 않고 index_->add() 만 호출한다.
    // 반환값 0 = '새로 점유된 셀 수' 개념 없음(복셀화 안 함).
    // 시간복잡도: O(1) — SpatialBoxIndex 삽입
    int add_box(const AABB& box) {
        index_->add(box.lo, box.hi);
        return 0;
    }

    // ---- 메타/통계 ----

    // marked_ 해시셋 크기(배관 셀 수). 장애물 셀 수는 계산 미지원.
    // 시간복잡도: O(1)
    long long count_blocked() const { return static_cast<long long>(marked_.size()); }

    // -------------------------------------------------------------------
    // blocked_cells — 실제 점유 셀 목록 반환 (배관 + 장애물 모두)
    // -------------------------------------------------------------------
    // 1. marked_ 의 셀 열거.
    // 2. index_ 의 모든 박스에 대해 grid_box_range() 로 셀 범위 열거.
    // seen 해시셋으로 중복 제거.
    //
    // 사용처: 시각화(점유맵 표시), 통계, A3(Dense→Implicit 전환 시 상태 이관).
    // 시간복잡도: O(배관 셀 수 + 장애물 복셀 총수)
    std::vector<Cell> blocked_cells() const {
        std::vector<Cell> cells;
        std::unordered_set<uint64_t> seen;
        seen.reserve(marked_.size() + 64);

        auto append = [&](const Cell& c) {
            if (!in_bounds(c)) return;
            const uint64_t key = pack(c);
            if (seen.insert(key).second) cells.push_back(c);
        };

        for (uint64_t key : marked_) append(unpack(key));

        index_->for_each_box([&](const Vec3& lo, const Vec3& hi) {
            const CellRange r = grid_box_range(AABB(lo, hi), origin_, cell_, shape_);
            for (int k = r.lo.k; k < r.hi.k; ++k)
                for (int j = r.lo.j; j < r.hi.j; ++j)
                    for (int i = r.lo.i; i < r.hi.i; ++i)
                        append(Cell{i, j, k});
        });
        return cells;
    }

    // 격자 형상.
    Cell shape() const { return shape_; }
    // 격자 원점 (mm).
    Vec3 origin() const { return origin_; }
    // 셀 크기 (mm).
    double cell_mm() const { return cell_; }
    // 격자 총 셀 수 (long long, 18억도 안전).
    long long size() const {
        return static_cast<long long>(shape_.i) * shape_.j * shape_.k;
    }

    // -------------------------------------------------------------------
    // lin — 셀 → 64비트 선형 인덱스
    // -------------------------------------------------------------------
    // Dense/Sparse 의 int lin 과 달리 long long 을 반환한다.
    // 공식: i + nx*(j + ny*k)  (nx, ny, nz 모두 long long 산술)
    // 10mm 거대 격자에서도 오버플로 없음 (17.8억 < 2^31 = 2.1억 → 단, i는 int 범위)
    // astar_weighted 가 state_of 계산에 이 값을 직접 사용.
    // 시간복잡도: O(1)
    long long lin(const Cell& c) const {
        return static_cast<long long>(c.i) +
               static_cast<long long>(shape_.i) *
                   (static_cast<long long>(c.j) + static_cast<long long>(shape_.j) * c.k);
    }

    // -------------------------------------------------------------------
    // unlin — 64비트 선형 인덱스 → 셀 역변환
    // -------------------------------------------------------------------
    // lin() 의 역함수. A* state_of → 셀 복원에 사용.
    // 공식: i = idx % nx, j = (idx/nx) % ny, k = idx/(nx*ny)
    // 시간복잡도: O(1)
    Cell unlin(long long idx) const {
        long long nx = shape_.i, ny = shape_.j;
        int i = static_cast<int>(idx % nx);
        int j = static_cast<int>((idx / nx) % ny);
        int k = static_cast<int>(idx / (nx * ny));
        return Cell{i, j, k};
    }

    // -------------------------------------------------------------------
    // copy — 복사본 생성 (장애물 인덱스 공유, 배관 셀 독립 복사)
    // -------------------------------------------------------------------
    // 장애물 인덱스(index_)는 불변이므로 shared_ptr 공유(비용 없음).
    // marked_(배관 셀 해시셋)는 깊은 복사 → 사본이 독립적으로 배관을 추가.
    // M2 불변식: route_multi 의 '작업별 사본' 에서 원본 장애물 불변 보장.
    // 시간복잡도: O(배관 셀 수)
    ImplicitOccupancy copy() const { return *this; }

    // ---- S4: 온디맨드 클리어런스 ----

    // -------------------------------------------------------------------
    // clearance_cells — 셀 중심에서 가장 가까운 장애물 면까지 거리 (셀 단위)
    // -------------------------------------------------------------------
    // 인자:
    //   c          — 기준 셀
    //   max_radius — 탐색 최대 반경 (셀 수). 0 이하이면 즉시 반환.
    // 반환: 가장 가까운 장애물 표면까지의 거리(셀 단위), 최대 max_radius.
    //
    // 구현:
    //   1. 셀 중심 월드 좌표 계산.
    //   2. cap = max_radius × cell_mm 범위 내에서 nearest_dist() 호출.
    //   3. floor(d / cell_mm) → 셀 단위 거리 반환 (min max_radius).
    //
    // DenseOccupancy 의 전역 거리변환(vector<int>(N))과 달리:
    //   - 배열 미할당 → 메모리 O(탐색 수)
    //   - 격자 크기 무관 (18억 셀도 동일 비용)
    //   - 탐색 시마다 SpatialBoxIndex 질의 → 장애물이 많으면 상수가 커짐
    //
    // CostModel 의 HasClearanceQuery 컨셉으로 컴파일타임에 감지하여
    // DenseOccupancy 에서는 배열 기반, ImplicitOccupancy 에서는 이 함수 사용.
    //
    // 시간복잡도: O(셀 주변 장애물 후보 수) — SpatialBoxIndex 에 의존
    int clearance_cells(const Cell& c, int max_radius) const {
        if (max_radius <= 0) return max_radius;
        Vec3 ctr = grid_cell_to_world(c, origin_, cell_);
        double cap = static_cast<double>(max_radius) * cell_;  // 검색 반경(mm).
        double d = index_->nearest_dist(ctr, cap + cell_);
        int dc = static_cast<int>(std::floor(d / cell_));
        return dc < max_radius ? dc : max_radius;
    }

private:
    // -------------------------------------------------------------------
    // pack — 셀 → 64비트 해시키 (배관 셀 marked_ 용)
    // -------------------------------------------------------------------
    // uint32_t 로 먼저 캐스팅하여 음수 인덱스의 비트 부호 확장을 제거한 후
    // (i << 42) | (j << 21) | k 로 합친다.
    // SparseOccupancy.pack 과 의미상 동일하나 uint32_t 캐스팅 여부가 다름.
    // 8,000m/4mm 까지 커버(최대 인덱스 2,000,000 < 2^21 = 2,097,152).
    static uint64_t pack(const Cell& c) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(c.i)) << 42) |
               (static_cast<uint64_t>(static_cast<uint32_t>(c.j)) << 21) |
               static_cast<uint64_t>(static_cast<uint32_t>(c.k));
    }

    // -------------------------------------------------------------------
    // unpack — 64비트 해시키 → 셀 역변환
    // -------------------------------------------------------------------
    // M = 2^21 - 1 = 2,097,151. 21비트 마스크로 각 축을 추출.
    static Cell unpack(uint64_t key) {
        constexpr uint64_t M = (1ull << 21) - 1ull;
        return Cell{static_cast<int>((key >> 42) & M),
                    static_cast<int>((key >> 21) & M),
                    static_cast<int>(key & M)};
    }

    // 격자 형상.
    Cell shape_;
    // 격자 원점 (mm).
    Vec3 origin_;
    // 셀 크기 (mm).
    double cell_;
    // 장애물 AABB 인덱스. 불변(add_box 만 가능) → 복사본 간 shared_ptr 공유.
    std::shared_ptr<SpatialBoxIndex> index_;
    // 동적 점유 셀(깔린 배관). 복사본마다 독립(M2 불변식).
    std::unordered_set<uint64_t> marked_;
};

}  // namespace routing3d