// 기하/격자 기본 타입 (Geometry) — Routing3D C++ 엔진 (Phase 3)
// =============================================================================
// [이 파일이 하는 일]
//   셀 인덱스(Cell), 월드 좌표(Vec3), 축정렬 박스(AABB), 6방향 이웃 상수,
//   맨해튼 거리 등 엔진 전반이 공유하는 기본 타입을 정의한다. 단위는 mm.
//   명세: docs/spec/algorithm_spec.md §0,§1,§7 와 1:1 대응.
//
// [빌드/실행]  (프로젝트 루트에서)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release
//   ctest --test-dir cpp/build -C Release
//
// =============================================================================
// [전체 모듈 개요]
//
// 이 파일은 Routing3D 엔진의 **기하 기반(Geometric Foundation)** 계층이다.
// 모든 상위 모듈(occupancy, astar, cost, scene_io 등)이 이 파일의 타입을
// 공통으로 사용하며, 아래 세 가지 목적을 달성한다.
//
//  1. 단위 통일: 모든 좌표·치수를 mm(밀리미터) 단위로 표현한다.
//     플랜트 배관 도메인은 수 km 범위를 mm 정밀도로 다루므로 double 사용.
//
//  2. 격자-월드 쌍방향 변환: 연속 mm 공간과 이산 셀 격자(cell_mm 크기) 사이를
//     grid_world_to_cell / grid_cell_to_world 한 쌍으로 변환한다.
//     모든 점유맵 백엔드(Dense/Sparse/Implicit)가 이 함수를 공유하므로
//     "같은 월드 좌표 → 같은 셀"이 백엔드 무관하게 보장된다(불변식 O1).
//
//  3. A* 결정성 보장(불변식 A2/W1): NEIGHBORS_6 의 6이웃 순서를 컴파일타임
//     상수로 고정하여, 동일 입력에서 항상 동일한 탐색 순서·경로가 나온다.
//     tie-break 우선순위(f, counter) 와 이웃 순서를 고정하면 A* 가 결정적.
//
// [의존 관계]
//   <algorithm> <array> <cmath> <cstdint> <stdexcept> — STL 헤더만 사용,
//   외부 의존성 없음. 헤더 전용(inline 함수만) → 별도 .cpp 컴파일 불필요.
// =============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace routing3d {

// =============================================================================
// Cell — 정수 셀 인덱스
// =============================================================================
// 3D 격자 내 한 셀의 (i, j, k) 인덱스를 나타낸다.
// 인덱스는 0 기반이며, 셀 (0,0,0) 이 격자 origin 위치에 해당한다.
//
// 좌표축 대응:
//   i = X 축 방향 (동-서),  j = Y 축 방향 (남-북),  k = Z 축 방향 (수직)
//
// 불변식:
//   - 유효 인덱스: 0 ≤ i < shape.i, 0 ≤ j < shape.j, 0 ≤ k < shape.k
//   - 격자 경계 밖 셀은 '점유(blocked)' 로 처리(is_blocked 가 true 반환)
//
// 사용처:
//   A* 상태 키, 경로(path) 표현, 점유맵 질의·변경 인자 전반
struct Cell {
    // X 축 격자 인덱스. 값이 클수록 origin 에서 +X 방향으로 멀어진다.
    int i{0};
    // Y 축 격자 인덱스. 값이 클수록 +Y 방향.
    int j{0};
    // Z 축 격자 인덱스. 값이 클수록 +Z(위) 방향.
    int k{0};
    // 동등 비교: 세 축 인덱스가 모두 같아야 같은 셀.
    bool operator==(const Cell& o) const { return i == o.i && j == o.j && k == o.k; }
};

// =============================================================================
// Vec3 — 3차원 월드 좌표 또는 벡터 (단위: mm)
// =============================================================================
// double 정밀도 3-벡터. 좌표·치수·방향 어디에나 사용한다.
//
// 주의:
//   플랜트 도메인에서 도메인 크기가 최대 ~8,000,000mm(8km) 이므로
//   float(약 7자리) 대신 double(약 15자리) 을 써야 mm 단위 정밀도가 유지된다.
struct Vec3 {
    // X 좌표 (mm). 서-동 방향.
    double x{0.0};
    // Y 좌표 (mm). 남-북 방향.
    double y{0.0};
    // Z 좌표 (mm). 수직(높이) 방향.
    double z{0.0};
};

// =============================================================================
// AABB — 축 정렬 바운딩 박스 (Axis-Aligned Bounding Box, 단위: mm)
// =============================================================================
// lo(최소 코너)와 hi(최대 코너) 두 Vec3 로 직육면체 영역을 표현한다.
//
// 불변식:
//   lo.x < hi.x  AND  lo.y < hi.y  AND  lo.z < hi.z
//   (모든 축에서 크기가 양수여야 한다)
//   생성자가 위 조건을 검사하며, 위반 시 std::invalid_argument 를 던진다.
//
// 사용처:
//   장애물(Obstacle.min_xyz/max_xyz), 격자 원점+형상 정의,
//   점유맵 add_box(), grid_box_range() 인자
struct AABB {
    // 박스의 최소 코너 (lo.x ≤ lo.y ≤ lo.z 는 아님 — 각 축의 최소).
    Vec3 lo;
    // 박스의 최대 코너 (hi > lo 는 모든 축에서 보장됨).
    Vec3 hi;
    // 생성자: hi > lo 를 모든 축에서 검증. 위반 시 invalid_argument 예외.
    // 시간복잡도: O(1)
    AABB(const Vec3& lo_, const Vec3& hi_) : lo(lo_), hi(hi_) {
        if (hi.x <= lo.x || hi.y <= lo.y || hi.z <= lo.z)
            throw std::invalid_argument("AABB hi must be > lo");
    }
};

// =============================================================================
// NEIGHBORS_6 — 면 인접 6방향 이웃 델타 (컴파일타임 상수)
// =============================================================================
// 직교 격자에서 한 셀의 6면에 접하는 이웃 방향을 정의한다.
// 순서: +X, -X, +Y, -Y, +Z, -Z (인덱스 0~5)
//
// 왜 순서를 고정하는가?
//   A* 는 같은 f 값의 노드에 counter(삽입 순서)로 tie-break 하지만,
//   counter 가 같은 경우(같은 확장 단계에서 동시 생성된 이웃)에는
//   이웃 열거 순서가 탐색 방향을 결정한다.
//   NEIGHBORS_6 의 순서를 constexpr 로 고정하면 플랫폼·컴파일러에 무관하게
//   동일 입력 → 동일 탐색 순서 → 동일 경로가 보장된다(불변식 A2/W1).
//
// 방향 인덱스(P3o goal_dir 등에서 axis 정수로 참조):
//   0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z
inline constexpr std::array<Cell, 6> NEIGHBORS_6 = {{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
}};

// =============================================================================
// manhattan — 맨해튼 거리 (셀 단위)
// =============================================================================
// 두 셀 a, b 사이의 맨해튼(L1) 거리를 셀 수로 반환한다.
//
// 사용처:
//   A* 휴리스틱(h 값). 직교 격자에서 맨해튼 거리는 '턴 없는 최단 경로 셀 수'
//   와 동일하므로 admissible(낙관적 하한) 휴리스틱이 된다.
//   → w_heur=1.0 이면 A* 가 최적 경로를 보장(admissibility 불변식 C1).
//
// 시간복잡도: O(1)
inline int manhattan(const Cell& a, const Cell& b) {
    return std::abs(a.i - b.i) + std::abs(a.j - b.j) + std::abs(a.k - b.k);
}

// =============================================================================
// 격자 좌표 변환 유틸리티
// =============================================================================
// 아래 4개 함수는 '월드 좌표(mm) ↔ 격자 셀 인덱스' 변환의 공식 구현이다.
// Dense/Sparse/ImplicitOccupancy 세 백엔드가 모두 이 함수를 그대로 사용하여
// 좌표·복셀화 규칙의 완전한 일치를 보장한다(불변식 O1).
//
// 격자 규칙:
//   - 격자 원점(origin)은 셀 (0,0,0) 의 최소 코너(lo) 에 해당한다.
//   - 셀 (i,j,k) 의 월드 공간 범위:
//       [origin + (i,j,k)*cell_mm, origin + (i+1,j+1,k+1)*cell_mm)
//   - 셀 중심 좌표 = origin + (i+0.5, j+0.5, k+0.5) * cell_mm

// -------------------------------------------------------------------
// grid_in_bounds — 셀이 격자 범위 [0, shape) 안에 있는지 검사
// -------------------------------------------------------------------
// 인자:
//   c     — 검사할 셀 인덱스
//   shape — 격자 형상 (셀 수, 각 축)
// 반환: true = 격자 내부, false = 격자 밖 (경계 포함 외부)
// 시간복잡도: O(1)
inline bool grid_in_bounds(const Cell& c, const Cell& shape) {
    return c.i >= 0 && c.i < shape.i && c.j >= 0 && c.j < shape.j && c.k >= 0 && c.k < shape.k;
}

// -------------------------------------------------------------------
// grid_cell_to_world — 셀 중심의 월드 좌표(mm) 계산
// -------------------------------------------------------------------
// 인자:
//   c       — 대상 셀 인덱스
//   origin  — 격자 원점 (셀 (0,0,0) 의 lo 코너, mm)
//   cell_mm — 셀 크기 (mm, 예: 25.0, 50.0, 100.0)
// 반환: 셀 중심의 월드 좌표 (Vec3, mm)
//
// 공식: result.x = origin.x + (i + 0.5) * cell_mm  (y, z 도 동일)
// 시간복잡도: O(1)
//
// 왜 중심인가?
//   경로 폴리라인을 '셀 중심들의 연결' 로 표현하므로, 셀 경계(lo/hi)가
//   아닌 중심 좌표가 기하 길이 계산·렌더링의 기준점이 된다.
inline Vec3 grid_cell_to_world(const Cell& c, const Vec3& origin, double cell_mm) {
    return Vec3{origin.x + (c.i + 0.5) * cell_mm,
                origin.y + (c.j + 0.5) * cell_mm,
                origin.z + (c.k + 0.5) * cell_mm};
}

// -------------------------------------------------------------------
// grid_world_to_cell — 월드 좌표를 포함하는 셀 인덱스(floor) 계산
// -------------------------------------------------------------------
// 인자:
//   w       — 변환할 월드 좌표 (mm)
//   origin  — 격자 원점 (mm)
//   cell_mm — 셀 크기 (mm)
// 반환: w 를 포함하는 셀의 인덱스 (Cell)
//
// 공식: result.i = floor((w.x - origin.x) / cell_mm)  (y, z 도 동일)
// 시간복잡도: O(1)
//
// 주의:
//   반환된 셀 인덱스가 격자 범위 [0, shape) 안에 있다는 보장은 없다.
//   호출자가 grid_in_bounds() 로 범위를 확인해야 한다.
//   격자 밖 좌표에 대해서는 음수 인덱스나 shape 초과 인덱스가 나올 수 있다.
inline Cell grid_world_to_cell(const Vec3& w, const Vec3& origin, double cell_mm) {
    return Cell{static_cast<int>(std::floor((w.x - origin.x) / cell_mm)),
                static_cast<int>(std::floor((w.y - origin.y) / cell_mm)),
                static_cast<int>(std::floor((w.z - origin.z) / cell_mm))};
}

// =============================================================================
// CellRange — 셀 인덱스의 반열린 직육면체 구간 [lo, hi)
// =============================================================================
// 격자 내 셀들의 축정렬 범위를 표현한다. Python 의 range() 와 유사하게
// lo 는 포함, hi 는 제외(반열린)다.
//
// 사용처:
//   - grid_box_range(): AABB → 해당 격자 셀 범위
//   - 점유맵 add_box(): 박스 내 모든 셀을 block 할 때 반복 범위
//   - ImplicitOccupancy.blocked_cells(): AABB 인덱스를 셀로 열거
//
// 불변식:
//   empty() = true 이면 lo ≥ hi (축 중 하나 이상) → 반복 없음.
struct CellRange {
    // 범위의 최소 셀 (포함, inclusive).
    Cell lo;
    // 범위의 최대 셀 (제외, exclusive). hi - lo = 각 축의 셀 수.
    Cell hi;
    // 비어 있는가? (한 축이라도 lo ≥ hi 이면 true → 반복 대상 없음)
    // 시간복잡도: O(1)
    bool empty() const { return lo.i >= hi.i || lo.j >= hi.j || lo.k >= hi.k; }
};

// -------------------------------------------------------------------
// grid_box_range — AABB 가 덮는 셀 범위를 격자 [0, shape) 로 클리핑
// -------------------------------------------------------------------
// 인자:
//   box     — 클리핑 대상 AABB (mm)
//   origin  — 격자 원점 (mm)
//   cell_mm — 셀 크기 (mm)
//   shape   — 격자 형상 (셀 수)
// 반환: CellRange [lo_clipped, hi_clipped) — 격자 내부로 잘린 범위
//
// 알고리즘:
//   1. box.lo → floor 로 셀 시작 인덱스 계산
//   2. box.hi → ceil  로 셀 끝 인덱스 계산 (끝 경계 포함을 위해 ceil)
//   3. max(0, ...)  / min(shape, ...) 로 격자 범위 클리핑
//
// 왜 floor + ceil 조합인가?
//   박스가 셀 경계에 걸쳐 있을 때, lo 는 floor(가장 작은 포함 셀),
//   hi 는 ceil(경계에 닿은 셀까지 포함)로 잡아야 복셀화 과소차단이 없다.
//
// 시간복잡도: O(1)
inline CellRange grid_box_range(const AABB& box, const Vec3& origin, double cell_mm,
                                const Cell& shape) {
    auto fl = [&](double w, double o) { return static_cast<int>(std::floor((w - o) / cell_mm)); };
    auto cl = [&](double w, double o) { return static_cast<int>(std::ceil((w - o) / cell_mm)); };
    CellRange r;
    r.lo = Cell{std::max(fl(box.lo.x, origin.x), 0), std::max(fl(box.lo.y, origin.y), 0),
                std::max(fl(box.lo.z, origin.z), 0)};
    r.hi = Cell{std::min(cl(box.hi.x, origin.x), shape.i), std::min(cl(box.hi.y, origin.y), shape.j),
                std::min(cl(box.hi.z, origin.z), shape.k)};
    return r;
}

}  // namespace routing3d