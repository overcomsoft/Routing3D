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
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace routing3d {

// 정수 셀 인덱스 (i, j, k). 셀 (0,0,0) 은 origin 에서 시작.
struct Cell {
    int i{0};
    int j{0};
    int k{0};
    bool operator==(const Cell& o) const { return i == o.i && j == o.j && k == o.k; }
};

// 월드 좌표/치수 (mm).
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

// 축 정렬 바운딩 박스(AABB), 단위 mm. lo < hi (모든 축) 를 만족해야 한다.
struct AABB {
    Vec3 lo;
    Vec3 hi;
    AABB(const Vec3& lo_, const Vec3& hi_) : lo(lo_), hi(hi_) {
        if (hi.x <= lo.x || hi.y <= lo.y || hi.z <= lo.z)
            throw std::invalid_argument("AABB hi must be > lo");
    }
};

// 면 인접 6방향 이웃 (±X,±Y,±Z). 순서 고정 — A* 결정성(tie-break)에 사용.
inline constexpr std::array<Cell, 6> NEIGHBORS_6 = {{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
}};

// 두 셀의 맨해튼 거리(셀 수).
inline int manhattan(const Cell& a, const Cell& b) {
    return std::abs(a.i - b.i) + std::abs(a.j - b.j) + std::abs(a.k - b.k);
}

}  // namespace routing3d
