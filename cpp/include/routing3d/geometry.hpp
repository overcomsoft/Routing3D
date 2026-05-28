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

#include <algorithm>
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

// ---- 격자 좌표 변환 (모든 점유맵 백엔드 공유 → 불변식 O1: 질의 결과 동일) ----
// 백엔드(Dense/Sparse/OpenVDB)가 이 함수들을 그대로 써서 좌표·복셀화 규칙을 일치시킨다.

// 셀이 격자 [0,shape) 안에 있는가.
inline bool grid_in_bounds(const Cell& c, const Cell& shape) {
    return c.i >= 0 && c.i < shape.i && c.j >= 0 && c.j < shape.j && c.k >= 0 && c.k < shape.k;
}

// 셀 중심의 월드좌표(mm).
inline Vec3 grid_cell_to_world(const Cell& c, const Vec3& origin, double cell_mm) {
    return Vec3{origin.x + (c.i + 0.5) * cell_mm,
                origin.y + (c.j + 0.5) * cell_mm,
                origin.z + (c.k + 0.5) * cell_mm};
}

// 월드좌표를 포함하는 셀 인덱스(floor).
inline Cell grid_world_to_cell(const Vec3& w, const Vec3& origin, double cell_mm) {
    return Cell{static_cast<int>(std::floor((w.x - origin.x) / cell_mm)),
                static_cast<int>(std::floor((w.y - origin.y) / cell_mm)),
                static_cast<int>(std::floor((w.z - origin.z) / cell_mm))};
}

// 셀 범위 [lo, hi) — 반열린 구간.
struct CellRange {
    Cell lo;
    Cell hi;
    bool empty() const { return lo.i >= hi.i || lo.j >= hi.j || lo.k >= hi.k; }
};

// AABB 가 덮는 셀 범위(시작=floor, 끝=ceil 제외경계)를 격자 [0,shape) 로 클리핑.
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
