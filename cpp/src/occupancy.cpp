// 점유맵 구현 — occupancy.hpp 참고. 명세 algorithm_spec.md §1,§2.
#include "routing3d/occupancy.hpp"

#include <algorithm>
#include <cmath>

namespace routing3d {

DenseOccupancy::DenseOccupancy(Cell shape, Vec3 origin, double cell_mm)
    : shape_(shape), origin_(origin), cell_(cell_mm) {
    if (shape.i <= 0 || shape.j <= 0 || shape.k <= 0)
        throw std::invalid_argument("shape must be 3 positive ints");
    if (cell_mm <= 0.0)
        throw std::invalid_argument("cell_mm must be positive");
    grid_.assign(static_cast<size_t>(size()), 0);
}

bool DenseOccupancy::in_bounds(const Cell& c) const {
    return c.i >= 0 && c.i < shape_.i && c.j >= 0 && c.j < shape_.j && c.k >= 0 && c.k < shape_.k;
}

bool DenseOccupancy::is_blocked(const Cell& c) const {
    if (!in_bounds(c)) return true;  // 격자 밖 = 점유 (불변식 G1)
    return grid_[static_cast<size_t>(lin(c))] != 0;
}

void DenseOccupancy::block_cell(const Cell& c) {
    if (in_bounds(c)) grid_[static_cast<size_t>(lin(c))] = 1;
}

Vec3 DenseOccupancy::to_world(const Cell& c) const {
    return Vec3{origin_.x + (c.i + 0.5) * cell_,
               origin_.y + (c.j + 0.5) * cell_,
               origin_.z + (c.k + 0.5) * cell_};
}

Cell DenseOccupancy::to_cell(const Vec3& w) const {
    return Cell{static_cast<int>(std::floor((w.x - origin_.x) / cell_)),
                static_cast<int>(std::floor((w.y - origin_.y) / cell_)),
                static_cast<int>(std::floor((w.z - origin_.z) / cell_))};
}

int DenseOccupancy::add_box(const AABB& box) {
    // 월드 → 셀 범위 (시작=floor, 끝=ceil 제외경계), 격자로 클리핑.
    auto fl = [&](double w, double o) { return static_cast<int>(std::floor((w - o) / cell_)); };
    auto cl = [&](double w, double o) { return static_cast<int>(std::ceil((w - o) / cell_)); };
    int lo_i = std::max(fl(box.lo.x, origin_.x), 0);
    int lo_j = std::max(fl(box.lo.y, origin_.y), 0);
    int lo_k = std::max(fl(box.lo.z, origin_.z), 0);
    int hi_i = std::min(cl(box.hi.x, origin_.x), shape_.i);
    int hi_j = std::min(cl(box.hi.y, origin_.y), shape_.j);
    int hi_k = std::min(cl(box.hi.z, origin_.z), shape_.k);
    if (lo_i >= hi_i || lo_j >= hi_j || lo_k >= hi_k) return 0;
    int newly = 0;
    for (int k = lo_k; k < hi_k; ++k)
        for (int j = lo_j; j < hi_j; ++j)
            for (int i = lo_i; i < hi_i; ++i) {
                size_t idx = static_cast<size_t>(lin(Cell{i, j, k}));
                if (grid_[idx] == 0) { grid_[idx] = 1; ++newly; }
            }
    return newly;
}

long long DenseOccupancy::count_blocked() const {
    long long n = 0;
    for (uint8_t v : grid_) n += (v != 0);
    return n;
}

Cell DenseOccupancy::unlin(int idx) const {
    int i = idx % shape_.i;
    int j = (idx / shape_.i) % shape_.j;
    int k = idx / (shape_.i * shape_.j);
    return Cell{i, j, k};
}

}  // namespace routing3d
